/******************************************************************************/
/*                                                                            */
/* FILE:        kinoped.cpp                                                   */
/*                                                                            */
/* DESCRIPTION:	Code to read/write data to the PLC, ROS, and the postgres     */
/*              database for operation of the Kinoped                         */
/*                                                                            */
/* AUTHOR(S):   USA Firmware, LLC                                             */
/*                                                                            */
/* DATE:        September 30, 2020                                            */
/*                                                                            */
/* This is an unpublished work subject to Trade Secret and Copyright          */
/* protection by IHS and USA Firmware Corporation                             */
/*                                                                            */
/* USA Firmware Corporation                                                   */
/* 10060 Brecksville Road Brecksville, OH 44141                               */
/*                                                                            */
/* EDIT HISTORY:                                                              */
/*                                                                            */
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
#include <ros/ros.h>
#include <sensor_msgs/JointState.h> 
#include <tf2_msgs/TFMessage.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <stdlib.h> 
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <arpa/inet.h>
#include <signal.h>
#include <inttypes.h>
#include <math.h>
#include <time.h>
#include <pthread.h>
#include <libpq-fe.h>

#include <string>
#include <stdexcept>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <set>
#include <list>
#include <utility>

#include "kinoped.h"
#include "kinoped_msgs/user_command.h"
#include "kinoped_msgs/user_measurements.h"
#include "kinoped_msgs/kinoped_message.h"

char thisNodeType[] = { ROS_NODE_TYPE_PLC };
int sockfd, portno, n;
struct timeval tv;
char keepRunning = 1;
#if (0)
char gotoLoadReceived = 0;
char gotoHomeReceived = 0;
char debugOn = 0;
char shutdownReceived = 0;
char gotoLoadOneShot = 0;
char startReceived = 0;
char done = 0;
#endif
char ROSPublisherThread_keep_running = 1;
char ROSPublisherThreadIsRunning = 0;

char write_data_flag = 0;
unsigned char service_id = 0;
fd_set rset;
int maxfd;
int nready;
unsigned int error_count = 0;
struct sockaddr_in serv_addr;
struct sockaddr_in my_sending_port;
struct timespec ts1, ts2;
double minVal = 10000000.0;
double maxVal = 0.0;
unsigned long loopCount = 0;
double totalTime = 0.0;
int slen = sizeof(serv_addr);
struct linger sl;
char buffer[2048];
struct sigaction act;
FINS_UDP_MEMORY_AREA_READ_REQUEST memory_read_request;
FINS_NODE_ADDRESS_DATA_SEND nads;
FINS_UDP_MEMORY_AREA_READ_RESPONSE *memory_read_response;
FINS_UDP_CONTROLLER_STATUS_READ_REQUEST controller_status_request;
FINS_UDP_CONTROLLER_STATUS_READ_RESPONSE *controller_status_response;
FINS_UDP_CONTROLLER_DATA_READ_REQUEST controller_data_request;
FINS_UDP_CONTROLLER_DATA_READ_RESPONSE *controller_data_response;
FINS_UDP_CONTROLLER_CYCLE_TIME_READ_REQUEST cycle_time_request;
FINS_UDP_CONTROLLER_CYCLE_TIME_READ_RESPONSE *cycle_time_response;
FINS_UDP_MEMORY_DATA_WRITE_REQUEST memory_write_request;
FINS_UDP_MEMORY_DATA_WRITE_RESPONSE *memory_write_response;
FINS_UDP_STOP_REQUEST controller_stop_request;
FINS_UDP_STOP_RESPONSE *controller_stop_response;
FINS_UDP_RUN_REQUEST controller_run_request;
FINS_UDP_RUN_RESPONSE *controller_run_response;
int enable = 1;
int i;

pthread_t ros_publisher_thread_id;

unsigned int numBufferedItems = 0;

pthread_cond_t ros_publisher_cv = PTHREAD_COND_INITIALIZER;
pthread_mutex_t ros_publisher_lock = PTHREAD_MUTEX_INITIALIZER;

PATIENT_ROW patientData[ROWS_PER_INSERT];
PATIENT_ROW currentData;
PATIENT_ROW subscribedData;
PATIENT_ROW defaultData;
PATIENT_ROW patientPersonalData;


//unsigned int patientId = 0;
//char patientFirstName[MAX_PATIENT_NAME_LENGTH];
//char patientMiddleName[MAX_PATIENT_NAME_LENGTH];
//char patientLastName[MAX_PATIENT_NAME_LENGTH];
//char userName[MAX_USER_NAME_LENGTH];
char dbName[MAX_DATABASE_NAME_LENGTH];

sensor_msgs::JointState msg;
ros::Publisher pub;

// Status publisher
char messageDesc[128];
ros::Publisher statusPub;
kinoped_msgs::kinoped_message statusMsg;


const char *joint_names[NUM_REGULAR_JOINTS] = {
        "ar_joint",
        "br1_joint",
        "cr1_joint",
        "dr_joint",
        "er1_joint",
        "fr1_joint",
        "gr_joint",
        "hr_joint",
        "ir_joint",
        "er2_joint",
        "fr2_joint",
        "br2_joint",
        "cr2_joint",
        "al_joint",
        "bl1_joint",
        "cl1_joint",
        "dl_joint",
        "el1_joint",
        "fl1_joint",
        "gl_joint",
        "hl_joint",
        "il_joint",
        "el2_joint",
        "fl2_joint",
        "bl2_joint",
        "cl2_joint"
        };

double initialPositions[NUM_REGULAR_JOINTS] = {
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0
};
double upper_limit_degrees[NUM_REGULAR_JOINTS] = {
        10.0,  //ar_joint
        0.0,   //br1_joint
        0.0,   //cr1_joint
        0.0,   //dr1_joint
        0.0,   //er1_joint
        0.0,   //fr1_joint
        0.0,  //gr_joint
        30.0,  //hr_joint
        10.0,  //ir_joint
        0.0,   //er2_joint
        0.0,  //fr2_joint
        0.0,   //br2_joint
        0.0,   //cr2_joint
        
        30.0,   //al_joint
        0.0,   //bl1_joint
        0.0,  //cl1_joint
        0.0,  //dl_joint
        0.0,   //el1_joint
        0.0,   //fl1_joint
        0.0,   //gl_joint
        10.0,   //hl_joint
        10.0,   //il_joint
        0.0,   //el2_joint
        0.0,   //fl2_joint
        0.0,   //bl2_joint
        0.0    //cl2_joint
};
double lower_limit_degrees[NUM_REGULAR_JOINTS] = {
        -30.0,   //ar1_joint
        -95.0,   //br1_joint
        0.0,     //cr1_joint (not used)
        -95.0,   //dr_joint (unknown), same value as br1 in trace
        -88.0,   //er1_joint
        0.0,     //fr1_joint (not used)
        -88.0,   //gr_joint (unknown), same as er1_joint in trace
        -10.0,   //hr_joint
        -10.0,   //ir_joint
        -88.0,   //er2_joint (unknown), same as er1_joint in trace
        0.0,     //fr2_joint (unused)
        -95.0,   //br2_joint
        0.0,     //cr2_joint (unused)
        
        -10.0,   //al_joint
        -95.0,   //bl1_joint
        0.0,     //cl1_joint (unused)
        -95.0,   //d1_joint
        -88.0,   //el1_joint
        0.0,     //fl1_joint (unused)
        -95.0,   //gl_joint (unknown) same as bl1_joint in trace
        -30.0,   //hl_joint
        -10.0,   //il_joint
        -95.0,    //el2_joint (unknown) same as bl1_joint in trace
        0.0,     //fl2_joint (unused)
        -95.0,     //bl2_joint
        0.0      //cl2_joint (unused)
};

POSITION previous_position[NUM_REGULAR_JOINTS];
POSITION current_position[NUM_REGULAR_JOINTS];

char previous_is_valid = 0;
char debug_mode = KINOPED_DEBUG_MODE_OFF;
char run_mode = KINOPED_RUN_MODE_STOP;


/************************************************************************************************/
/*                                                                                              */
/* void publishStatus(uint16_t statusCode, uint8_t type, char* description)                     */
/*                                                                                              */
/* Publish a status message to ROS. Most likely visible via the Web UI.                         */
/*                                                                                              */
/************************************************************************************************/
void publishStatus(uint16_t statusCode, uint8_t type, const char* description)
{
    statusMsg.node = kinoped_msgs::kinoped_message::KINOPED_NODE_TYPE_PLC;
    statusMsg.code = statusCode;
    statusMsg.type = type;
    statusMsg.description = description;
    statusMsg.header.stamp = ros::Time::now();
    statusPub.publish(statusMsg);
}


#include "utils.cpp"
#include "kinopedDBDebug.cpp"
#include "callbacks.cpp"


KinopedDB *debug_data;

/*******************************************************************************************/
/*                                                                                         */
/* void umsubCallback(const kinoped_msgs::user_measurements::ConstPtr& msg)                     */
/*                                                                                         */
/* Callback for handling ROS user_measurements messages.                                   */
/*                                                                                         */
/*******************************************************************************************/
void umsubCallback(const kinoped_msgs::user_measurements::ConstPtr& msg)
{
  ROS_INFO("I heard user measurements message: height %lf", msg->height);
}


/*******************************************************************************************/
/*                                                                                         */
/* void tfMessageCallback(const tf2_msgs::TFMessage::ConstPtr& msg)                        */
/*                                                                                         */
/* Callback for handling ROS TF messages.                                                  */
/*                                                                                         */
/*******************************************************************************************/
void tfMessageCallback(const tf2_msgs::TFMessage::ConstPtr& msg)
{
    printf("\ntransforms -\n");
    for (int i = 0; i < msg->transforms.size(); i++)
    {
#if (0)
        printf("\theader:\n");
        printf("\t\tseq: %d\n",msg->transforms[i].header.seq);
        printf("\t\tstamp:\n"); 
        printf("\t\t\tsecs: %lu\n",msg->transforms[i].header.stamp.sec);
        printf("\t\t\tnsecs: %lu\n",msg->transforms[i].header.stamp.nsec);
        printf("\t\tframe_id: \"%s\"\n",msg->transforms[i].header.frame_id.c_str()); 
        printf("\tchild_frame_id: \"%s\"\n",msg->transforms[i].child_frame_id.c_str()); 
        printf("\ttransform:\n"); 
        printf("\t\ttranslation:\n"); 
        printf("\t\t\tx: %lf\n",msg->transforms[i].transform.translation.x); 
        printf("\t\t\ty: %lf\n",msg->transforms[i].transform.translation.y); 
        printf("\t\t\tz: %lf\n",msg->transforms[i].transform.translation.z); 
        printf("\t\trotation:\n"); 
        printf("\t\t\tx: %lf\n",msg->transforms[i].transform.rotation.x); 
        printf("\t\t\ty: %lf\n",msg->transforms[i].transform.rotation.y); 
        printf("\t\t\tz: %lf\n\n",msg->transforms[i].transform.rotation.z); 
        printf("\t\t\tz: %lf\n\n",msg->transforms[i].transform.rotation.w); 
#endif
        current_position[i].secs = (unsigned long)msg->transforms[i].header.stamp.sec;
        current_position[i].nsecs = (unsigned long)msg->transforms[i].header.stamp.nsec;
        strcpy(current_position[i].name, msg->transforms[i].child_frame_id.c_str());
        current_position[i].trans_x = (double)msg->transforms[i].transform.translation.x;
        current_position[i].trans_y = (double)msg->transforms[i].transform.translation.y;
        current_position[i].trans_z = (double)msg->transforms[i].transform.translation.z;
        current_position[i].rot_x = (double)msg->transforms[i].transform.rotation.x;
        current_position[i].rot_y = (double)msg->transforms[i].transform.rotation.y;
        current_position[i].rot_z = (double)msg->transforms[i].transform.rotation.z;
        current_position[i].rot_angle = (double)msg->transforms[i].transform.rotation.w;
    }
    if (previous_is_valid)
    {
//        printf("\nVELOCITIES\n");
        print_current_time_with_ms();
        double delta_time_secs;
        unsigned long diff_secs = (previous_position[0].secs - current_position[0].secs);
        unsigned long diff_nsecs;
        if (current_position[0].nsecs < previous_position[0].nsecs)
        {
            diff_nsecs = (current_position[0].nsecs + 1000000000) - previous_position[0].nsecs;
        }
        else
        {
            diff_nsecs = current_position[0].nsecs - previous_position[0].nsecs;
        }
        delta_time_secs = (double)((diff_nsecs) / 1000000000.0);
        if (previous_position[0].secs != current_position[0].secs)
        {
            printf("\tCurrent  time secs %lu  nsecs %lu\n", current_position[0].secs, current_position[0].nsecs);
            printf("\tPrevious time secs %lu  nsecs %lu\n", previous_position[0].secs, previous_position[0].nsecs);
            printf("\tdiff_nsecs %lu\n", diff_nsecs);
        }
        printf("\tdelta time: %lf seconds\n", delta_time_secs);
        for (int i = 0; i < msg->transforms.size(); i++)
        {
#if (1)
            printf("\tLink Name: %s\n", current_position[i].name);
            printf("\tVel X: %lf m/s  Delta: %lf\n", (current_position[i].trans_x -previous_position[i].trans_x) / delta_time_secs, current_position[i].trans_x - previous_position[i].trans_x);
            if (abs(((current_position[i].trans_x -previous_position[i].trans_x) / delta_time_secs)) > 0.25) printf("*** Translation X over limit ***\n");
            printf("\tVel Y: %lf m/s  Delta: %lf\n", (current_position[i].trans_y - previous_position[i].trans_y) / delta_time_secs, current_position[i].trans_y - previous_position[i].trans_y);
            if (abs(((current_position[i].trans_y -previous_position[i].trans_y) / delta_time_secs)) > 0.25) printf("*** Translation Y over limit ***\n");
            printf("\tVel Z: %lf m/s  Delta: %lf\n", (current_position[i].trans_z - previous_position[i].trans_z) / delta_time_secs, current_position[i].trans_z - previous_position[i].trans_z);
            if (abs(((current_position[i].trans_z -previous_position[i].trans_z) / delta_time_secs)) > 0.25) printf("*** Translation Z over limit ***\n");
            printf("\tRot X: %lf m/s  Delta: %lf\n", (current_position[i].rot_x - previous_position[i].rot_x) / delta_time_secs, current_position[i].rot_x - previous_position[i].rot_x);
            if (abs(((current_position[i].rot_x -previous_position[i].rot_x) / delta_time_secs)) > 0.25) printf("*** Rotation X over limit ***\n");
            printf("\tRot Y: %lf m/s  Delta: %lf\n", (current_position[i].rot_y - previous_position[i].rot_y) / delta_time_secs, current_position[i].rot_y - previous_position[i].rot_y);
            if (abs(((current_position[i].rot_y -previous_position[i].rot_y) / delta_time_secs)) > 0.25) printf("*** Rotation Y over limit ***\n");
            printf("\tRot Z: %lf m/s  Delta: %lf\n", (current_position[i].rot_z - previous_position[i].rot_z) / delta_time_secs, current_position[i].trans_z - previous_position[i].rot_z);
            if (abs(((current_position[i].rot_z -previous_position[i].rot_z) / delta_time_secs)) > 0.25) printf("*** Rotation Z over limit ***\n");
            printf("\tRot Angle: %lf m/s  Delta: %lf\n", (current_position[i].rot_angle - previous_position[i].rot_angle) / delta_time_secs, current_position[i].rot_angle - previous_position[i].rot_angle);
            if (abs(((current_position[i].rot_angle -previous_position[i].rot_angle) / delta_time_secs)) > 0.25) printf("*** Rotation Angle over limit ***\n");
            printf("\n");
//            // save current position as previous
//            memcpy(&previous_position[i], &current_position[i], sizeof(POSITION));
#endif
        }
        // save current position as previous
        memcpy(&previous_position[0], &current_position[0], msg->transforms.size() * sizeof(POSITION));
    }
    else
    {
        memcpy(&previous_position[0], &current_position[0], msg->transforms.size() * sizeof(POSITION));
        previous_is_valid = 1;
    }
}

/*******************************************************************************************/
/*                                                                                         */
/* void intHandler(int x)                                                                  */
/*                                                                                         */
/* Interrupt handler callback when control-C is pressed. This was used when NOT running    */
/* under ROS, since it already has its own handler. Kept for posterity in case this code   */
/* needs to be run under Linux by itself.                                                  */
/*                                                                                         */
/*******************************************************************************************/
void intHandler(int x)
{
    keepRunning = 0;
    ROSPublisherThread_keep_running = 0;
    run_mode = KINOPED_RUN_MODE_END;
}

/*******************************************************************************************/
/*                                                                                         */
/* void error(const char *mesg)                                                            */
/*                                                                                         */
/* Error handler callback. Print an error message and exit.                                */
/*                                                                                         */
/*******************************************************************************************/
void error(const char *mesg)
{
    perror(mesg);
    exit(0);
}

/************************************************************************************************/
/*                                                                                              */
/* void gotoHomePosition()                                                                      */
/*                                                                                              */
/* Moves both legs to the "home" position (fully retracted)                                     */
/*                                                                                              */
/************************************************************************************************/
void gotoHomePosition()
{
    // Build ROS data for the publisher
    for (int i = 0; i < NUM_REGULAR_JOINTS; i++)
    {
        switch (i)
        {
            case CR1_JOINT_INDEX:
            case CR2_JOINT_INDEX:
            case FR1_JOINT_INDEX:
            case FR2_JOINT_INDEX:
            case CL1_JOINT_INDEX:
            case CL2_JOINT_INDEX:
            case FL1_JOINT_INDEX:
            case FL2_JOINT_INDEX:
                msg.position[i]  = patientPersonalData.joints[i].initial_position_meters;
                break;
            default:
                msg.position[i]  = patientPersonalData.joints[i].initial_position_radians;
                break;
        }
        msg.velocity[i]  = defaultData.joints[i].velocity;
        msg.effort[i]    = defaultData.joints[i].effort;
    }
    msg.header.frame_id = "kinoped";
    msg.header.stamp = ros::Time::now();

    // publish this instances' data to ROS
    pthread_cond_signal(&ros_publisher_cv);
}

/************************************************************************************************/
/*                                                                                              */
/* void gotoInitialPose()                                                                       */
/*                                                                                              */
/* Moves both legs to the "initial pose" position (loading position)                            */
/*                                                                                              */
/************************************************************************************************/
void gotoInitialPose()
{
    // Build ROS data for the publisher
    for (int i = 0; i < NUM_REGULAR_JOINTS; i++)
    {
        switch (i)
        {
            case CR1_JOINT_INDEX:
            case CR2_JOINT_INDEX:
            case FR1_JOINT_INDEX:
            case FR2_JOINT_INDEX:
            case CL1_JOINT_INDEX:
            case CL2_JOINT_INDEX:
            case FL1_JOINT_INDEX:
            case FL2_JOINT_INDEX:
//                msg.position[i]  = defaultData.joints[i].initial_position_meters;
                msg.position[i]  = -0.20;
                break;
            case BL1_JOINT_INDEX:
            case BR1_JOINT_INDEX:
            case BL2_JOINT_INDEX:
            case BR2_JOINT_INDEX:
                msg.position[i]  = -1.00;
                break;
            case EL1_JOINT_INDEX:
            case ER1_JOINT_INDEX:
            case EL2_JOINT_INDEX:
            case ER2_JOINT_INDEX:
                msg.position[i]  = -1.54;
                break;
            default:
//                msg.position[i]  = defaultData.joints[i].initial_position_radians;
                msg.position[i]  = 0.0;
                break;
        }
        msg.velocity[i]  = defaultData.joints[i].velocity;
        msg.effort[i]    = defaultData.joints[i].effort;
    }
    msg.header.frame_id = "kinoped";
    msg.header.stamp = ros::Time::now();

    // publish this instances' data to ROS
    pthread_cond_signal(&ros_publisher_cv);
}

/************************************************************************************************/
/*                                                                                              */
/* void init_data()                                                                             */
/*                                                                                              */
/* Initializes the outgoing ethernet packets and other necessary data                           */
/*                                                                                              */
/************************************************************************************************/
void init_data()
{
    //Sets up the random number generator
    srand(time(0));
    
    //patientId = 1;
    //strcpy(patientFirstName, "John");
    //strcpy(patientMiddleName, "Q.");
    //strcpy(patientLastName, "Public");
    //strcpy(userName, "jpublic");
    strcpy(dbName, "postgres");
    
    memset(&previous_position[0], 0, NUM_REGULAR_JOINTS * sizeof(POSITION));
    memset(&current_position[0], 0, NUM_REGULAR_JOINTS * sizeof(POSITION));
    memset(&defaultData, 0, sizeof(PATIENT_ROW));
    memset(&patientPersonalData, 0, sizeof(PATIENT_ROW));
    

    for (int i = 0; i < NUM_REGULAR_JOINTS; i++)
    {
        msg.name.push_back(joint_names[i]);
        msg.position.push_back(0.0);
        msg.velocity.push_back(0.0);
        msg.effort.push_back(0.0);
        strcpy(defaultData.joints[i].name, joint_names[i]);
        switch (i)
        {
            case CR1_JOINT_INDEX:
            case CR2_JOINT_INDEX:
            case FR1_JOINT_INDEX:
            case FR2_JOINT_INDEX:
            case CL1_JOINT_INDEX:
            case CL2_JOINT_INDEX:
            case FL1_JOINT_INDEX:
            case FL2_JOINT_INDEX:
                defaultData.joints[i].initial_position_meters = inchesToMeters(initialPositions[i]);
                defaultData.joints[i].upper_limit_meters = inchesToMeters(0.0);
                defaultData.joints[i].lower_limit_meters = inchesToMeters(-8.0);
                break;
            default:
                defaultData.joints[i].initial_position_radians = degreesToRadians(initialPositions[i]);
                defaultData.joints[i].upper_limit_radians = degreesToRadians(upper_limit_degrees[i]);
                defaultData.joints[i].lower_limit_radians = degreesToRadians(lower_limit_degrees[i]);
                break;
        }
        defaultData.joints[i].velocity = 0.0;
        defaultData.joints[i].effort = 0.0;
        
    }
    memcpy(&patientPersonalData, &defaultData, sizeof(PATIENT_ROW));
    
    memset(&controller_stop_request, 0, sizeof(FINS_UDP_STOP_REQUEST));
    controller_stop_request.hdr.icf = ICF;
    controller_stop_request.hdr.reserved = 0x00;
    controller_stop_request.hdr.gateway_count = GATEWAY_COUNT;
    controller_stop_request.hdr.destination_network_address = NETWORK_ADDRESS_LOCAL;
    controller_stop_request.hdr.destination_node_number = SYSMAC_NET;
    controller_stop_request.hdr.destination_unit_address = UNIT_ADDRESS_PC;
    controller_stop_request.hdr.source_network_address = NETWORK_ADDRESS_LOCAL;
    controller_stop_request.hdr.source_node_number = UNKNOWN_NODE_NUMBER;
    controller_stop_request.hdr.source_unit_address = UNIT_ADDRESS_PC;
    controller_stop_request.command_code = htons(COMMAND_STOP);

    memset(&controller_run_request, 0, sizeof(FINS_UDP_RUN_REQUEST));
    controller_run_request.hdr.icf = ICF;
    controller_run_request.hdr.reserved = 0x00;
    controller_run_request.hdr.gateway_count = GATEWAY_COUNT;
    controller_run_request.hdr.destination_network_address = NETWORK_ADDRESS_LOCAL;
    controller_run_request.hdr.destination_node_number = SYSMAC_NET;
    controller_run_request.hdr.destination_unit_address = UNIT_ADDRESS_PC;
    controller_run_request.hdr.source_network_address = NETWORK_ADDRESS_LOCAL;
    controller_run_request.hdr.source_node_number = UNKNOWN_NODE_NUMBER;
    controller_run_request.hdr.source_unit_address = UNIT_ADDRESS_PC;
    controller_run_request.command_code = htons(COMMAND_RUN);
    controller_run_request.program_number = htons(0xFFFF);
    controller_run_request.mode = FINS_CPU_MODE_RUN;

    memset(&memory_write_request, 0, sizeof(FINS_UDP_MEMORY_DATA_WRITE_REQUEST));
    memory_write_request.hdr.icf = ICF;
    memory_write_request.hdr.reserved = 0x00;
    memory_write_request.hdr.gateway_count = GATEWAY_COUNT;
    memory_write_request.hdr.destination_network_address = NETWORK_ADDRESS_LOCAL;
    memory_write_request.hdr.destination_node_number = SYSMAC_NET;
    memory_write_request.hdr.destination_unit_address = UNIT_ADDRESS_PC;
    memory_write_request.hdr.source_network_address = NETWORK_ADDRESS_LOCAL;
    memory_write_request.hdr.source_node_number = UNKNOWN_NODE_NUMBER;
    memory_write_request.hdr.source_unit_address = UNIT_ADDRESS_PC;
    memory_write_request.command_code = htons(COMMAND_MEMORY_WRITE_AREA);
    memory_write_request.memory_area_code = MEMORY_AREA_DM_WORD_CONTENTS;
    memory_write_request.beginning_address = htons(BEGINNING_ADDRESS);
    memory_write_request.num_items = htons(NUM_ITEMS);
    
    memset(&cycle_time_request, 0, sizeof(FINS_UDP_CONTROLLER_CYCLE_TIME_READ_REQUEST));
    cycle_time_request.hdr.icf = ICF;
    cycle_time_request.hdr.reserved = 0x00;
    cycle_time_request.hdr.gateway_count = GATEWAY_COUNT;
    cycle_time_request.hdr.destination_network_address = NETWORK_ADDRESS_LOCAL;
    cycle_time_request.hdr.destination_node_number = SYSMAC_NET;
    cycle_time_request.hdr.destination_unit_address = UNIT_ADDRESS_PC;
    cycle_time_request.hdr.source_network_address = NETWORK_ADDRESS_LOCAL;
    cycle_time_request.hdr.source_node_number = UNKNOWN_NODE_NUMBER;
    cycle_time_request.hdr.source_unit_address = UNIT_ADDRESS_PC;
    cycle_time_request.command = htons(COMMAND_CONTROLLER_CYCLE_TIME_READ);
    cycle_time_request.parameter = CYCLE_TIME_PARAMETER_READ; 
    
    memset(&controller_status_request, 0, sizeof(FINS_UDP_CONTROLLER_STATUS_READ_REQUEST));
    controller_status_request.hdr.icf = ICF;
    controller_status_request.hdr.reserved = 0x00;
    controller_status_request.hdr.gateway_count = GATEWAY_COUNT;
    controller_status_request.hdr.destination_network_address = NETWORK_ADDRESS_LOCAL;
    controller_status_request.hdr.destination_node_number = SYSMAC_NET;
    controller_status_request.hdr.destination_unit_address = UNIT_ADDRESS_PC;
    controller_status_request.hdr.source_network_address = NETWORK_ADDRESS_LOCAL;
    controller_status_request.hdr.source_node_number = UNKNOWN_NODE_NUMBER;
    controller_status_request.hdr.source_unit_address = UNIT_ADDRESS_PC;
    controller_status_request.command = htons(COMMAND_CONTROLLER_STATUS_READ);
    
    memset(&controller_data_request, 0, sizeof(FINS_UDP_CONTROLLER_DATA_READ_REQUEST));
    controller_data_request.hdr.icf = ICF;
    controller_data_request.hdr.reserved = 0x00;
    controller_data_request.hdr.gateway_count = GATEWAY_COUNT;
    controller_data_request.hdr.destination_network_address = NETWORK_ADDRESS_LOCAL;
    controller_data_request.hdr.destination_node_number = SYSMAC_NET;
    controller_data_request.hdr.destination_unit_address = UNIT_ADDRESS_PC;
    controller_data_request.hdr.source_network_address = NETWORK_ADDRESS_LOCAL;
    controller_data_request.hdr.source_node_number = UNKNOWN_NODE_NUMBER;
    controller_data_request.hdr.source_unit_address = UNIT_ADDRESS_PC;
    controller_data_request.command = htons(COMMAND_CONTROLLER_DATA_READ);

    memset(&nads, 0, sizeof(FINS_NODE_ADDRESS_DATA_SEND));
    nads.magic[0] = 'F';
    nads.magic[1] = 'I';
    nads.magic[2] = 'N';
    nads.magic[3] = 'S';
    nads.length = htonl(12);
    nads.command = htonl(NODE_ADDRESS_DATA_SEND);
    nads.error_code = htonl(ERROR_CODE_NORMAL);
    nads.node_address = htonl(UNIT_ADDRESS_PC);

    memset(&memory_read_request, 0, sizeof(FINS_UDP_MEMORY_AREA_READ_REQUEST));
    // this magic value in the header is only needed if you want to run FINS using TCP/IP instead of UDP
    //memory_read_request.hdr.magic[0] = 'F';
    //memory_read_request.hdr.magic[1] = 'I';
    //memory_read_request.hdr.magic[2] = 'N';
    //memory_read_request.hdr.magic[3] = 'S';
//    memory_read_request.hdr.length = htonl(sizeof(FINS_UDP_HEADER));
    memory_read_request.hdr.icf = ICF;
    memory_read_request.hdr.reserved = 0x00;
    memory_read_request.hdr.gateway_count = GATEWAY_COUNT;
    memory_read_request.hdr.destination_network_address = NETWORK_ADDRESS_LOCAL;
    memory_read_request.hdr.destination_node_number = SYSMAC_NET;
    memory_read_request.hdr.destination_unit_address = UNIT_ADDRESS_PC;
    memory_read_request.hdr.source_network_address = NETWORK_ADDRESS_LOCAL;
    memory_read_request.hdr.source_node_number = UNKNOWN_NODE_NUMBER;
    memory_read_request.hdr.source_unit_address = UNIT_ADDRESS_PC;

    memory_read_request.cmd.command = htons(COMMAND_MEMORY_READ_AREA);
    memory_read_request.cmd.memory_area = MEMORY_AREA_DM_WORD_CONTENTS;
    memory_read_request.cmd.beginning_address = htons(BEGINNING_ADDRESS);
    memory_read_request.cmd.beginning_address_bits = BEGINNING_ADDRESS_BITS;
    memory_read_request.cmd.num_items = htons(NUM_ITEMS);
}

/************************************************************************************************/
/*                                                                                              */
/* void read_cycle_time()                                                                       */
/*                                                                                              */
/* Issues the cycle time read request and gets the response from the PLC using FINS/UDP         */
/*                                                                                              */
/************************************************************************************************/
void read_cycle_time()
{
    memset(&buffer, 0, sizeof(buffer));
    cycle_time_request.hdr.service_id = service_id++;
    n = sendto(sockfd, &cycle_time_request, sizeof(FINS_UDP_CONTROLLER_CYCLE_TIME_READ_REQUEST), 0, (struct sockaddr *) &serv_addr, sizeof(serv_addr));
    n = recvfrom(sockfd, buffer, sizeof(buffer), 0, (struct sockaddr *)&serv_addr, (socklen_t *)&slen);
    printf("Response to FINS_UDP_CONTROLLER_CYCLE_TIME_READ_REQUEST received %d bytes\n", n);

    cycle_time_response = (FINS_UDP_CONTROLLER_CYCLE_TIME_READ_RESPONSE *)buffer;
    cycle_time_response->average = ntohl(cycle_time_response->average);
    cycle_time_response->max = ntohl(cycle_time_response->max);
    cycle_time_response->min = ntohl(cycle_time_response->min);
    printf("Cycle time data:\n");
    printf("Avg: %0.1lfms  Max: %0.1lfms  Min: %0.1lfms\n", (double)cycle_time_response->average * 0.1, (double)cycle_time_response->max * 0.1, (double)cycle_time_response->min * 0.1);
}

/************************************************************************************************/
/*                                                                                              */
/* void read_controller_data()                                                                  */
/*                                                                                              */
/* Issues the controller data read request and gets the response from the PLC using FINS/UDP    */
/*                                                                                              */
/************************************************************************************************/
void read_controller_data()
{
    memset(&buffer, 0, sizeof(buffer));
    controller_data_request.hdr.service_id = service_id++;
    n = sendto(sockfd, &controller_data_request, sizeof(FINS_UDP_CONTROLLER_DATA_READ_REQUEST), 0, (struct sockaddr *) &serv_addr, sizeof(serv_addr));
    n = recvfrom(sockfd, buffer, sizeof(buffer), 0, (struct sockaddr *)&serv_addr, (socklen_t *)&slen);
    printf("Response to FINS_UDP_CONTROLLER_DATA_READ_REQUEST received %d bytes\n", n);

    controller_data_response = (FINS_UDP_CONTROLLER_DATA_READ_RESPONSE *)buffer;
    printf("Controller data:\n");
    printf("model = \"%s\"\n", controller_data_response->controller_model);
    printf("version = \"%s\"\n", controller_data_response->controller_version);
    printf("Raw byte data: ");
    for (i = 0; i < sizeof(controller_data_response->area_data.byte_data); i++)
    {
        printf("0x%02X ", (uint8_t)controller_data_response->area_data.byte_data[i]);
    }
    printf("\n");
    printf("program area size = 0x%02X\n", controller_data_response->area_data.data.program_area_size);
    printf("num_dm_words = 0x%04X\n", ntohs(controller_data_response->area_data.data.num_dm_words));
    printf("kind_of_file_memory = 0x%02X\n", controller_data_response->area_data.data.kind_of_file_memory);
    printf("file_memory_size = 0x%04X\n", ntohs(controller_data_response->area_data.data.file_memory_size));
    printf("pc_status = 0x%02X\n", controller_data_response->pc_status);
}

/************************************************************************************************/
/*                                                                                              */
/* void read_controller_status()                                                                */
/*                                                                                              */
/* Issues the controller status read request and gets the response from the PLC using FINS/UDP  */
/*                                                                                              */
/************************************************************************************************/
void read_controller_status()
{
    memset(&buffer, 0, sizeof(buffer));

    controller_status_request.hdr.service_id = service_id++;
    n = sendto(sockfd, &controller_status_request, sizeof(FINS_UDP_CONTROLLER_STATUS_READ_REQUEST), 0, (struct sockaddr *) &serv_addr, sizeof(serv_addr));
    n = recvfrom(sockfd, buffer, sizeof(buffer), 0, (struct sockaddr *)&serv_addr, (socklen_t *)&slen);
    printf("Response to FINS_UDP_CONTROLLER_STATUS_READ_REQUEST received %d bytes\n", n);

    controller_status_response = (FINS_UDP_CONTROLLER_STATUS_READ_RESPONSE *)buffer;
    printf("Controller status:\n");
    // swap necessary values
    controller_status_response->command_code = ntohs(controller_status_response->command_code);
    controller_status_response->response_code = ntohs(controller_status_response->response_code);
    controller_status_response->fatal_error.int_value = ntohs(controller_status_response->fatal_error.int_value);
    controller_status_response->non_fatal_error.int_value = ntohs(controller_status_response->non_fatal_error.int_value);
    controller_status_response->fals_number = ntohs(controller_status_response->fals_number);
    printf("command_code = 0x%04X\n", controller_status_response->command_code);
    printf("response_code = 0x%04X\n", controller_status_response->response_code);
    printf("status = 0x%02X\n", controller_status_response->status);
    printf("mode = 0x%02X\n", controller_status_response->mode);
    printf("fatal_error = 0x%04X\n", controller_status_response->fatal_error.int_value);
    printf("fatal_error.memory_error = %d\n", controller_status_response->fatal_error.memory_error);
//            uint16_t memory_error      : 1,
//                 cpu_bus_error     : 1,
//                 reserved2         : 2,
//                 io_point_overflow : 1,
//                 io_setting_error  : 1,
//                 reserved3         : 3,
//                 fals_system_error : 1,
//                 reserved6         : 6;

    printf("non_fatal_error = %04X\n", controller_status_response->non_fatal_error.int_value);
    printf("fals_number = %04X\n", controller_status_response->fals_number);
    printf("error_message = \"%s\"\n", controller_status_response->error_message);
    switch (controller_status_response->mode)
    {
        case FINS_CPU_MODE_PROGRAM:
            printf("Run Mode: Program\n");
            break;
        case FINS_CPU_MODE_MONITOR:
            printf("Run Mode: Monitor\n");
            break;
        case FINS_CPU_MODE_RUN:
            printf("Run Mode: Run\n");
            break;
    }
}

/************************************************************************************************/
/*                                                                                              */
/* void controller_stop()                                                                       */
/*                                                                                              */
/* Issues the controller stop request and gets the response from the PLC using FINS/UDP         */
/*                                                                                              */
/************************************************************************************************/
void controller_stop()
{
    memset(&buffer, 0, sizeof(buffer));

    controller_stop_request.hdr.service_id = service_id++;
    n = sendto(sockfd, &controller_stop_request, sizeof(FINS_UDP_STOP_REQUEST), 0, (struct sockaddr *) &serv_addr, sizeof(serv_addr));
    n = recvfrom(sockfd, buffer, sizeof(buffer), 0, (struct sockaddr *)&serv_addr, (socklen_t *)&slen);
    printf("Response to FINS_UDP_STOP_REQUEST received %d bytes\n", n);

    controller_stop_response = (FINS_UDP_STOP_RESPONSE *)buffer;
    printf("Controller stop status:\n");
    // swap necessary values
    controller_stop_response->command_code = ntohs(controller_stop_response->command_code);
    controller_stop_response->response_code = ntohs(controller_stop_response->response_code);
    printf("Stop command code: 0x%04X  response code: 0x%04X\n", controller_stop_response->command_code, controller_stop_response->response_code);
}

/************************************************************************************************/
/*                                                                                              */
/* void controller_run()                                                                        */
/*                                                                                              */
/* Issues the controller run request and gets the response from the PLC using FINS/UDP          */
/*                                                                                              */
/************************************************************************************************/
void controller_run()
{
    memset(&buffer, 0, sizeof(buffer));

    controller_run_request.hdr.service_id = service_id++;
    n = sendto(sockfd, &controller_run_request, sizeof(FINS_UDP_RUN_REQUEST), 0, (struct sockaddr *) &serv_addr, sizeof(serv_addr));
    n = recvfrom(sockfd, buffer, sizeof(buffer), 0, (struct sockaddr *)&serv_addr, (socklen_t *)&slen);
    printf("Response to FINS_UDP_RUN_REQUEST received %d bytes\n", n);

    controller_run_response = (FINS_UDP_RUN_RESPONSE *)buffer;
    printf("Controller run status:\n");
    // swap necessary values
    controller_run_response->command_code = ntohs(controller_run_response->command_code);
    controller_run_response->response_code = ntohs(controller_run_response->response_code);
    printf("Run command code: 0x%04X  response code: 0x%04X\n", controller_run_response->command_code, controller_run_response->response_code);
}


/************************************************************************************************/
/*                                                                                              */
/* void *ros_publisher_thread(void *arg)                                                        */
/*                                                                                              */
/* Publishes the currently received data from the PLC to ROS, when signalled.                   */
/*                                                                                              */
/************************************************************************************************/
void *ros_publisher_thread(void *arg)
{
    ROSPublisherThreadIsRunning = 1;
    printf("ros_publisher_thread started\n");
    while (ROSPublisherThread_keep_running)
    {
        pthread_mutex_lock(&ros_publisher_lock);
        pthread_cond_wait(&ros_publisher_cv, &ros_publisher_lock);
        //Publish the message
        pub.publish(msg);
        pthread_mutex_unlock(&ros_publisher_lock);
    }
    printf("End of ros_publisher_thread\n");
    ROSPublisherThreadIsRunning = 0;
    return(NULL);
}

/*******************************************************************************************/
/*                                                                                         */
/* void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)                   */
/*                                                                                         */
/* Callback for handling ROS JointState messages.                                          */
/*                                                                                         */
/*******************************************************************************************/
void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    subscribedData.dataSource = DATA_SOURCE_ROS;
    subscribedData.command = COMMAND_MEMORY_WRITE_AREA;
    for (int i = 0; i < NUM_REGULAR_JOINTS; i++)
    {
        subscribedData.joints[i].current_value = msg->position[i];
        subscribedData.joints[i].velocity = msg->velocity[i];
        subscribedData.joints[i].effort = msg->effort[i];
    }
    write_data_flag = 1;
}

/************************************************************************************************/
/*                                                                                              */
/* int main(int argc, char **argv)                                                              */
/*                                                                                              */
/* The main processing loop. Invokes worker threads, calls the database initialization, and     */
/* sends and receives FINS messages to the PLC. Publishes messages to ROS, and subscribes to    */
/* ROS messages for sending of data to the PLC.                                                 */
/*                                                                                              */
/************************************************************************************************/
int main(int argc, char **argv)
{
#if (0)
    double pulse_count = 0.0;
    double degrees = 0.0;
    double radians = 0.0;
#endif
    int portno = OMRON_FINS_PORT;
    char ip[] = OMRON_PLC_IP_ADDRESS;
    unsigned long totalRecords = 0;
	char newDBName[128];
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    int publishThrottle = 0;

    // register Control-C handler
    act.sa_handler = intHandler;
    sigaction(SIGINT, &act, NULL);

     //Sets up the random number generator
    srand(time(NULL));

    init_data();
     //Initializes ROS, and sets up a node
    ros::init(argc, argv, "kinoped_plc_node");
    ros::NodeHandle nh;

    // we need to use AsyncSpinner in order to have multiple subscriber callbacks work correctly
    ros::AsyncSpinner spinner(0);
    spinner.start();

     //Ceates the publisher, and tells it to publish
     //to the joint_states topic, with a queue size of 1000
    pub=nh.advertise<sensor_msgs::JointState>("joint_states", 1000);
//    ros::Subscriber subJointState = nh.subscribe("kinoped", 1000, jointStateCallback);
//    ros::Subscriber subTFMessage = nh.subscribe("tf", 1000, tfMessageCallback);
    ros::Subscriber subUserCommand = nh.subscribe("user_command", 1000, ucsubCallback);
    ros::Subscriber subUserMeasurements = nh.subscribe("user_measurements", 1000, umsubCallback);
    statusPub = nh.advertise<kinoped_msgs::kinoped_message>("kinoped_status", 1000);

     //Sets the loop to publish at a rate of 100Hz
    ros::Rate rate(100);

    sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sockfd < 0)
    { 
        sprintf(messageDesc, "ERROR opening socket");
        printf("%s\n", messageDesc);
        publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_GENERAL_ERROR, kinoped_msgs::kinoped_message::KINOPED_MSG_FATAL, messageDesc);
        error("ERROR opening socket");
    }
    if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0)
    {
        sprintf(messageDesc, "setsockopt(SO_REUSEADDR) failed");
        printf("%s\n", messageDesc);
        publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_GENERAL_ERROR, kinoped_msgs::kinoped_message::KINOPED_MSG_FATAL, messageDesc);
        error("setsockopt(SO_REUSEADDR) failed");
    }
    
    sl.l_onoff = 1;		/* non-zero value enables linger option in kernel */
    sl.l_linger = 0;	/* timeout interval in seconds */
    setsockopt(sockfd, SOL_SOCKET, SO_LINGER, &sl, sizeof(sl));
#if (0)
    tv.tv_sec = 0;
    tv.tv_usec = TWO_HUNDRED_FIFTY_MS;
    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));
#endif
    memset(&my_sending_port, 0, sizeof(my_sending_port));
    memset(&serv_addr, 0, sizeof(serv_addr));
    
    serv_addr.sin_addr.s_addr = inet_addr(ip);
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(portno);
    printf("IP = %s  Port = %d\n", ip, portno);
    
    my_sending_port.sin_family = AF_INET;
    my_sending_port.sin_port = htons(KINOPED_PORT);
    my_sending_port.sin_addr.s_addr = INADDR_ANY;

    // clear the descriptor set 
    FD_ZERO(&rset); 

    // get maxfd 
    maxfd = sockfd + 1; 
    
    /* Assign a port number to the outbound socket */
    if (bind(sockfd, (struct sockaddr*)&my_sending_port, sizeof(my_sending_port)) != 0)
    {
        sprintf(messageDesc, "Socket bind error - output socket");
        printf("%s\n", messageDesc);
        publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_GENERAL_ERROR, kinoped_msgs::kinoped_message::KINOPED_MSG_FATAL, messageDesc);
        perror("Socket bind error - output socket");
        exit(-1);
    }
    bzero(buffer,sizeof(buffer));

    // read cycle time
    read_cycle_time();

    // read controller data
    read_controller_data();

    // read controller status
    read_controller_status();

    if( pthread_create(&ros_publisher_thread_id, NULL, ros_publisher_thread, 0))
    {
        printf( "Fail...Cannot spawn the ROS publisher thread.\n");
        exit(-1);
    }

    memset(&buffer, 0, sizeof(buffer));
    memory_read_request.hdr.service_id = service_id++;
    n = sendto(sockfd, &memory_read_request, sizeof(FINS_UDP_MEMORY_AREA_READ_REQUEST), 0, (struct sockaddr *) &serv_addr, sizeof(serv_addr));
    n = recvfrom(sockfd, buffer, sizeof(buffer), 0, (struct sockaddr *)&serv_addr, (socklen_t *)&slen);

    memory_read_response = (FINS_UDP_MEMORY_AREA_READ_RESPONSE *)buffer;
    for (i = 0; i < NUM_ITEMS; i++)
    {
        memory_write_request.data[i] = memory_read_response->data[i];
    }
    memset(&buffer, 0, sizeof(buffer));
    memory_write_request.hdr.service_id = service_id++;
    n = sendto(sockfd, &memory_write_request, sizeof(FINS_UDP_MEMORY_DATA_WRITE_REQUEST), 0, (struct sockaddr *) &serv_addr, sizeof(serv_addr));
    n = recvfrom(sockfd, buffer, sizeof(buffer), 0, (struct sockaddr *)&serv_addr, (socklen_t *)&slen);
    memory_write_response = (FINS_UDP_MEMORY_DATA_WRITE_RESPONSE *)buffer;
    printf("Memory write command code: 0x%04X  response code: 0x%04X\n", ntohs(memory_write_response->command_code), ntohs(memory_write_response->response_code));
 
    while (keepRunning && ros::ok() && (KINOPED_RUN_MODE_END != run_mode))
    {
        publishThrottle = 0;
        while (!gotoLoadReceived && (KINOPED_RUN_MODE_END != run_mode))
        {
            if (0 == (publishThrottle % 10))
            {
                sprintf(messageDesc, "kinoped - waiting for goto Load message");
                printf("%s\n", messageDesc);
			    publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_WAITING_FOR_GOTO_LOAD_COMMAND, kinoped_msgs::kinoped_message::KINOPED_MSG_INFO, messageDesc);
            }
            publishThrottle++;
            sleep(1);
            if (KINOPED_RUN_MODE_END == run_mode)
            {
                sprintf(messageDesc, "kinoped - end command received");
			    printf("%s\n", messageDesc);
			    publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_END_RECEIVED, kinoped_msgs::kinoped_message::KINOPED_MSG_INFO, messageDesc);
                keepRunning = 0;
//                exit(0);
            }
        }
        publishThrottle = 0;
        while (!startReceived && (KINOPED_RUN_MODE_END != run_mode))
        {
            if (gotoLoadOneShot)
            {
                 gotoInitialPose();
                 gotoLoadOneShot = 0;
            }
            if (0 == (publishThrottle %10))
            {
                sprintf(messageDesc, "kinoped - waiting for Start message");
                printf("%s\n", messageDesc);
			    publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_WAITING_FOR_START_COMMAND, kinoped_msgs::kinoped_message::KINOPED_MSG_INFO, messageDesc);
            }
            publishThrottle++;
            sleep(1);
            if (KINOPED_RUN_MODE_END == run_mode)
            {
                sprintf(messageDesc, "kinoped - end command received");
			    printf("%s\n", messageDesc);
			    publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_END_RECEIVED, kinoped_msgs::kinoped_message::KINOPED_MSG_INFO, messageDesc);
                keepRunning = 0;
//                exit(0);
            }
        }
        if (startReceived)
        {
            // create db
	        char c1[] = {"postgres"};
            char c2[] = {POSTGRES_DB_TABLE_NAME_USER_TRACKING_DEBUG};
            char c3[] = {DROP_DEBUG_TABLE};
            char c4[] = {CREATE_DEBUG_TABLE};
            char c5[] = {DEBUG_ROW_INSERT_STRING};
            debug_data = new KinopedDB((char*)dbName, (char*)c1, (char*)c2, (char*)c3, (char*)c4, (char*)c5);
            debug_data->init_database_connection();

            while (keepRunning && (KINOPED_RUN_MODE_STOP != run_mode) && (KINOPED_RUN_MODE_END != run_mode))
            {
                if ((KINOPED_RUN_MODE_START == run_mode) || (KINOPED_RUN_MODE_RUN == run_mode))
                {
                   // skip this if we're paused, but do it if start or run mode
                    loopCount++;
                    bzero(buffer,sizeof(buffer));
                    clock_gettime(CLOCK_REALTIME, &ts1);
                    // bump sequence number per packet
                    memory_read_request.hdr.service_id = service_id++;
                    n = sendto(sockfd, &memory_read_request, sizeof(FINS_UDP_MEMORY_AREA_READ_REQUEST), 0, (struct sockaddr *) &serv_addr, sizeof(serv_addr));
                    n = recvfrom(sockfd, buffer, sizeof(buffer), 0, (struct sockaddr *)&serv_addr, (socklen_t *)&slen);
                    clock_gettime(CLOCK_REALTIME, &ts2);
                    if (ts2.tv_nsec < ts1.tv_nsec)
                    {
                        ts2.tv_nsec += 1000000000;
                        ts2.tv_sec--;
                    }
                    printf("Elapsed read time %0.3lfms\n", (double)(ts2.tv_nsec - ts1.tv_nsec) / 1000000.0);
                    if ((ts2.tv_nsec - ts1.tv_nsec) > maxVal) maxVal = (double)(ts2.tv_nsec - ts1.tv_nsec);
                    if ((ts2.tv_nsec - ts1.tv_nsec) < minVal) minVal = (double)(ts2.tv_nsec - ts1.tv_nsec);
                    totalTime += (double)(ts2.tv_nsec - ts1.tv_nsec);
                    
                    // swap the integer payload values
            //        clock_gettime(CLOCK_REALTIME, &ts1);
                    memory_read_response = (FINS_UDP_MEMORY_AREA_READ_RESPONSE *)buffer;
                    for (i = 0; i < NUM_ITEMS; i++)
                    {
                        // after swapping to host format, all of this integer data 
                        // needs to be converted from pulse counts to radians
                        // and then saved in the correct field in the patientData
                        // for writing to the DB, as well as sent to ROS
                        memory_read_response->data[i] = ntohs(memory_read_response->data[i]);
            #if (0)
                        // one or more of these will need to be used to convert the pulse count to required value
                        pulse_count = getPulseCountFromDegrees((double)degrees, PULSE_COUNT_PER_REVOLUTION);
                        degrees = getDegreesFromPulseCount((unsigned int)pulse_count, PULSE_COUNT_PER_REVOLUTION);
                        radians = degreesToRadians(degrees);
                        degrees = radiansToDegrees(radians);
                        pulse_count = getPulseCountFromRadians(radians, PULSE_COUNT_PER_REVOLUTION);
                        radians = getRadiansFromPulseCount((unsigned int)pulse_count, PULSE_COUNT_PER_REVOLUTION);
            #endif
                    }
            //        patientData[numBufferedItems].patientId = patientId;
            //        strcpy(patientData[numBufferedItems].firstName, patientFirstName);
            //        strcpy(patientData[numBufferedItems].middleName, patientMiddleName);
            //        strcpy(patientData[numBufferedItems].lastName, patientLastName);
                    patientData[numBufferedItems].dataSource = DATA_SOURCE_PLC;
                    patientData[numBufferedItems].command = COMMAND_MEMORY_READ_AREA;
                    // save the received, swapped, and converted data for writing to the database
            #if (0)
                    patientData[numBufferedItems].joints[AR_JOINT_INDEX].current_position = randfrom(defaultData.joints[AR_JOINT_INDEX].lower_limit_radians, defaultData.joints[AR_JOINT_INDEX].upper_limit_radians);
                    patientData[numBufferedItems].br_1_joint = randfrom(-1.30, 0.0);
                    patientData[numBufferedItems].dr_1_joint = randfrom(-1.0, 0.0);
                    patientData[numBufferedItems].al_joint = randfrom(-0.70, 0.30);
                    patientData[numBufferedItems].bl_1_joint = randfrom(-1.30, 0.0);
                    patientData[numBufferedItems].dl_1_joint = randfrom(-1.0, 0.0);
                   
                    patientData[numBufferedItems].ar_joint_effort = 0.5;
                    patientData[numBufferedItems].br_1_joint_effort = 1.5;
                    patientData[numBufferedItems].dr_1_joint_effort = 2.5;
                    patientData[numBufferedItems].al_joint_effort = 3.5;
                    patientData[numBufferedItems].bl_1_joint_effort = 4.5;
                    patientData[numBufferedItems].dl_1_joint_effort = 5.5;

                    patientData[numBufferedItems].ar_joint_velocity = 1.0;
                    patientData[numBufferedItems].br_1_joint_velocity = 2.0;
                    patientData[numBufferedItems].dr_1_joint_velocity = 3.0;
                    patientData[numBufferedItems].al_joint_velocity = 4.0;
                    patientData[numBufferedItems].bl_1_joint_velocity = 5.0;
                    patientData[numBufferedItems].dl_1_joint_velocity = 6.0;
            #endif        
                    time_t t = time(NULL);
                    clock_gettime(CLOCK_REALTIME, &ts1);
                    struct tm tm = *localtime(&t);
                    getTimestamp(patientData[numBufferedItems].timeString);
                    // Build ROS data for the publisher
                    for (int i = 0; i < NUM_REGULAR_JOINTS; i++)
                    {
                        // until we can get real data from the PLC, substitute RANDOM DATA using randfrom()
                        switch (i)
                        {
                            case CR1_JOINT_INDEX:
                            case CR2_JOINT_INDEX:
                            case FR1_JOINT_INDEX:
                            case FR2_JOINT_INDEX:
                            case CL1_JOINT_INDEX:
                            case CL2_JOINT_INDEX:
                            case FL1_JOINT_INDEX:
                            case FL2_JOINT_INDEX:
                                patientData[numBufferedItems].joints[i].current_value = randfrom(defaultData.joints[i].lower_limit_meters, defaultData.joints[i].upper_limit_meters);
                                break;
                            default:
                                patientData[numBufferedItems].joints[i].current_value = randfrom(defaultData.joints[i].lower_limit_radians, defaultData.joints[i].upper_limit_radians);
                                break;
                        }
                        patientData[numBufferedItems].joints[i].velocity = randfrom(2.0, 8.0);
                        patientData[numBufferedItems].joints[i].effort = randfrom(10.0, 30.0);
                        msg.position[i]  = patientData[numBufferedItems].joints[i].current_value;
                        msg.velocity[i]  = patientData[numBufferedItems].joints[i].velocity;
                        msg.effort[i]    = patientData[numBufferedItems].joints[i].effort;
                    }
                    msg.header.frame_id = "kinoped";
                    msg.header.stamp = ros::Time::now();

                    memcpy(&currentData, &patientData[numBufferedItems], sizeof(PATIENT_ROW));
                    // publish this instances' data to ROS
                    pthread_cond_signal(&ros_publisher_cv);

                    numBufferedItems++;
                    // write the data to the DB every ROWS_PER_INSERT if debug mode is on
                    if (numBufferedItems && ((numBufferedItems % ROWS_PER_INSERT) == 0) && (KINOPED_DEBUG_MODE_ON == debug_mode))
                    {
                        totalRecords += ROWS_PER_INSERT;
                        printf("numBufferedItems = %d\n", numBufferedItems);
                        debug_data->write_buffered_data(patientData, numBufferedItems);
                        printf("write_patient_data ran after counter tripped totalRecords = %lu\n", totalRecords);
                        memset(patientData, 0, sizeof(patientData));
                        numBufferedItems = 0;
                    }
                    else if (KINOPED_DEBUG_MODE_OFF == debug_mode)
                    {
                        // if we're not in debug mode, don't save the data
                        numBufferedItems = 0;
                    }
                    
                    if (write_data_flag)
                    {
                        // this will be set upon reception of a message from ROS or the Web UI
                        write_data_flag = 0;
                        // for now, just write back the data we read
                        // we'll get "real" data from the subscribed message
                        memory_read_response = (FINS_UDP_MEMORY_AREA_READ_RESPONSE *)buffer;
                        for (i = 0; i < NUM_ITEMS; i++)
                        {
                            memory_write_request.data[i] = htons(memory_read_response->data[i]);
                        }
                        memset(&buffer, 0, sizeof(buffer));
                        memory_write_request.hdr.service_id = service_id++;
                        n = sendto(sockfd, &memory_write_request, sizeof(FINS_UDP_MEMORY_DATA_WRITE_REQUEST), 0, (struct sockaddr *) &serv_addr, sizeof(serv_addr));
                        n = recvfrom(sockfd, buffer, sizeof(buffer), 0, (struct sockaddr *)&serv_addr, (socklen_t *)&slen);
                        memory_write_response = (FINS_UDP_MEMORY_DATA_WRITE_RESPONSE *)buffer;
                        printf("Memory write command code: 0x%04X  response code: 0x%04X\n", ntohs(memory_write_response->command_code), ntohs(memory_write_response->response_code));
                        time_t t = time(NULL);
                        clock_gettime(CLOCK_REALTIME, &ts1);
                        struct tm tm = *localtime(&t);
                        memcpy(&patientData[numBufferedItems], &subscribedData, sizeof(PATIENT_ROW));
                        getTimestamp(patientData[numBufferedItems].timeString);
                        numBufferedItems++;
                        // write the data to the DB every ROWS_PER_INSERT if debug mode is on
                        if (numBufferedItems && ((numBufferedItems % ROWS_PER_INSERT) == 0) && (KINOPED_DEBUG_MODE_ON == debug_mode))
                        {
                            totalRecords += ROWS_PER_INSERT;
                            printf("numBufferedItems = %d\n", numBufferedItems);
                            debug_data->write_buffered_data(patientData, numBufferedItems);
                            printf("write_patient_data ran after counter tripped totalRecords = %lu\n", totalRecords);
                            memset(patientData, 0, sizeof(patientData));
                            numBufferedItems = 0;
                        }
                        else if (KINOPED_DEBUG_MODE_OFF == debug_mode)
                        {
                            // if we're not in debug mode, don't save the data
                            numBufferedItems = 0;
                        }
                    }
                }
				if (KINOPED_RUN_MODE_END == run_mode)
				{
				    keepRunning = 0;
				}
            }
            // close DB
            if (numBufferedItems && (KINOPED_DEBUG_MODE_ON == debug_mode))
            {
                printf("writing final %d records\n", numBufferedItems);
                debug_data->write_buffered_data(patientData, numBufferedItems);
                sleep(1);
            }
            debug_data->close_database_connection();
            // reset flags
            delete debug_data;
            numBufferedItems = 0;
            startReceived = 0;
            gotoLoadReceived = 0;
            debug_mode = KINOPED_DEBUG_MODE_OFF;
        }
        numBufferedItems = 0;
        startReceived = 0;
        gotoLoadReceived = 0;
        debug_mode = KINOPED_DEBUG_MODE_OFF;
    }
#if (0)
 for (int i = 0; i < 20; i++)
 {
 gotoInitialPose();
 sleep(1);
 }
#endif   
    printf("Closing socket\n");
    close(sockfd);
    printf("Total loops: %lu\n", loopCount);
    printf("Average time: %0.03lfms   Min: %0.03lfms  Max: %0.03lfms\n", (totalTime / (double)loopCount) / 1000000.0, minVal / 1000000.0, maxVal / 1000000.0);
    if (shutdownReceived)
    {
        system(LINUX_SHUTDOWN);
    }
    sleep(1);
    exit(0);
}
