/******************************************************************************/
/*                                                                            */
/* FILE:        kinoped.h                                                     */
/*                                                                            */
/* DESCRIPTION:	Structure definitions and defines for the Kinoped             */
/*                                                                            */
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

char gotoLoadReceived = 0;
char gotoHomeReceived = 0;
char debugOn = 0;
char shutdownReceived = 0;
char gotoLoadOneShot = 0;
char startReceived = 0;
char done = 0;

// comment out this line to suppress verbose debug messages during normal runtime exexution
#define DEBUG_PRINTF 1

#define FINS_CPU_MODE_PROGRAM		0			/* The CPU is in program mode				*/
#define FINS_CPU_MODE_MONITOR		2			/* The CPU is in monitor mode				*/
#define FINS_CPU_MODE_RUN			4			/* The CPU is in run mode				*/

#define FRAME_SEND 0x00000002
#define NODE_ADDRESS_DATA_SEND 0x00000000

#define ERROR_CODE_NORMAL 0x0
#define ICF 0x80
#define GATEWAY_COUNT 0x03
#define SYSMAC_NET 0x01
#define UNKNOWN_NODE_NUMBER 0xEF
#define UNIT_ADDRESS_PC 0x00
#define NETWORK_ADDRESS_LOCAL 0x00

#define COMMAND_MEMORY_READ_AREA 0x0101
#define COMMAND_MEMORY_WRITE_AREA 0x0102
#define COMMAND_CONTROLLER_DATA_READ 0x0501
#define COMMAND_CONTROLLER_STATUS_READ 0x0601
#define COMMAND_CONTROLLER_CYCLE_TIME_READ 0x0620
#define COMMAND_RUN     0x0401
#define COMMAND_STOP    0x0402

#define CYCLE_TIME_PARAMETER_INIT 0x00
#define CYCLE_TIME_PARAMETER_READ 0x01

#define MEMORY_AREA_DM_WORD_CONTENTS 0x82
#define BEGINNING_ADDRESS      0x0001
#define BEGINNING_ADDRESS_BITS 0x00
#define NUM_ITEMS 100
#define PACKED __attribute__((__packed__))

#define NUM_RECORDS 5000
#define ROWS_PER_INSERT 50
#define MAX_TIMESTAMP_LENGTH 32
#define MAX_PATIENT_NAME_LENGTH 40
#define MAX_DB_INSERT_STRING_LENGTH 131072
#define MAX_INDIVIDUAL_VALUES_STRING_LENGTH 4096
#define BILLION 1000000000.0
#define TWO_HUNDRED_FIFTY_MS 250000

#define MAX_USER_NAME_LENGTH     128
#define MAX_DATABASE_NAME_LENGTH 128
#define MAX_DATABASE_TABLE_NAME  128

#define KINOPED_PORT 16384
#define OMRON_FINS_PORT 9600
#define OMRON_PLC_IP_ADDRESS "192.168.1.199"

#define PI 3.1415926535897932384626
#define TWO_PI (2 * PI)
#define PULSE_COUNT_PER_REVOLUTION 4096
#define DEGREES_IN_A_CIRCLE 360.0
#define DEGREES_IN_HALF_OF_A_CIRCLE 180.0
#define ONE_PSI_IN_PASCALS          6894.7572931783
#define ONE_BAR_IN_PSI              14.503773773
#define ONE_PSI_IN_BAR               0.068947572975
#define ONE_BAR_IN_PASCALS          100000.0

#define INCHES_IN_A_METER 39.3701
#define MM_IN_AN_INCH 25.4

#define KILOGRAMS_IN_A_POUND   0.45359237

#define DATA_SOURCE_PLC   0
#define DATA_SOURCE_ROS   1
#define DATA_SOURCE_USER  2

#define NUM_REGULAR_JOINTS     26
#define MAX_LENGTH_JOINT_NAME  32

#define AR_JOINT_INDEX    0
#define BR1_JOINT_INDEX   1
#define CR1_JOINT_INDEX   2
#define DR_JOINT_INDEX    3
#define ER1_JOINT_INDEX   4
#define FR1_JOINT_INDEX   5
#define GR_JOINT_INDEX    6
#define HR_JOINT_INDEX    7
#define IR_JOINT_INDEX    8
#define ER2_JOINT_INDEX   9
#define FR2_JOINT_INDEX   10
#define BR2_JOINT_INDEX   11
#define CR2_JOINT_INDEX   12
#define AL_JOINT_INDEX    13
#define BL1_JOINT_INDEX   14
#define CL1_JOINT_INDEX   15
#define DL_JOINT_INDEX    16
#define EL1_JOINT_INDEX   17
#define FL1_JOINT_INDEX   18
#define GL_JOINT_INDEX    19
#define HL_JOINT_INDEX    20
#define IL_JOINT_INDEX    21
#define EL2_JOINT_INDEX   22
#define FL2_JOINT_INDEX   23
#define BL2_JOINT_INDEX   24
#define CL2_JOINT_INDEX   25

#define NUM_POSTGRES_DATABASE_CONNECTIONS 2
#define POSTGRES_USER_TRACKING_INDEX  0
#define POSTGRES_DEBUG_POSITION_INDEX 1
#define POSTGRES_PATIENT_DB_INDEX     2   /*  currently not used  */
#define POSTGRES_DB_TABLE_NAME_USER_TRACKING "user_tracking"
#define POSTGRES_DB_TABLE_NAME_USER_TRACKING_DEBUG "user_tracking_debug"

#define ROS_COMMAND_DEBUG_MODE "debug"
#define ROS_SUB_COMMAND_DEBUG_MODE_ON "on"
#define ROS_SUB_COMMAND_DEBUG_MODE_OFF "off"
#define ROS_COMMAND_SESSION "session"
#define ROS_SUB_COMMAND_START   "start"
#define ROS_SUB_COMMAND_RUN     "run"
#define ROS_SUB_COMMAND_PAUSE   "pause"
#define ROS_SUB_COMMAND_STOP    "stop"
#define ROS_SUB_COMMAND_END     "end"
#define ROS_SUB_COMMAND_RESTART "restart"
#define ROS_SUB_COMMAND_REBOOT  "reboot"
#define ROS_SUB_COMMAND_SHUTDOWN  "shutdown"
#define ROS_COMMAND_GOTO    "goto"
#define ROS_SUB_COMMAND_HOME_POSITION "home"
#define ROS_SUB_COMMAND_LOAD_POSITION "load"
#define ROS_COMMAND_ECHO  "echo"
#define ROS_SUB_COMMAND_ECHO_PLC "plc"
#define ROS_SUB_COMMAND_ECHO_XSENS "xsens"
#define ROS_SUB_COMMAND_ECHO_WEBUI "webui"

#define ROS_NODE_TYPE_PLC    "plc"
#define ROS_NODE_TYPE_XSENS  "xsens"
#define ROS_NODE_TYPE_WEBUI  "webui"

#define LINUX_SHUTDOWN "/sbin/shutdown -r now"

enum RUN_MODES 
{
    KINOPED_RUN_MODE_STOP = 0,
    KINOPED_RUN_MODE_START,
    KINOPED_RUN_MODE_PAUSE,
    KINOPED_RUN_MODE_RUN,
    KINOPED_RUN_MODE_END,
    KINOPED_RUN_MODE_RESTART,
    KINOPED_RUN_MODE_REBOOT,
    KINOPED_RUN_MODE_SHUTDOWN
};

enum DEBUG_MODES
{
    KINOPED_DEBUG_MODE_OFF = 0,
    KINOPED_DEBUG_MODE_ON
};

#define DROP_DEBUG_TABLE "DROP TABLE IF EXISTS user_tracking_debug"

#ifdef INCLUDE_ALL_DATA_IN_DB
#define CREATE_DEBUG_TABLE "CREATE TABLE user_tracking_debug("\
         "user_tracking_debug_id integer NOT NULL GENERATED BY DEFAULT AS IDENTITY ( INCREMENT 1 START 1 MINVALUE 1 MAXVALUE 2147483647 CACHE 1 ),"\
         "FirstName VARCHAR(40), MiddleName VARCHAR(40), LastName VARCHAR(40),"\
         "time_stamp TIMESTAMP WITHOUT TIME ZONE NOT NULL DEFAULT now(), "\
         "dataSource INTEGER, command INTEGER,"\
         "ar_joint FLOAT(8), br1_joint FLOAT(8), cr1_joint FLOAT(8), dr_joint FLOAT(8), er1_joint FLOAT(8), fr1_joint FLOAT(8), gr_joint FLOAT(8), hr_joint FLOAT(8), ir_joint FLOAT(8), er2_joint FLOAT(8), fr2_joint FLOAT(8), br2_joint FLOAT(8), cr2_joint FLOAT(8),"\
         "al_joint FLOAT(8), bl1_joint FLOAT(8), cl1_joint FLOAT(8), dl_joint FLOAT(8), el1_joint FLOAT(8), fl1_joint FLOAT(8), gl_joint FLOAT(8), hl_joint FLOAT(8), il_joint FLOAT(8), el2_joint FLOAT(8), fl2_joint FLOAT(8), bl2_joint FLOAT(8), cl2_joint FLOAT(8),"\
         "ar_joint_velocity FLOAT(8), br1_joint_velocity FLOAT(8), cr1_joint_velocity FLOAT(8), dr_joint_velocity FLOAT(8), er1_joint_velocity FLOAT(8), fr1_joint_velocity FLOAT(8), gr_joint_velocity FLOAT(8), hr_joint_velocity FLOAT(8), ir_joint_velocity FLOAT(8), er2_joint_velocity FLOAT(8), fr2_joint_velocity FLOAT(8), br2_joint_velocity FLOAT(8), cr2_joint_velocity FLOAT(8),"\
         "al_joint_velocity FLOAT(8), bl1_joint_velocity FLOAT(8), cl1_joint_velocity FLOAT(8), dl_joint_velocity FLOAT(8), el1_joint_velocity FLOAT(8), fl1_joint_velocity FLOAT(8), gl_joint_velocity FLOAT(8), hl_joint_velocity FLOAT(8), il_joint_velocity FLOAT(8), el2_joint_velocity FLOAT(8), fl2_joint_velocity FLOAT(8), bl2_joint_velocity FLOAT(8), cl2_joint_velocity FLOAT(8),"\
         "ar_joint_effort FLOAT(8), br1_joint_effort FLOAT(8), cr1_joint_effort FLOAT(8), dr_joint_effort FLOAT(8), er1_joint_effort FLOAT(8), fr1_joint_effort FLOAT(8), gr_joint_effort FLOAT(8), hr_joint_effort FLOAT(8), ir_joint_effort FLOAT(8), er2_joint_effort FLOAT(8), fr2_joint_effort FLOAT(8), br2_joint_effort FLOAT(8), cr2_joint_effort FLOAT(8),"\
         "al_joint_effort FLOAT(8), bl1_joint_effort FLOAT(8), cl1_joint_effort FLOAT(8), dl_joint_effort FLOAT(8), el1_joint_effort FLOAT(8), fl1_joint_effort FLOAT(8), gl_joint_effort FLOAT(8), hl_joint_effort FLOAT(8), il_joint_effort FLOAT(8), el2_joint_effort FLOAT(8), fl2_joint_effort FLOAT(8), bl2_joint_effort FLOAT(8), cl2_joint_effort FLOAT(8),"\
         "CONSTRAINT \"PK_user_tracking_debug\" PRIMARY KEY (user_tracking_debug_id))"
#else
#define CREATE_DEBUG_TABLE "CREATE TABLE user_tracking_debug("\
         "user_tracking_debug_id integer NOT NULL GENERATED BY DEFAULT AS IDENTITY ( INCREMENT 1 START 1 MINVALUE 1 MAXVALUE 2147483647 CACHE 1 ),"\
         "time_stamp TIMESTAMP WITHOUT TIME ZONE NOT NULL DEFAULT now(), "\
         "dataSource INTEGER, command INTEGER,"\
         "ar_joint FLOAT(8), br1_joint FLOAT(8), cr1_joint FLOAT(8), er1_joint FLOAT(8), fr1_joint FLOAT(8), hr_joint FLOAT(8), ir_joint FLOAT(8),"\
         "al_joint FLOAT(8), bl1_joint FLOAT(8), cl1_joint FLOAT(8), el1_joint FLOAT(8), fl1_joint FLOAT(8), hl_joint FLOAT(8), il_joint FLOAT(8),"\
         "ar_joint_velocity FLOAT(8), br1_joint_velocity FLOAT(8), cr1_joint_velocity FLOAT(8), er1_joint_velocity FLOAT(8), fr1_joint_velocity FLOAT(8), hr_joint_velocity FLOAT(8), ir_joint_velocity FLOAT(8),"\
         "al_joint_velocity FLOAT(8), bl1_joint_velocity FLOAT(8), cl1_joint_velocity FLOAT(8), el1_joint_velocity FLOAT(8), fl1_joint_velocity FLOAT(8), hl_joint_velocity FLOAT(8), il_joint_velocity FLOAT(8),"\
         "ar_joint_effort FLOAT(8), br1_joint_effort FLOAT(8), cr1_joint_effort FLOAT(8), er1_joint_effort FLOAT(8), fr1_joint_effort FLOAT(8), hr_joint_effort FLOAT(8), ir_joint_effort FLOAT(8),"\
         "al_joint_effort FLOAT(8), bl1_joint_effort FLOAT(8), cl1_joint_effort FLOAT(8), el1_joint_effort FLOAT(8), fl1_joint_effort FLOAT(8), hl_joint_effort FLOAT(8), il_joint_effort FLOAT(8),"\
         "CONSTRAINT \"PK_user_tracking_debug\" PRIMARY KEY (user_tracking_debug_id))"
#endif

#define DROP_USER_TRACKING_TABLE "DROP TABLE IF EXISTS user_tracking"

#define CREATE_USER_TRACKING_TABLE "CREATE TABLE user_tracking("\
         "user_tracking_id integer NOT NULL GENERATED BY DEFAULT AS IDENTITY ( INCREMENT 1 START 1 MINVALUE 1 MAXVALUE 2147483647 CACHE 1 ),"\
         "time_stamp TIMESTAMP WITHOUT TIME ZONE NOT NULL DEFAULT now(), "\
         "left_hip_x FLOAT(8), left_hip_y FLOAT(8), left_hip_z FLOAT(8), left_knee_x FLOAT(8), left_knee_y FLOAT(8), left_knee_z FLOAT(8), left_foot_ankle_x FLOAT(8), left_foot_ankle_y FLOAT(8), left_foot_ankle_z FLOAT(8),"\
         "right_hip_x FLOAT(8), right_hip_y FLOAT(8), right_hip_z FLOAT(8), right_knee_x FLOAT(8), right_knee_y FLOAT(8), right_knee_z FLOAT(8), right_foot_ankle_x FLOAT(8), right_foot_ankle_y FLOAT(8), right_foot_ankle_z FLOAT(8),"\
         "pulse FLOAT(8), respiration FLOAT(8), oximeter FLOAT(8),"\
         "CONSTRAINT \"PK_user_tracking\" PRIMARY KEY (user_tracking_id))"
         
#define USER_TRACKING_ROW_INSERT_STRING "(%ld,'%s',%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf)%s"

#ifdef INCLUDE_ALL_DATA_IN_DB
#define DEBUG_ROW_INSERT_STRING  "(%ld,'%s','%s','%s','%s',%d,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf)%s"
#else
#define DEBUG_ROW_INSERT_STRING  "(%ld,'%s',%d,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf)%s"
#endif
        
typedef struct
{
   unsigned long secs;
   unsigned long nsecs;
   char name[MAX_LENGTH_JOINT_NAME];
   double trans_x;
   double trans_y;
   double trans_z;
   double rot_x;
   double rot_y;
   double rot_z;
   double rot_angle;
} POSITION;

typedef struct
{
  double leftHipX;
  double leftHipY;
  double leftHipZ;
  double leftKneeX;
  double leftKneeY;
  double leftKneeZ;
  double leftFootX;
  double leftFootY;
  double leftFootZ;
  double rightHipX;
  double rightHipY;
  double rightHipZ;
  double rightKneeX;
  double rightKneeY;
  double rightKneeZ;
  double rightFootX;
  double rightFootY;
  double rightFootZ;
  double pulse;
  double respirationRate;
  double pulseOx;
  char timeString[MAX_TIMESTAMP_LENGTH];
} XSENS_POSITION_DATA;

#define MAX_NUM_XSENS_SENSORS 8
#define SUPPORTED_NUM_XSENS_SENSORS 6
typedef struct
{
    int left_hip_index;
    int left_knee_index;
    int left_foot_index;
    int right_hip_index;
    int right_knee_index;
    int right_foot_index;
    int alternate_1_index;
    int alternate_2_index;
} XSENS_SENSOR_MAPPING;


typedef struct
{
    char name[MAX_LENGTH_JOINT_NAME];
    double current_value;
    double upper_limit_radians;
    double lower_limit_radians;
    double initial_position_radians;
    double upper_limit_meters;
    double lower_limit_meters;
    double initial_position_meters;
    double velocity;
    double effort;
} PACKED REGULAR_JOINT;

typedef struct
{
//    unsigned int patientId;
//    char firstName[MAX_PATIENT_NAME_LENGTH];
//    char middleName[MAX_PATIENT_NAME_LENGTH];
//    char lastName[MAX_PATIENT_NAME_LENGTH];
    uint16_t dataSource;
    uint16_t command;
    REGULAR_JOINT joints[NUM_REGULAR_JOINTS];
    char timeString[MAX_TIMESTAMP_LENGTH];
} PACKED PATIENT_ROW;

typedef struct
{
    char magic[4];   // ASCII  "FINS"
    uint32_t length;
    uint32_t command;
    uint32_t error_code;
    uint32_t node_address;
} PACKED FINS_NODE_ADDRESS_DATA_SEND;

typedef struct
{
    char magic[4];   // ASCII  "FINS"
    uint32_t length;
    uint32_t command;
    uint32_t error_code;
    uint8_t  icf;
    uint8_t  reserved;
    uint8_t  gateway_count;
    uint8_t  destination_network_address;
    uint8_t  destination_node_number;
    uint8_t  destination_unit_address;
    uint8_t  source_network_address;
    uint8_t  source_node_number;
    uint8_t  source_unit_address;
    uint8_t  service_id;
} PACKED FINS_HEADER;

typedef struct
{
    uint8_t  icf;
    uint8_t  reserved;
    uint8_t  gateway_count;
    uint8_t  destination_network_address;
    uint8_t  destination_node_number;
    uint8_t  destination_unit_address;
    uint8_t  source_network_address;
    uint8_t  source_node_number;
    uint8_t  source_unit_address;
    uint8_t  service_id;
} PACKED FINS_UDP_HEADER;

typedef struct
{
    uint16_t command;
    uint8_t  memory_area;
    uint16_t beginning_address;
    uint8_t  beginning_address_bits;
    uint16_t num_items;
} PACKED FINS_COMMAND;

typedef struct
{
    FINS_HEADER hdr;
    FINS_COMMAND cmd;
} PACKED FINS_COMMAND_REQUEST_PACKET;

typedef struct
{
    FINS_HEADER hdr;
    uint16_t command;
    uint16_t response_code;
    uint8_t data[512];
} PACKED FINS_MEMORY_AREA_READ_RESPONSE;

typedef struct
{
   FINS_UDP_HEADER hdr;
   FINS_COMMAND cmd;
} PACKED FINS_UDP_MEMORY_AREA_READ_REQUEST;

typedef struct
{
    FINS_UDP_HEADER hdr;
    uint16_t command;
    uint16_t response_code;
    uint16_t data[NUM_ITEMS];
} PACKED FINS_UDP_MEMORY_AREA_READ_RESPONSE;

typedef struct
{
    FINS_UDP_HEADER hdr;
    uint16_t command;
} PACKED FINS_UDP_CONTROLLER_STATUS_READ_REQUEST;

typedef struct
{
    FINS_UDP_HEADER hdr;
    uint16_t command_code;
    uint16_t response_code;
    uint8_t  status;
    uint8_t  mode;
    union {
        uint16_t int_value;
        uint16_t memory_error      : 1,
                 cpu_bus_error     : 1,
                 reserved2         : 2,
                 io_point_overflow : 1,
                 io_setting_error  : 1,
                 reserved3         : 3,
                 fals_system_error : 1,
                 reserved6         : 6;
    } fatal_error;
    union {
        uint16_t int_value;
        uint16_t fal_system_error      : 1,
                 reserved5             : 5,
                 io_collate_error      : 1,
                 special_io_unit_error : 1,
                 reserved2             : 2,
                 remote_io_error       : 1,
                 battery_error         : 1,
                 reserved2more         : 2,
                 host_link_error       : 1,
                 pc_link_error         : 1;
    } non_fatal_error;
    uint16_t fals_number;
    char     error_message[16];
} PACKED FINS_UDP_CONTROLLER_STATUS_READ_RESPONSE;

typedef struct
{
    FINS_UDP_HEADER hdr;
    uint16_t command;
} PACKED FINS_UDP_CONTROLLER_DATA_READ_REQUEST;

typedef struct
{
    uint8_t program_area_size;   // in K words (1K words = 1024 words, 1 word = 2 bytes)
    uint8_t spare;
    uint16_t num_dm_words;       // words (1 word = 2 bytes)
    char  unused[3];
    uint8_t  kind_of_file_memory; // 0 = no file memory, 1 = SRAM, 4 = first half RAM, second half ROM
    uint16_t file_memory_size;    // 0 = no file memory, 1 = 1000 blocks, 2 = 2000 blocks
} PACKED CONTROLLER_AREA_DATA;

typedef struct
{
    FINS_UDP_HEADER hdr;
    uint16_t command_code;
    uint16_t response_code;
    char     controller_model[20];
    char     controller_version[20];
    char     dummy1[16];
    union {
        char     byte_data[10];
        CONTROLLER_AREA_DATA data;
    } area_data;
    char     dummy2[65];
    uint8_t  pc_status;
} PACKED FINS_UDP_CONTROLLER_DATA_READ_RESPONSE;

typedef struct
{
    FINS_UDP_HEADER hdr;
    uint16_t command;
    uint8_t parameter;
} PACKED FINS_UDP_CONTROLLER_CYCLE_TIME_READ_REQUEST;

typedef struct
{
    FINS_UDP_HEADER hdr;
    uint16_t command_code;
    uint16_t response_code;
    uint32_t average;
    uint32_t max;
    uint32_t min;
} PACKED FINS_UDP_CONTROLLER_CYCLE_TIME_READ_RESPONSE;

typedef struct
{
    FINS_UDP_HEADER hdr;
    uint16_t command_code;
    uint8_t  memory_area_code;
    uint16_t beginning_address;
    uint8_t  always_zero;
    uint16_t num_items;
    uint16_t data[NUM_ITEMS];
} PACKED FINS_UDP_MEMORY_DATA_WRITE_REQUEST;

typedef struct
{
    FINS_UDP_HEADER hdr;
    uint16_t command_code;
    uint16_t response_code;
} PACKED FINS_UDP_MEMORY_DATA_WRITE_RESPONSE;

typedef struct
{
    FINS_UDP_HEADER hdr;
    uint16_t command_code;
    uint16_t program_number;   // always zero
    uint8_t  mode;             // FINS_CPU_MODE_MONITOR or FINS_CPU_MODE_RUN
} PACKED FINS_UDP_RUN_REQUEST;

typedef struct
{
    FINS_UDP_HEADER hdr;
    uint16_t command_code;
    uint16_t response_code;
} PACKED FINS_UDP_RUN_RESPONSE;

typedef struct
{
    FINS_UDP_HEADER hdr;
    uint16_t command_code;
} PACKED FINS_UDP_STOP_REQUEST;

typedef struct
{
    FINS_UDP_HEADER hdr;
    uint16_t command_code;
    uint16_t response_code;
} PACKED FINS_UDP_STOP_RESPONSE;
