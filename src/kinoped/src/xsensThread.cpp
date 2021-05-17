/******************************************************************************/
/*                                                                            */
/* FILE:        xsensThread.cpp                                               */
/*                                                                            */
/* DESCRIPTION:	Code to read/write data from the Xsens sensors, and write it  */
 /*             the postgres database for operation of the Kinoped            */
/*                                                                            */
/* AUTHOR(S):   USA Firmware, LLC                                             */
/*                                                                            */
/* DATE:        November 23, 2020                                             */
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
#include <xsensdeviceapi.h>

#include <string>
#include <stdexcept>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <set>
#include <list>
#include <utility>

#include <xsmutex.h>
#include <xstypes/xstime.h>
#include "kinoped.h"
#include "kinoped_msgs/xsens_sensors.h"
#include "kinoped_msgs/user_command.h"
#include "kinoped_msgs/kinoped_message.h"

char thisNodeType[] = { ROS_NODE_TYPE_XSENS };
double minVal = 10000000.0;
double maxVal = 0.0;
unsigned long loopCount = 0;
double totalTime = 0.0;
struct timeval tv;
char xsensThread_keep_running = 1;
char xsensThreadIsRunning = 0;
char keepRunning = 1;
#if 0
char gotoLoadReceived = 0;
char gotoHomeReceived = 0;
char debugOn = 0;
char shutdownReceived = 0;
char gotoLoadOneShot = 0;
char startReceived = 0;
char done = 0;
#endif

char mappingReceived = 0;

char write_data_flag = 0;
struct timespec ts1, ts2;
char buffer[2048];
char messageDesc[128];


ros::Publisher statusPub;
ros::Publisher macPub;
kinoped_msgs::kinoped_message statusMsg;
kinoped_msgs::xsens_sensors macAddresses;

pthread_t kinoped_xsens_thread_id;
unsigned int numBufferedXsensItems = 0;
XSENS_POSITION_DATA positionData[ROWS_PER_INSERT];
int xsens_sensor_mapping[SUPPORTED_NUM_XSENS_SENSORS];
const char *sorted_array[SUPPORTED_NUM_XSENS_SENSORS];

char userName[MAX_USER_NAME_LENGTH];
char dbName[MAX_DATABASE_NAME_LENGTH];

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
    statusMsg.node = kinoped_msgs::kinoped_message::KINOPED_NODE_TYPE_XSENS;
    statusMsg.code = statusCode;
    statusMsg.type = type;
    statusMsg.description = description;
    statusMsg.header.stamp = ros::Time::now();
    statusPub.publish(statusMsg);
}


#include "utils.cpp"
#include "kinopedDBXsens.cpp"
#include "callbacks.cpp"

/*! \brief Given a list of update rates and a desired update rate, returns the closest update rate to the desired one */
int findClosestUpdateRate(const XsIntArray& supportedUpdateRates, const int desiredUpdateRate)
{
	if (supportedUpdateRates.empty())
	{
		return 0;
	}

	if (supportedUpdateRates.size() == 1)
	{
		return supportedUpdateRates[0];
	}

	int uRateDist = -1;
	int closestUpdateRate = -1;
	for (XsIntArray::const_iterator itUpRate = supportedUpdateRates.begin(); itUpRate != supportedUpdateRates.end(); ++itUpRate)
	{
		const int currDist = std::abs(*itUpRate - desiredUpdateRate);

		if ((uRateDist == -1) || (currDist < uRateDist))
		{
			uRateDist = currDist;
			closestUpdateRate = *itUpRate;
		}
	}
	return closestUpdateRate;
}

//----------------------------------------------------------------------
// Callback handler for wireless master
//----------------------------------------------------------------------
class WirelessMasterCallback : public XsCallback
{
public:
	typedef std::set<XsDevice*> XsDeviceSet;

	XsDeviceSet getWirelessMTWs() const
	{
		XsMutexLocker lock(m_mutex);
		return m_connectedMTWs;
	}

protected:
	virtual void onConnectivityChanged(XsDevice* dev, XsConnectivityState newState)
	{
		XsMutexLocker lock(m_mutex);
		switch (newState)
		{
		case XCS_Disconnected:		/*!< Device has disconnected, only limited informational functionality is available. */
			std::cout << "\nEVENT: MTW Disconnected" << std::endl;
			m_connectedMTWs.erase(dev);
			break;
		case XCS_Rejected:			/*!< Device has been rejected and is disconnected, only limited informational functionality is available. */
			std::cout << "\nEVENT: MTW Rejected"<< std::endl;
			m_connectedMTWs.erase(dev);
			break;
		case XCS_PluggedIn:			/*!< Device is connected through a cable. */
			std::cout << "\nEVENT: MTW PluggedIn" << std::endl;
			m_connectedMTWs.erase(dev);
			break;
		case XCS_Wireless:			/*!< Device is connected wirelessly. */
			std::cout << "\nEVENT: MTW Connected" << std::endl;
			m_connectedMTWs.insert(dev);
			break;
		case XCS_File:				/*!< Device is reading from a file. */
			std::cout << "\nEVENT: MTW File" << std::endl;
			m_connectedMTWs.erase(dev);
			break;
		case XCS_Unknown:			/*!< Device is in an unknown state. */
			std::cout << "\nEVENT: MTW Unknown" << std::endl;
			m_connectedMTWs.erase(dev);
			break;
		default:
			std::cout << "\nEVENT: MTW Error" << std::endl;
			m_connectedMTWs.erase(dev);
			break;
		}
	}
private:
	mutable XsMutex m_mutex;
	XsDeviceSet m_connectedMTWs;
};

//----------------------------------------------------------------------
// Callback handler for MTw
// Handles onDataAvailable callbacks for MTW devices
//----------------------------------------------------------------------
class MtwCallback : public XsCallback
{
public:
	MtwCallback(int mtwIndex, XsDevice* device)
		:m_mtwIndex(mtwIndex)
		,m_device(device)
	{}

	bool dataAvailable() const
	{
		XsMutexLocker lock(m_mutex);
		return !m_packetBuffer.empty();
	}

	XsDataPacket const * getOldestPacket() const
	{
		XsMutexLocker lock(m_mutex);
		XsDataPacket const * packet = &m_packetBuffer.front();
		return packet;
	}

	void deleteOldestPacket()
	{
		XsMutexLocker lock(m_mutex);
		m_packetBuffer.pop_front();
	}

	int getMtwIndex() const
	{
		return m_mtwIndex;
	}

	XsDevice const & device() const
	{
		assert(m_device != 0);
		return *m_device;
	}

protected:
	virtual void onLiveDataAvailable(XsDevice*, const XsDataPacket* packet)
	{
		XsMutexLocker lock(m_mutex);
		// NOTE: Processing of packets should not be done in this thread.

		m_packetBuffer.push_back(*packet);
		if (m_packetBuffer.size() > 300)
		{
			std::cout << std::endl;
			deleteOldestPacket();
		}
	}

private:
	mutable XsMutex m_mutex;
	std::list<XsDataPacket> m_packetBuffer;
	int m_mtwIndex;
	XsDevice* m_device;
};

static int myCompare(const void* a, const void* b) 
{ 
    // setting up rules for comparison 
    return strcmp(*(const char**)a, *(const char**)b); 
} 
  
// Function to sort the array 
void sort(const char* arr[], int n) 
{ 
    // calling qsort function to sort the array 
    // with the help of Comparator 
    qsort(arr, n, sizeof(const char*), myCompare); 
} 

/*******************************************************************************************/
/*                                                                                         */
/* void xsensSensorMappingCallback(const kinoped_msgs::xsens_sensors::ConstPtr& msg)            */
/*                                                                                         */
/* Callback for handling ROS user_measurements messages.                                   */
/*                                                                                         */
/*******************************************************************************************/
void xsensSensorMappingCallback(const kinoped_msgs::xsens_sensors::ConstPtr& msg)
{
    ROS_INFO("I heard: Xsens Sensor First MAC[%s]", msg->mac_addresses[0].c_str());
    if (mappingReceived)
    {
        printf("Xsens Mapping already received - Ignoring\n");
        return;
    }
    
    int n = SUPPORTED_NUM_XSENS_SENSORS;
    int i;
  
    for (i = 0; i < n; i ++)
    {
        sorted_array[i] = msg->mac_addresses[i].c_str();
    }

    // sort the list of MAC addresses, since that is the way the Xsens always returns the data
    // a sorted array by MAC addresses, lowest to highest, with no knowledge of where they are
    // placed on the person
    // Thus, we have to map the MAC to the physical location on the body
    sort(sorted_array, n); 
  
    for (i = 0; i < n; i++)
    {
        char found = 0;
        for (int j = 0; (j < n) && !found; j++)
        {
            if (!strcmp(msg->mac_addresses[i].c_str(), sorted_array[j]))
            {
                found = 1;
                xsens_sensor_mapping[j] = msg->positions[i];
            }
        }
    }
    // Print the sorted names 
    printf("\nSorted and mapped array is\n"); 
    for (i = 0; i < n; i++) 
    {
        printf("%d: %s mapped Index: %d Description: ", i, sorted_array[i], xsens_sensor_mapping[i]); 
        switch (xsens_sensor_mapping[i])
		{
		   case kinoped_msgs::xsens_sensors::POSITION_LEFT_FOOT:
		       printf("POSITION_LEFT_FOOT\n");
			   break;
		   case kinoped_msgs::xsens_sensors::POSITION_RIGHT_FOOT:
		       printf("POSITION_RIGHT_FOOT\n");
			   break;
		   case kinoped_msgs::xsens_sensors::POSITION_LEFT_KNEE:
		       printf("POSITION_LEFT_KNEE\n");
			   break;
		   case kinoped_msgs::xsens_sensors::POSITION_RIGHT_KNEE:
		       printf("POSITION_RIGHT_KNEE\n");
			   break;
		   case kinoped_msgs::xsens_sensors::POSITION_LEFT_HIP:
		       printf("POSITION_LEFT_HIP\n");
			   break;
		   case kinoped_msgs::xsens_sensors::POSITION_RIGHT_HIP:
		       printf("POSITION_RIGHT_HIP\n");
			   break;
		   case kinoped_msgs::xsens_sensors::POSITION_ALTERNATE_1:
		       printf("POSITION_ALTERNATE_1\n");
			   break;
		   case kinoped_msgs::xsens_sensors::POSITION_ALTERNATE_2:
		       printf("POSITION_ALTERNATE_2\n");
			   break;
		}
	}
    mappingReceived = 1;
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
    strcpy(userName, "kinoped");
    strcpy(dbName, "postgres");
    mappingReceived = 0;
    keepRunning = 1;
}


/************************************************************************************************/
/*                                                                                              */
/* void *kinoped_xsens_thread(void *arg)                                                        */
/*                                                                                              */
/* The main Xsens sensor processing loop.                                                       */
/*                                                                                              */
/************************************************************************************************/
void *kinoped_xsens_thread(void *arg)
{
    xsensThreadIsRunning = 1;
    int waitLoopCount = 0;
	const int desiredUpdateRate = 75;	// Use 75 Hz update rate for MTWs
	const int desiredRadioChannel = 19;	// Use radio channel 19 for wireless master.

	WirelessMasterCallback wirelessMasterCallback; // Callback for wireless master
	std::vector<MtwCallback*> mtwCallbacks; // Callbacks for mtw devices

	std::cout << "Constructing XsControl..." << std::endl;
	XsControl* control = XsControl::construct();
	if (control == 0)
	{
		std::cout << "Failed to construct XsControl instance." << std::endl;
        sprintf(messageDesc, "Failed to construct XsControl instance.");
        printf("%s\n", messageDesc);
        publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_GENERAL_ERROR, kinoped_msgs::kinoped_message::KINOPED_MSG_FATAL, messageDesc);
	}

	try
	{
		std::cout << "Scanning ports..." << std::endl;
		XsPortInfoArray detectedDevices = XsScanner::scanPorts();

		std::cout << "Finding wireless master..." << std::endl;
		XsPortInfoArray::const_iterator wirelessMasterPort = detectedDevices.begin();
		while (wirelessMasterPort != detectedDevices.end() && !wirelessMasterPort->deviceId().isWirelessMaster())
		{
			++wirelessMasterPort;
		}
		if (wirelessMasterPort == detectedDevices.end())
		{
            sprintf(messageDesc, "No wireless masters found");
            printf("%s\n", messageDesc);
            publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_GENERAL_ERROR, kinoped_msgs::kinoped_message::KINOPED_MSG_FATAL, messageDesc);
			throw std::runtime_error("No wireless masters found");
		}
		std::cout << "Wireless master found" << std::endl;

		std::cout << "Opening port..." << std::endl;
		if (!control->openPort(wirelessMasterPort->portName().toStdString(), wirelessMasterPort->baudrate()))
		{
            sprintf(messageDesc, "Failed to open port");
            printf("%s\n", messageDesc);
            publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_GENERAL_ERROR, kinoped_msgs::kinoped_message::KINOPED_MSG_FATAL, messageDesc);
			std::ostringstream error;
			error << "Failed to open port";
			throw std::runtime_error(error.str());
		}

		std::cout << "Getting XsDevice instance for wireless master..." << std::endl;
		XsDevicePtr wirelessMasterDevice = control->device(wirelessMasterPort->deviceId());
		if (wirelessMasterDevice == 0)
		{
            sprintf(messageDesc, "Failed to construct XsDevice instance");
            printf("%s\n", messageDesc);
            publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_GENERAL_ERROR, kinoped_msgs::kinoped_message::KINOPED_MSG_FATAL, messageDesc);
			std::ostringstream error;
			error << "Failed to construct XsDevice instance";
			throw std::runtime_error(error.str());
		}

		std::cout << "XsDevice instance created" << std::endl;

		std::cout << "Setting config mode..." << std::endl;
		if (!wirelessMasterDevice->gotoConfig())
		{
            sprintf(messageDesc, "Failed to goto config mode");
            printf("%s\n", messageDesc);
            publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_GENERAL_ERROR, kinoped_msgs::kinoped_message::KINOPED_MSG_FATAL, messageDesc);
			std::ostringstream error;
			error << "Failed to goto config mode: ";
			throw std::runtime_error(error.str());
		}

		std::cout << "Attaching callback handler..." << std::endl;
		wirelessMasterDevice->addCallbackHandler(&wirelessMasterCallback);

		std::cout << "Getting the list of the supported update rates..." << std::endl;
		const XsIntArray supportedUpdateRates = wirelessMasterDevice->supportedUpdateRates();

		std::cout << "Supported update rates: ";
		for (XsIntArray::const_iterator itUpRate = supportedUpdateRates.begin(); itUpRate != supportedUpdateRates.end(); ++itUpRate)
		{
			std::cout << *itUpRate << " ";
		}
		std::cout << std::endl;

		const int newUpdateRate = findClosestUpdateRate(supportedUpdateRates, desiredUpdateRate);

		std::cout << "Setting update rate to " << newUpdateRate << " Hz..." << std::endl;
		if (!wirelessMasterDevice->setUpdateRate(newUpdateRate))
		{
            sprintf(messageDesc, "Failed to set update rate");
            printf("%s\n", messageDesc);
            publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_GENERAL_ERROR, kinoped_msgs::kinoped_message::KINOPED_MSG_FATAL, messageDesc);
			std::ostringstream error;
			error << "Failed to set update rate";
			throw std::runtime_error(error.str());
		}

		std::cout << "Disabling radio channel if previously enabled..." << std::endl;
		if (wirelessMasterDevice->isRadioEnabled())
		{
			if (!wirelessMasterDevice->disableRadio())
			{
                sprintf(messageDesc, "Failed to disable radio channel");
                printf("%s\n", messageDesc);
                publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_GENERAL_ERROR, kinoped_msgs::kinoped_message::KINOPED_MSG_FATAL, messageDesc);
				std::ostringstream error;
				error << "Failed to disable radio channel";
				throw std::runtime_error(error.str());
			}
		}

		std::cout << "Setting radio channel to " << desiredRadioChannel << " and enabling radio..." << std::endl;
		if (!wirelessMasterDevice->enableRadio(desiredRadioChannel))
		{
            sprintf(messageDesc, "Failed to set radio channel");
            printf("%s\n", messageDesc);
            publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_GENERAL_ERROR, kinoped_msgs::kinoped_message::KINOPED_MSG_FATAL, messageDesc);
			std::ostringstream error;
			error << "Failed to set radio channel";
			throw std::runtime_error(error.str());
		}

		std::cout << "Waiting for MTW to wirelessly connect...\n" << std::endl;

		bool waitForConnections = true;
		size_t connectedMTWCount = wirelessMasterCallback.getWirelessMTWs().size();
		do
		{
			XsTime::msleep(100);

			while (true)
			{
				size_t nextCount = wirelessMasterCallback.getWirelessMTWs().size();
				if (nextCount != connectedMTWCount)
				{
					std::cout << "Number of connected MTWs: " << nextCount << ". Press 'Y' to start measurement." << std::endl;
					connectedMTWCount = nextCount;
				}
				else
				{
					break;
				}
			}
		}
		while (connectedMTWCount < 6);
        
		XsDeviceIdArray allDevIds = control->deviceIds();
		XsDeviceIdArray mtwDevIds;
        // publish the list of MAC addresses to the UI so that the operator doesn't have to type them all in
		for (XsDeviceIdArray::const_iterator i = allDevIds.begin(); i != allDevIds.end(); ++i)
		{
			if (i->isMtw())
			{
                char id[16];
                memset(id, 0, sizeof(id));
                sprintf(id,"%08X", i->toInt());
                macAddresses.mac_addresses.push_back(id);
			}
		}
        macAddresses.header.stamp = ros::Time::now();
        macPub.publish(macAddresses);

        while (!mappingReceived)
        {
           printf("kinoped_xsens_thread - waiting for Xsens Sensor Mappping: wait loop count: %d\n", waitLoopCount++);
           sleep(1);
        }

		std::cout << "Starting measurement..." << std::endl;
		if (!wirelessMasterDevice->gotoMeasurement())
		{
            sprintf(messageDesc, "Failed to goto measurement mode");
            printf("%s\n", messageDesc);
            publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_GENERAL_ERROR, kinoped_msgs::kinoped_message::KINOPED_MSG_FATAL, messageDesc);
			std::ostringstream error;
			error << "Failed to goto measurement mode";
			throw std::runtime_error(error.str());
		}

		std::cout << "Getting XsDevice instances for all MTWs..." << std::endl;
		XsDeviceIdArray allDeviceIds = control->deviceIds();
		XsDeviceIdArray mtwDeviceIds;
		for (XsDeviceIdArray::const_iterator i = allDeviceIds.begin(); i != allDeviceIds.end(); ++i)
		{
			if (i->isMtw())
			{
				mtwDeviceIds.push_back(*i);
			}
		}
		XsDevicePtrArray mtwDevices;
		for (XsDeviceIdArray::const_iterator i = mtwDeviceIds.begin(); i != mtwDeviceIds.end(); ++i)
		{
			XsDevicePtr mtwDevice = control->device(*i);
			if (mtwDevice != 0)
			{
				mtwDevices.push_back(mtwDevice);
			}
			else
			{
                sprintf(messageDesc, "Failed to create an MTW XsDevice instance");
                printf("%s\n", messageDesc);
                publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_GENERAL_ERROR, kinoped_msgs::kinoped_message::KINOPED_MSG_FATAL, messageDesc);
				throw std::runtime_error("Failed to create an MTW XsDevice instance");
			}
		}

		std::cout << "Attaching callback handlers to MTWs..." << std::endl;
		mtwCallbacks.resize(mtwDevices.size());
		for (int i = 0; i < (int)mtwDevices.size(); ++i)
		{
			mtwCallbacks[i] = new MtwCallback(i, mtwDevices[i]);
			mtwDevices[i]->addCallbackHandler(mtwCallbacks[i]);
		}

		std::cout << "\nMain loop. Press any key to quit\n" << std::endl;
		std::cout << "Waiting for data available..." << std::endl;

//		std::vector<XsEuler> eulerData(mtwCallbacks.size()); // Room to store euler data for each mtw
		std::vector<XsQuaternion> quaternionData(mtwCallbacks.size()); // Room to store quaternion data for each mtw
        sprintf(messageDesc, "Receiving data from Xsens sensors");
        printf("%s\n", messageDesc);
        publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_OK, kinoped_msgs::kinoped_message::KINOPED_MSG_INFO, messageDesc);
		while (xsensThread_keep_running)
        {
			XsTime::msleep(0);

			bool newDataAvailable = false;
			for (size_t i = 0; i < mtwCallbacks.size(); ++i)
			{
				if (mtwCallbacks[i]->dataAvailable())
				{
					newDataAvailable = true;
					XsDataPacket const * packet = mtwCallbacks[i]->getOldestPacket();
//					eulerData[i] = packet->orientationEuler();
                    quaternionData[i] = packet->orientationQuaternion();
					mtwCallbacks[i]->deleteOldestPacket();
				}
			}

			if (newDataAvailable && ((KINOPED_RUN_MODE_RUN == run_mode) || (KINOPED_RUN_MODE_START == run_mode)))
			{
                for (size_t i = 0; i < mtwCallbacks.size(); ++i)
                {
#if defined(DEBUG_PRINTF)
//                  std::cout << "[" << i << "]: ID: " << mtwCallbacks[i]->device().deviceId().toString().toStdString()
//								  << ", Roll: " << std::setw(7) << std::fixed << std::setprecision(2) << eulerData[i].roll()
//								  << ", Pitch: " << std::setw(7) << std::fixed << std::setprecision(2) << eulerData[i].pitch()
//								  << ", Yaw: " <<  std::setw(7) << std::fixed << std::setprecision(2) << eulerData[i].yaw()
//								  << "\n";
//                  XsQuaternion quaternion = packet.orientationQuaternion();
                    std::cout << "[" << i << "]: ID: " << mtwCallbacks[i]->device().deviceId().toString().toStdString()
                              << " W:" << quaternionData[i].w()
                              << ", X:" << quaternionData[i].x()
                              << ", Y:" << quaternionData[i].y()
                              << ", Z:" << quaternionData[i].z()
                              << "\n";
#endif
                    // since the Xsens library always returns the data sorted by MAC address (lowest = 0, highest = 5)
                    // we need to map the data to the actual position of the sensor on the body
                    // as told to us in the ROS xsens_sensors message (filled in by the operator on the Web UI)
                    switch (xsens_sensor_mapping[i])
                    {
                        case kinoped_msgs::xsens_sensors::POSITION_LEFT_HIP:
                            positionData[numBufferedXsensItems].leftHipX = (double)quaternionData[i].x();
                            positionData[numBufferedXsensItems].leftHipY = (double)quaternionData[i].y();
                            positionData[numBufferedXsensItems].leftHipZ = (double)quaternionData[i].z();
                            break;
                        case kinoped_msgs::xsens_sensors::POSITION_LEFT_KNEE:
                            positionData[numBufferedXsensItems].leftKneeX = (double)quaternionData[i].x();
                            positionData[numBufferedXsensItems].leftKneeY = (double)quaternionData[i].y();
                            positionData[numBufferedXsensItems].leftKneeZ = (double)quaternionData[i].z();
                            break;
                        case kinoped_msgs::xsens_sensors::POSITION_LEFT_FOOT:
                            positionData[numBufferedXsensItems].leftFootX = (double)quaternionData[i].x();
                            positionData[numBufferedXsensItems].leftFootY = (double)quaternionData[i].y();
                            positionData[numBufferedXsensItems].leftFootZ = (double)quaternionData[i].z();
                            break;
                        case kinoped_msgs::xsens_sensors::POSITION_RIGHT_HIP:
                            positionData[numBufferedXsensItems].rightHipX = (double)quaternionData[i].x();
                            positionData[numBufferedXsensItems].rightHipY = (double)quaternionData[i].y();
                            positionData[numBufferedXsensItems].rightHipZ = (double)quaternionData[i].z();
                            break;
                        case kinoped_msgs::xsens_sensors::POSITION_RIGHT_KNEE:
                            positionData[numBufferedXsensItems].rightKneeX = (double)quaternionData[i].x();
                            positionData[numBufferedXsensItems].rightKneeY = (double)quaternionData[i].y();
                            positionData[numBufferedXsensItems].rightKneeZ = (double)quaternionData[i].z();
                            break;
                        case kinoped_msgs::xsens_sensors::POSITION_RIGHT_FOOT:
                            positionData[numBufferedXsensItems].rightFootX = (double)quaternionData[i].x();
                            positionData[numBufferedXsensItems].rightFootY = (double)quaternionData[i].y();
                            positionData[numBufferedXsensItems].rightFootZ = (double)quaternionData[i].z();
                            break;
                        case kinoped_msgs::xsens_sensors::POSITION_ALTERNATE_1:
                        case kinoped_msgs::xsens_sensors::POSITION_ALTERNATE_2:
                        default:
                            // no DB entries for these items (no corresponding entries in the create or insert statements)
                            // ignore them for now
                            break;
                    }
                }
                // placeholders for these since we don't have any way of measuring them
                positionData[numBufferedXsensItems].pulse = 0.0;
                positionData[numBufferedXsensItems].respirationRate = 0.0;
                positionData[numBufferedXsensItems].pulseOx = 0.0;
                getTimestamp(positionData[numBufferedXsensItems].timeString);
                numBufferedXsensItems++;
                if (ROWS_PER_INSERT == numBufferedXsensItems)
                {
printf("Write Xsens data to DB\n");
                    xsens_data->write_buffered_data(&positionData[0], numBufferedXsensItems);
                    numBufferedXsensItems = 0;
                }
			}
            if (KINOPED_RUN_MODE_END == run_mode)
            {
               xsensThread_keep_running = 0;
            }
		}

        // when we exit the thread, we need to shut down gracefully
        // otherwise, the next time we attempt to talk to the Xsens USB dongle,
        // it will fail
		std::cout << "Setting config mode..." << std::endl;
		if (!wirelessMasterDevice->gotoConfig())
		{
            sprintf(messageDesc, "Failed to goto config mode");
            printf("%s\n", messageDesc);
            publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_GENERAL_ERROR, kinoped_msgs::kinoped_message::KINOPED_MSG_FATAL, messageDesc);
			std::cout << "Failed to goto config mode" << std::endl;
		}

		std::cout << "Disabling radio... " << std::endl;
		if (!wirelessMasterDevice->disableRadio())
		{
            sprintf(messageDesc, "Failed to disable radio");
            printf("%s\n", messageDesc);
            publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_GENERAL_ERROR, kinoped_msgs::kinoped_message::KINOPED_MSG_FATAL, messageDesc);
			std::cout << "Failed to disable radio" << std::endl;
		}
	}
	catch (std::exception const & ex)
	{
		std::cout << ex.what() << std::endl;
		std::cout << "****ABORT****" << std::endl;
        sprintf(messageDesc, "****ABORT****");
        printf("%s\n", messageDesc);
        publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_GENERAL_ERROR, kinoped_msgs::kinoped_message::KINOPED_MSG_FATAL, messageDesc);
	}
	catch (...)
	{
		std::cout << "An unknown fatal error has occured. Aborting." << std::endl;
		std::cout << "****ABORT****" << std::endl;
        sprintf(messageDesc, "An unknown fatal error has occured. Aborting.");
        printf("%s\n", messageDesc);
        publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_GENERAL_ERROR, kinoped_msgs::kinoped_message::KINOPED_MSG_FATAL, messageDesc);
	}

	std::cout << "Closing XsControl..." << std::endl;
	control->close();

	std::cout << "Deleting mtw callbacks..." << std::endl;
	for (std::vector<MtwCallback*>::iterator i = mtwCallbacks.begin(); i != mtwCallbacks.end(); ++i)
	{
		delete (*i);
	}

	std::cout << "Successful exit of xsens thread." << std::endl;
    sprintf(messageDesc, "Successful exit of xsens thread.");
    printf("%s\n", messageDesc);
    publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_OK, kinoped_msgs::kinoped_message::KINOPED_MSG_INFO, messageDesc);
    kinoped_xsens_thread_id = 0;
    return NULL;
}

/************************************************************************************************/
/*                                                                                              */
/* void publishXsensSensorMACS()                                                                */
/*                                                                                              */
/* Read the Xsens sensor MAC addresses and publish them to ROS so that the UI operator doesn't  */
/* have to type them in manually.                                                               */
/*                                                                                              */
/************************************************************************************************/
void publishXsensSensorMACS()
{
    const int desiredUpdateRate = 75;	// Use 75 Hz update rate for MTWs
    const int desiredRadioChannel = 19;	// Use radio channel 19 for wireless master.

	WirelessMasterCallback wirelessMasterCallback; // Callback for wireless master
	std::vector<MtwCallback*> mtwCallbacks; // Callbacks for mtw devices

	std::cout << "Constructing XsControl..." << std::endl;
	XsControl* control = XsControl::construct();
	if (control == 0)
	{
		std::cout << "Failed to construct XsControl instance." << std::endl;
        sprintf(messageDesc, "Failed to construct XsControl instance.");
        printf("%s\n", messageDesc);
        publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_GENERAL_ERROR, kinoped_msgs::kinoped_message::KINOPED_MSG_FATAL, messageDesc);
	}

	try
	{
		std::cout << "Scanning ports..." << std::endl;
		XsPortInfoArray detectedDevices = XsScanner::scanPorts();

		std::cout << "Finding wireless master..." << std::endl;
		XsPortInfoArray::const_iterator wirelessMasterPort = detectedDevices.begin();
		while (wirelessMasterPort != detectedDevices.end() && !wirelessMasterPort->deviceId().isWirelessMaster())
		{
			++wirelessMasterPort;
		}
		if (wirelessMasterPort == detectedDevices.end())
		{
            sprintf(messageDesc, "No wireless masters found");
            printf("%s\n", messageDesc);
            publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_GENERAL_ERROR, kinoped_msgs::kinoped_message::KINOPED_MSG_FATAL, messageDesc);
			throw std::runtime_error("No wireless masters found");
		}
		std::cout << "Wireless master found" << std::endl;

		std::cout << "Opening port..." << std::endl;
		if (!control->openPort(wirelessMasterPort->portName().toStdString(), wirelessMasterPort->baudrate()))
		{
            sprintf(messageDesc, "Failed to open port");
            printf("%s\n", messageDesc);
            publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_GENERAL_ERROR, kinoped_msgs::kinoped_message::KINOPED_MSG_FATAL, messageDesc);
			std::ostringstream error;
			error << "Failed to open port";
			throw std::runtime_error(error.str());
		}

		std::cout << "Getting XsDevice instance for wireless master..." << std::endl;
		XsDevicePtr wirelessMasterDevice = control->device(wirelessMasterPort->deviceId());
		if (wirelessMasterDevice == 0)
		{
            sprintf(messageDesc, "Failed to construct XsDevice instance");
            printf("%s\n", messageDesc);
            publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_GENERAL_ERROR, kinoped_msgs::kinoped_message::KINOPED_MSG_FATAL, messageDesc);
			std::ostringstream error;
			error << "Failed to construct XsDevice instance";
			throw std::runtime_error(error.str());
		}

		std::cout << "XsDevice instance created" << std::endl;

		std::cout << "Setting config mode..." << std::endl;
		if (!wirelessMasterDevice->gotoConfig())
		{
            sprintf(messageDesc, "Failed to goto config mode");
            printf("%s\n", messageDesc);
            publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_GENERAL_ERROR, kinoped_msgs::kinoped_message::KINOPED_MSG_FATAL, messageDesc);
			std::ostringstream error;
			error << "Failed to goto config mode: ";
			throw std::runtime_error(error.str());
		}

		std::cout << "Attaching callback handler..." << std::endl;
		wirelessMasterDevice->addCallbackHandler(&wirelessMasterCallback);

		std::cout << "Getting the list of the supported update rates..." << std::endl;
		const XsIntArray supportedUpdateRates = wirelessMasterDevice->supportedUpdateRates();

		std::cout << "Supported update rates: ";
		for (XsIntArray::const_iterator itUpRate = supportedUpdateRates.begin(); itUpRate != supportedUpdateRates.end(); ++itUpRate)
		{
			std::cout << *itUpRate << " ";
		}
		std::cout << std::endl;

		const int newUpdateRate = findClosestUpdateRate(supportedUpdateRates, desiredUpdateRate);

		std::cout << "Setting update rate to " << newUpdateRate << " Hz..." << std::endl;
		if (!wirelessMasterDevice->setUpdateRate(newUpdateRate))
		{
            sprintf(messageDesc, "Failed to set update rate");
            printf("%s\n", messageDesc);
            publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_GENERAL_ERROR, kinoped_msgs::kinoped_message::KINOPED_MSG_FATAL, messageDesc);
			std::ostringstream error;
			error << "Failed to set update rate";
			throw std::runtime_error(error.str());
		}

		std::cout << "Disabling radio channel if previously enabled..." << std::endl;
		if (wirelessMasterDevice->isRadioEnabled())
		{
			if (!wirelessMasterDevice->disableRadio())
			{
                sprintf(messageDesc, "Failed to disable radio channel");
                printf("%s\n", messageDesc);
                publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_GENERAL_ERROR, kinoped_msgs::kinoped_message::KINOPED_MSG_FATAL, messageDesc);
				std::ostringstream error;
				error << "Failed to disable radio channel";
				throw std::runtime_error(error.str());
			}
		}

		std::cout << "Setting radio channel to " << desiredRadioChannel << " and enabling radio..." << std::endl;
		if (!wirelessMasterDevice->enableRadio(desiredRadioChannel))
		{
            sprintf(messageDesc, "Failed to set radio channel");
            printf("%s\n", messageDesc);
            publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_GENERAL_ERROR, kinoped_msgs::kinoped_message::KINOPED_MSG_FATAL, messageDesc);
			std::ostringstream error;
			error << "Failed to set radio channel";
			throw std::runtime_error(error.str());
		}

		std::cout << "Waiting for MTW to wirelessly connect...\n" << std::endl;

		bool waitForConnections = true;
		size_t connectedMTWCount = wirelessMasterCallback.getWirelessMTWs().size();
		do
		{
			XsTime::msleep(100);

			while (true)
			{
				size_t nextCount = wirelessMasterCallback.getWirelessMTWs().size();
				if (nextCount != connectedMTWCount)
				{
					std::cout << "Number of connected MTWs: " << nextCount << ". Press 'Y' to start measurement." << std::endl;
					connectedMTWCount = nextCount;
				}
				else
				{
					break;
				}
			}
		}
		while (connectedMTWCount < 6);
        
		XsDeviceIdArray allDevIds = control->deviceIds();
		XsDeviceIdArray mtwDevIds;
        // publish the list of MAC addresses to the UI so that the operator doesn't have to type them all in
		for (XsDeviceIdArray::const_iterator i = allDevIds.begin(); i != allDevIds.end(); ++i)
		{
			if (i->isMtw())
			{
                char id[16];
                memset(id, 0, sizeof(id));
                sprintf(id,"%08X", i->toInt());
                macAddresses.mac_addresses.push_back(id);
			}
		}
        macAddresses.header.stamp = ros::Time::now();
        macPub.publish(macAddresses);
        
		std::cout << "Setting config mode..." << std::endl;
		if (!wirelessMasterDevice->gotoConfig())
		{
            sprintf(messageDesc, "Failed to goto config mode");
            printf("%s\n", messageDesc);
            publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_GENERAL_ERROR, kinoped_msgs::kinoped_message::KINOPED_MSG_FATAL, messageDesc);
			std::cout << "Failed to goto config mode" << std::endl;
		}

		std::cout << "Disabling radio... " << std::endl;
		if (!wirelessMasterDevice->disableRadio())
		{
            sprintf(messageDesc, "Failed to disable radio");
            printf("%s\n", messageDesc);
            publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_GENERAL_ERROR, kinoped_msgs::kinoped_message::KINOPED_MSG_FATAL, messageDesc);
			std::cout << "Failed to disable radio" << std::endl;
		}
	}
	catch (std::exception const & ex)
	{
		std::cout << ex.what() << std::endl;
		std::cout << "****ABORT****" << std::endl;
        sprintf(messageDesc, "****ABORT****");
        printf("%s\n", messageDesc);
        publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_GENERAL_ERROR, kinoped_msgs::kinoped_message::KINOPED_MSG_FATAL, messageDesc);
	}
	catch (...)
	{
		std::cout << "An unknown fatal error has occured. Aborting." << std::endl;
		std::cout << "****ABORT****" << std::endl;
        sprintf(messageDesc, "An unknown fatal error has occured. Aborting.");
        printf("%s\n", messageDesc);
        publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_GENERAL_ERROR, kinoped_msgs::kinoped_message::KINOPED_MSG_FATAL, messageDesc);
	}

	std::cout << "Closing XsControl..." << std::endl;
	control->close();

	std::cout << "Deleting mtw callbacks..." << std::endl;
	for (std::vector<MtwCallback*>::iterator i = mtwCallbacks.begin(); i != mtwCallbacks.end(); ++i)
	{
		delete (*i);
	}

	std::cout << "Successful publishXsensSensorMACS." << std::endl;
    sprintf(messageDesc, "Successful publishXsensSensorMACS.");
    printf("%s\n", messageDesc);
    publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_OK, kinoped_msgs::kinoped_message::KINOPED_MSG_INFO, messageDesc);
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
    int publishThrottle = 0;
     //Initializes ROS, and sets up a node
    ros::init(argc, argv, "xsensThread");
    ros::NodeHandle nh;

    // we need to use AsyncSpinner in order to have multiple subscriber callbacks work correctly
    ros::AsyncSpinner spinner(0);
    spinner.start();

    ros::Subscriber subUserCommand = nh.subscribe("user_command", 1000, ucsubCallback);
    ros::Subscriber subXsensSensorMapping = nh.subscribe("xsens_sensors", 1000, xsensSensorMappingCallback);
    statusPub = nh.advertise<kinoped_msgs::kinoped_message>("xsens_status", 1000);
    macPub = nh.advertise<kinoped_msgs::xsens_sensors>("xsens_macs", 1000);

    ros::Rate rate(100);

    init_data();
    
    while (KINOPED_RUN_MODE_END != run_mode)
    {
        publishThrottle = 0;
        publishXsensSensorMACS();
        while (!mappingReceived)
        {
            if (0 == (publishThrottle % 10))
            {
	            sprintf(messageDesc, "xsensThread - waiting for Xsens Sensor Mapping data");
				printf("%s\n", messageDesc);
			    publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_WAITING_FOR_SENSOR_MAP, kinoped_msgs::kinoped_message::KINOPED_MSG_INFO, messageDesc);
            }
            publishThrottle++;
            sleep(1);
            if (KINOPED_RUN_MODE_END == run_mode)
            {
                sprintf(messageDesc, "xsensThread - end command received");
			    printf("%s\n", messageDesc);
			    publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_END_RECEIVED, kinoped_msgs::kinoped_message::KINOPED_MSG_INFO, messageDesc);
                exit(0);
            }
        }
        publishThrottle = 0;
        while (!startReceived)
        {
            if (0 == (publishThrottle % 10))
            {
	            sprintf(messageDesc, "xsensThread - waiting for Start message");
				printf("%s\n", messageDesc);
			    publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_WAITING_FOR_START_COMMAND, kinoped_msgs::kinoped_message::KINOPED_MSG_INFO, messageDesc);
            }
            publishThrottle++;
            sleep(1);
            if (KINOPED_RUN_MODE_END == run_mode)
            {
                sprintf(messageDesc, "xsensThread - end command received");
			    printf("%s\n", messageDesc);
			    publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_END_RECEIVED, kinoped_msgs::kinoped_message::KINOPED_MSG_INFO, messageDesc);
                exit(0);
            }
        }
        if (startReceived)
        {
            printf("DB Name = %s\n", dbName);
            char c1[] = {"postgres"};
            char c2[] = {"user_tracking"};
            char c3[] = {DROP_USER_TRACKING_TABLE};
            char c4[] = {CREATE_USER_TRACKING_TABLE};
            char c5[] = {USER_TRACKING_ROW_INSERT_STRING};

            xsens_data = new KinopedDB((char*)dbName, (char*)c1, (char*)c2, (char*)c3, (char*)c4, (char*)c5);
            xsens_data->init_database_connection();
            
            if (pthread_create(&kinoped_xsens_thread_id, NULL, kinoped_xsens_thread, 0))
            {
                sprintf(messageDesc, "Fail...Cannot spawn the Xsens thread.");
			    printf("%s\n", messageDesc);
			    publishStatus(kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_GENERAL_ERROR, kinoped_msgs::kinoped_message::KINOPED_MSG_FATAL, messageDesc);
                exit(-1);
            }
            keepRunning = 1;
            xsensThread_keep_running = 1;
            done = 0;

            while(ros::ok() && keepRunning)
            {
                // main loop doesn't really do anything except check for when to restart or end
                sleep(1);
            }
            xsensThread_keep_running = 0;
            sleep(1);

            // write any remaining buffered data
            xsens_data->write_buffered_data(positionData, numBufferedXsensItems);
            xsens_data->close_database_connection();
            delete xsens_data;
            mappingReceived = 0;
            numBufferedXsensItems = 0;
            startReceived = 0;
        }
    }
    sleep(1);
    exit(0);
}
