#include <math.h>
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include "std_msgs/String.h"
#include "Definitions.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <sstream>
#include "std_msgs/Int64.h"
#include "std_msgs/Float64.h"

typedef void* HANDLE;
typedef int BOOL;

using namespace std;

void* g_pKeyHandle = 0;
int vel[] = {0,0,0,0};
unsigned short g_usNodeId[] = {1,2,3,4};
int Id_length = sizeof(g_usNodeId)/sizeof(*g_usNodeId);
string g_deviceName;
string g_protocolStackName;
string g_interfaceName;
string g_portName;
int g_baudrate = 0;


#ifndef MMC_SUCCESS
#define MMC_SUCCESS 0
#endif

#ifndef MMC_FAILED
#define MMC_FAILED 1
#endif

#ifndef MMC_MAX_LOG_MSG_SIZE
#define MMC_MAX_LOG_MSG_SIZE 512
#endif


#define PI 3.14159265358979323846

double wheel_radius = 0.076;
double l = 0.5165;
double gear_ratio = 73.5;
double radps_to_rpm = 60.0 / 2.0 / PI;
double rpm_to_radps = 2.0 * PI / 60;

// Saturate velocity with l1 norm ball
double norm_limit = 0.74;

double linear_x_d = 0;
double linear_y_d = 0;
double angular_z_d = 0;

void SeparatorLine()
{
    const int lineLength = 65;
    for(int i=0; i<lineLength; i++)
    {
        cout << "-";
    }
    cout << endl;
}

void PrintSettings()
{
    std_msgs::String msg;

    std::stringstream ss;
    ss << endl;

    ss << "number of node      = " << Id_length << endl;
    ss << "device name         = '" << g_deviceName << "'" << endl;
    ss << "protocal stack name = '" << g_protocolStackName << "'" << endl;
    ss << "interface name      = '" << g_interfaceName << "'" << endl;
    ss << "port name           = '" << g_portName << "'"<< endl;
    ss << "baudrate            = " << g_baudrate;

    msg.data = ss.str();
    ROS_INFO("%s",msg.data.c_str());

    SeparatorLine();
}

void SetDefaultParameters()
{
    //USB
    g_deviceName = "EPOS4";
    g_protocolStackName = "MAXON SERIAL V2";
    g_interfaceName = "USB";
    g_portName = "USB0";
    g_baudrate = 1000000;
}

int OpenDevice(unsigned int* p_pErrorCode)
{
    int lResult = MMC_FAILED;

    char* pDeviceName = new char[255];
    char* pProtocolStackName = new char[255];
    char* pInterfaceName = new char[255];
    char* pPortName = new char[255];

    strcpy(pDeviceName, g_deviceName.c_str());
    strcpy(pProtocolStackName, g_protocolStackName.c_str());
    strcpy(pInterfaceName, g_interfaceName.c_str());
    strcpy(pPortName, g_portName.c_str());

    ROS_INFO("Open device...");

    g_pKeyHandle = VCS_OpenDevice(pDeviceName, pProtocolStackName, pInterfaceName, pPortName, p_pErrorCode);

    if(g_pKeyHandle!=0 && *p_pErrorCode == 0)
    {
        unsigned int lBaudrate = 0;
        unsigned int lTimeout = 0;

        if(VCS_GetProtocolStackSettings(g_pKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=0)
        {
            if(VCS_SetProtocolStackSettings(g_pKeyHandle, g_baudrate, lTimeout, p_pErrorCode)!=0)
            {
                if(VCS_GetProtocolStackSettings(g_pKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=0)
                {
                    if(g_baudrate==(int)lBaudrate)
                    {
                        lResult = MMC_SUCCESS;
                        ROS_INFO("Open device Success");
                    }
                }
            }
        }
    }
    else
    {
        g_pKeyHandle = 0;
    }

    delete []pDeviceName;
    delete []pProtocolStackName;
    delete []pInterfaceName;
    delete []pPortName;

    return lResult;
}

int CloseDevice(unsigned int* p_pErrorCode)
{
    int lResult = MMC_FAILED;

    *p_pErrorCode = 0;

    ROS_INFO("Close device");

    if(VCS_CloseDevice(g_pKeyHandle, p_pErrorCode)!=0 && *p_pErrorCode == 0)
    {
        lResult = MMC_SUCCESS;
    }

    return lResult;
}


bool ProfileVelocityMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode, long vel_d)
{
    int lResult = MMC_SUCCESS;

//    ROS_INFO("move with target velocity : %ld rpm, node = %d", vel_d, p_usNodeId);

    if(VCS_MoveWithVelocity(p_DeviceHandle, p_usNodeId, vel_d, &p_rlErrorCode) == 0)
    {
        lResult = MMC_FAILED;
        ROS_INFO("VCS_MoveWithVelocity failed");
    }

    return lResult;
}

int PrepareDriver(unsigned short NodeId, unsigned int* p_pErrorCode)
{
    int lResult = MMC_SUCCESS;
    BOOL oIsFault = 0;

    if(VCS_GetFaultState(g_pKeyHandle, NodeId, &oIsFault, p_pErrorCode ) == 0)
    {
//        ROS_INFO("Device (node: %d) is in fault state", NodeId);
        lResult = MMC_FAILED;
    }

    if(lResult==0)
    {
        if(oIsFault)
        {
//            ROS_INFO("Clear fault state (node: %d)", NodeId);
            if(VCS_ClearFault(g_pKeyHandle, NodeId, p_pErrorCode) == 0)
            {
//                ROS_INFO("Clearing is failed")
                lResult = MMC_FAILED;
            }
        }

        if(lResult==0)
        {
            BOOL oIsEnabled = 0;

            if(VCS_GetEnableState(g_pKeyHandle, NodeId, &oIsEnabled, p_pErrorCode) == 0)
            {
                lResult = MMC_FAILED;
            }

            if(lResult==0)
            {
                if(!oIsEnabled)
                {
                    if(VCS_SetEnableState(g_pKeyHandle, NodeId, p_pErrorCode) == 0)
                    {
                        ROS_INFO("Device (node: %d) is enabled", NodeId);
                        lResult = MMC_FAILED;
                    }
                }
            }
        }
    }
    return lResult;
}

void commandCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
    int lResult = MMC_FAILED;
    unsigned int lErrorCode = 0;

    //int64_t setVel[] = {msg->data[0], msg->data[1], msg->data[2], msg->data[3]};
    int64_t setVel[] = {0, 0, 0, 0};

    double u_x = 0;
    double u_y = 0;
    double u_p = 0;
    linear_x_d  = cmd_vel->linear.x;
    linear_y_d  = cmd_vel->linear.y;
    angular_z_d = cmd_vel->angular.z;

    double norm_desired = abs(linear_x_d) + abs(linear_y_d) + abs(l * angular_z_d);

    if (norm_desired > 0.01)
    {
        if (norm_desired > norm_limit)
        {
            u_p = norm_limit / norm_desired * angular_z_d;
            u_x = norm_limit / norm_desired * linear_x_d;
            u_y = norm_limit / norm_desired * linear_y_d;
        }
        else
        {
            u_p = angular_z_d;
            u_x = linear_x_d;
            u_y = linear_y_d;
        }

        setVel[0] = int( radps_to_rpm / wheel_radius * ( u_x - u_y - l * u_p) * gear_ratio);
        setVel[1] = int(-radps_to_rpm / wheel_radius * ( u_x + u_y + l * u_p) * gear_ratio);
        setVel[3] = int( radps_to_rpm / wheel_radius * ( u_x + u_y - l * u_p) * gear_ratio);
        setVel[2] = int(-radps_to_rpm / wheel_radius * ( u_x - u_y + l * u_p) * gear_ratio);
    }

    ROS_INFO("Desired velocity (rpm) = %ld, %ld, %ld, %ld", (long int)setVel[0], (long int)setVel[1], (long int)setVel[2], (long int)setVel[3]);

    for (int i=0; i<Id_length;i++)
    {
        if((lResult = ProfileVelocityMode(g_pKeyHandle, g_usNodeId[i], lErrorCode, setVel[i])!=MMC_SUCCESS))
        {
            ROS_INFO("Velocity Control Failed node: %d", g_usNodeId[i]);
        }
    }
}

tf::Transform getTransformForMotion(double linear_vel_x, double linear_vel_y, double angular_vel, double timeSeconds)
{
    tf::Transform tmp;
    tmp.setIdentity();
    
    
    if (abs(angular_vel) < 0.0001) {
        //Drive straight
        tmp.setOrigin(tf::Vector3(static_cast<double>(linear_vel_x*timeSeconds), static_cast<double>(linear_vel_y*timeSeconds), 0.0));
    }
    else
    {
        //Follow circular arc
        double angleChange = angular_vel * timeSeconds;
        double arcRadius_x = linear_vel_x * timeSeconds / angleChange;
        double arcRadius_y = linear_vel_y * timeSeconds / angleChange;
        
        tmp.setOrigin(tf::Vector3(sin(angleChange) * arcRadius_x + cos(angleChange) * arcRadius_y - arcRadius_y,
        						  sin(angleChange) * arcRadius_y - cos(angleChange) * arcRadius_x + arcRadius_x,
        						  0.0));
        tmp.setRotation(tf::createQuaternionFromYaw(angleChange));
    }
    
    return tmp;
}
nav_msgs::Odometry computeOdometry(double linear_vel_x, double linear_vel_y, double angular_vel_z, double step_time)
{
    nav_msgs::Odometry odom;

    tf::Transform odom_transform =
	      odom_transform * getTransformForMotion(
				    linear_vel_x, linear_vel_y, angular_vel_z, step_time);
		tf::poseTFToMsg(odom_transform, odom.pose.pose);

    odom.twist.twist.linear.x  = linear_vel_x;
		odom.twist.twist.linear.y  = linear_vel_y;
		odom.twist.twist.angular.z = angular_vel_z;

		odom.header.stamp = ros::Time::now();
		odom.header.frame_id = "odom";
		odom.child_frame_id = "base_footprint";

		odom.pose.covariance[0] = 0.001;
		odom.pose.covariance[7] = 0.001;
		odom.pose.covariance[14] = 1000000000000.0;
		odom.pose.covariance[21] = 1000000000000.0;
		odom.pose.covariance[28] = 1000000000000.0;
		
		if ( abs(angular_vel_z) < 0.0001 )
    {
	      odom.pose.covariance[35] = 0.01;
		}
    else
    {
	      odom.pose.covariance[35] = 100.0;
		}

		odom.twist.covariance[0] = 0.001;
		odom.twist.covariance[7] = 0.001;
		odom.twist.covariance[14] = 0.001;
		odom.twist.covariance[21] = 1000000000000.0;
		odom.twist.covariance[28] = 1000000000000.0;

		if ( abs(angular_vel_z) < 0.0001 )
    {
	      odom.twist.covariance[35] = 0.01;
		}
    else
    {
	      odom.twist.covariance[35] = 100.0;
		}

    return odom;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motor_controller");
    ros::NodeHandle nh;
    ros::NodeHandle ph;

    tf::Transform odom_transform;
    odom_transform.setIdentity();

    ros::Time last_odom_publish_time = ros::Time::now();

    int lResult = MMC_FAILED;
    unsigned int ulErrorCode = 0;
    SetDefaultParameters();
    PrintSettings();

    if ((lResult = OpenDevice(&ulErrorCode))!=MMC_SUCCESS)
    {
        ROS_INFO("OpenDevice Failed");
    }

    for (int i=0; i< Id_length;i++)
    {
        if ((lResult = PrepareDriver(g_usNodeId[i], &ulErrorCode))!=MMC_SUCCESS)
        {
            ROS_INFO("PrepareDemo Failed node: %d", g_usNodeId[i]);
            return 0;
        }
    }

    for (int i=0; i<sizeof(g_usNodeId)/sizeof(*g_usNodeId);i++)
    {
        if (VCS_ActivateProfileVelocityMode(g_pKeyHandle, g_usNodeId[i], &ulErrorCode) == 0)
        {
            ROS_INFO("VCS_ActivateProfileVelocityMode node: %d Failed", g_usNodeId[i]);
            lResult = MMC_FAILED;
        }
        VCS_SetVelocityProfile(g_pKeyHandle, g_usNodeId[i], 5000, 5000, &ulErrorCode);
    }

    ROS_INFO("Ready to control velocity");

    ros::Subscriber sub = nh.subscribe("cmd_vel", 10, commandCallback);
    ros::Publisher odom_pub = ph.advertise<nav_msgs::Odometry>("wheel_encoder/odom", 10);
    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        for (int i=0; i<Id_length;i++)
        {
            VCS_GetVelocityIsAveraged(g_pKeyHandle, g_usNodeId[i], &vel[i], &ulErrorCode);
        }
        ros::Time current_time = ros::Time::now();

		    double wheel_speed_lf = (double)  vel[0] * rpm_to_radps / gear_ratio;
		    double wheel_speed_rf = (double) -vel[1] * rpm_to_radps / gear_ratio;
		    double wheel_speed_lb = (double)  vel[3] * rpm_to_radps / gear_ratio;
		    double wheel_speed_rb = (double) -vel[2] * rpm_to_radps / gear_ratio;

		    double linear_vel_x =
            wheel_radius/4.0*(wheel_speed_lf+wheel_speed_rf+wheel_speed_lb+wheel_speed_rb);
		    double linear_vel_y =
            wheel_radius/4.0*(-wheel_speed_lf+wheel_speed_rf+wheel_speed_lb-wheel_speed_rb);
		    double angular_vel_z =
            wheel_radius/(4.0*l)*(-wheel_speed_lf+wheel_speed_rf-wheel_speed_lb+wheel_speed_rb);

        double step_time = current_time.toSec() - last_odom_publish_time.toSec();
		    last_odom_publish_time = current_time;

        nav_msgs::Odometry odom_msg = computeOdometry(linear_vel_x, linear_vel_y, angular_vel_z, step_time);

        odom_pub.publish(odom_msg);
        
        ros::spinOnce();

        loop_rate.sleep();
    }

    if((lResult = CloseDevice(&ulErrorCode))!=MMC_SUCCESS)
    {
        ROS_INFO("CloseDevice");
        return lResult;
    }

	  return 0;

}

