#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sstream>
#include <iostream>

#include <net/if.h>
#include <time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#define CANID_DELIM '#'
#define DATA_SEPERATOR '.'

typedef void* HANDLE;
typedef int BOOL;

using namespace std;

void* g_pKeyHandle = 0;
int vel[] = {0, 0, 0, 0};
unsigned short g_usNodeId[] = {1,2,3,4};
int Id_length = sizeof(g_usNodeId)/sizeof(*g_usNodeId);

// CAN Parameters
int s;
struct can_frame frame;
string data;
string vel_desired_hex[4];
int setVel[] = {0, 0, 0, 0};
bool setVel_lock = false;

string Control_Enable = "0F00";
unsigned short COB_ID_Rx[][4] = {
  {0x221, 0x321, 0x421, 0x521},
  {0x222, 0x322, 0x422, 0x522},
  {0x223, 0x323, 0x423, 0x523},
  {0x224, 0x324, 0x424, 0x524}
};
unsigned short COB_ID_Tx[][4] = {
  {0x1A1, 0x2A1, 0x3A1, 0x4A1},
  {0x1A2, 0x2A2, 0x3A2, 0x4A2},
  {0x1A3, 0x2A3, 0x3A3, 0x4A3},
  {0x1A4, 0x2A4, 0x3A4, 0x4A4}
};
string COB_ID_Rx2[][4] = {
  {"221", "321", "421", "521"},
  {"222", "322", "422", "522"},
  {"223", "323", "423", "523"},
  {"224", "324", "424", "524"}
};


// platform param
#define PI 3.14159265358979323846

double wheel_radius = 0.076;
double l = 0.5165;
double gear_ratio = 73.5;
double radps_to_rpm = 60.0 / 2.0 / PI;
double rpm_to_radps = 2.0 * PI / 60;

double param_IK = radps_to_rpm * gear_ratio / wheel_radius; // Parameter for inverse kinematics
double param_FK_linear = wheel_radius * rpm_to_radps / gear_ratio / 4.0;
double param_FK_angular = param_FK_linear / l;

double linear_x_d = 0;
double linear_y_d = 0;
double angular_z_d = 0;

// Saturate velocity with l1 norm ball
double norm_limit = 0.74;


unsigned char asc2nibble(char c) {

  if ((c >= '0') && (c <= '9'))
    return c - '0';

  if ((c >= 'A') && (c <= 'F'))
    return c - 'A' + 10;

  if ((c >= 'a') && (c <= 'f'))
    return c - 'a' + 10;

  return 16; /* error */
}


int hexstring2data(char *arg, unsigned char *data, int maxdlen) {

  int len = strlen(arg);
  int i;
  unsigned char tmp;

  if (!len || len%2 || len > maxdlen*2)
    return 1;

  memset(data, 0, maxdlen);

  for (i=0; i < len/2; i++) {

    tmp = asc2nibble(*(arg+(2*i)));
    if (tmp > 0x0F)
      return 1;

    data[i] = (tmp << 4);

    tmp = asc2nibble(*(arg+(2*i)+1));
    if (tmp > 0x0F)
      return 1;

    data[i] |= tmp;
  }

  return 0;
}


int parse_canframe(char *cs, struct canfd_frame *cf)
{
  /* documentation see lib.h */

  int i, idx, dlen, len;
  int maxdlen = CAN_MAX_DLEN;
  int ret = CAN_MTU;
  unsigned char tmp;

  len = strlen(cs);

  memset(cf, 0, sizeof(*cf)); /* init CAN FD frame, e.g. LEN = 0 */

  if (len < 4)
    return 0;

  if (cs[3] == CANID_DELIM) /* 3 digits */
  {

    idx = 4;
    for (i=0; i<3; i++){
      if ((tmp = asc2nibble(cs[i])) > 0x0F)
        return 0;
      cf->can_id |= (tmp << (2-i)*4);
    }

  }
  else if (cs[8] == CANID_DELIM) /* 8 digits */
  {

    idx = 9;
    for (i=0; i<8; i++){
      if ((tmp = asc2nibble(cs[i])) > 0x0F)
        return 0;
      cf->can_id |= (tmp << (7-i)*4);
    }
    if (!(cf->can_id & CAN_ERR_FLAG)) /* 8 digits but no errorframe?  */
      cf->can_id |= CAN_EFF_FLAG;   /* then it is an extended frame */

  }
  else
  {
    return 0;
  }

  if((cs[idx] == 'R') || (cs[idx] == 'r')) /* RTR frame */
  {
    cf->can_id |= CAN_RTR_FLAG;

    /* check for optional DLC value for CAN 2.0B frames */
    if(cs[++idx] && (tmp = asc2nibble(cs[idx])) <= CAN_MAX_DLC)
      cf->len = tmp;

    return ret;
  }

  if (cs[idx] == CANID_DELIM) /* CAN FD frame escape char '##' */
  {

    maxdlen = CANFD_MAX_DLEN;
    ret = CANFD_MTU;

    /* CAN FD frame <canid>##<flags><data>* */
    if ((tmp = asc2nibble(cs[idx+1])) > 0x0F)
      return 0;

    cf->flags = tmp;
    idx += 2;
  }

  for (i=0, dlen=0; i < maxdlen; i++){

    if(cs[idx] == DATA_SEPERATOR) /* skip (optional) separator */
      idx++;

    if(idx >= len) /* end of string => end of data */
      break;

    if ((tmp = asc2nibble(cs[idx++])) > 0x0F)
      return 0;
    cf->data[i] = (tmp << 4);
    if ((tmp = asc2nibble(cs[idx++])) > 0x0F)
      return 0;
    cf->data[i] |= tmp;
    dlen++;
  }
  cf->len = dlen;

  return ret;

} // end parse_canframe(char *cs, struct canfd_frame *cf)


void LogInfo(string message)
{
  cout << message << endl;
}


template <typename T>
std::string dec_to_hex(T dec, int NbofByte){
  std::stringstream stream_HL;
  string s, s_LH;
  stream_HL << std::setfill ('0') << std::setw(sizeof(T)*2) <<std::hex << dec;
  s = stream_HL.str();
  for (int i=0; i<NbofByte; i++){
    s_LH.append(s.substr(2*(NbofByte-1-i),2));
  }
  return s_LH;
}


int hexarray_to_int(unsigned char *buffer, int length){
  int hextoint = 0;
  for (int i=0; i<length; i++)
  {
    hextoint += (buffer[i] << 8*i);
  }
  return hextoint;
}


double hexarray_to_double(unsigned char *buffer, int length){
  int hextoint = 0;
  for (int i=0; i<length; i++)
  {
    hextoint += (buffer[i] << 8*i);
  }
  return double(hextoint);
}


std::string stringappend(string a, string b)
{
  string s;
  s.append(a);
  s.append(b);
  return s;
}


void commandCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{

  linear_x_d  = cmd_vel->linear.x;
  linear_y_d  = cmd_vel->linear.y;
  angular_z_d = cmd_vel->angular.z;

  double norm_desired = abs(linear_x_d) + abs(linear_y_d) + abs(l * angular_z_d);

  if (norm_desired > 0.0001)
  {
    if (norm_desired > norm_limit)
    {
      double scale_factor = norm_limit / norm_desired;
      angular_z_d *= scale_factor;
      linear_x_d  *= scale_factor;
      linear_y_d  *= scale_factor;
    }
    while (setVel_lock) {}
    setVel_lock = true;
    setVel[0] = int( param_IK * (linear_x_d - linear_y_d - l * angular_z_d));
    setVel[1] = int(-param_IK * (linear_x_d + linear_y_d + l * angular_z_d));
    setVel[2] = int(-param_IK * (linear_x_d - linear_y_d + l * angular_z_d));
    setVel[3] = int( param_IK * (linear_x_d + linear_y_d - l * angular_z_d));
    setVel_lock = false;
  }
  else
  {
    while (setVel_lock) {}
    setVel_lock = true;
    setVel[0] = 0;
    setVel[1] = 0;
    setVel[2] = 0;
    setVel[3] = 0;
    setVel_lock = false;
  }

} // end commandCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel)


tf::Transform getTransformForMotion(
  double linear_vel_x, double linear_vel_y, double angular_vel, double timeSeconds)
{
  tf::Transform tmp;
  tmp.setIdentity();
  
  
  if (abs(angular_vel) < 0.0001)
  {
    //Drive straight
    tmp.setOrigin(
      tf::Vector3(
        static_cast<double>(linear_vel_x*timeSeconds),
        static_cast<double>(linear_vel_y*timeSeconds),
        0.0
      )
    );
    tmp.setRotation(tf::createQuaternionFromYaw(0.0));
  }
  else
  {
    //Follow circular arc
    double angleChange = angular_vel * timeSeconds;
    double arcRadius_x = linear_vel_x * timeSeconds / angleChange;
    double arcRadius_y = linear_vel_y * timeSeconds / angleChange;
    
    tmp.setOrigin(
      tf::Vector3(
        sin(angleChange) * arcRadius_x + cos(angleChange) * arcRadius_y - arcRadius_y,
        sin(angleChange) * arcRadius_y - cos(angleChange) * arcRadius_x + arcRadius_x,
        0.0
      )
    );
    tmp.setRotation(tf::createQuaternionFromYaw(angleChange));
  }
  
  return tmp;

} // end tf::Transform getTransformForMotion


nav_msgs::Odometry computeOdometry(
  double linear_vel_x, double linear_vel_y, double angular_vel_z, double step_time)
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

} // end computeOdometry


int main(int argc, char **argv)
{
  ros::init(argc, argv, "motor_controller");
  ros::NodeHandle nh;
  string rtr;
  unsigned char buffer[4];

  // SocketCAN Initialize
  struct sockaddr_can addr;
  //struct can_frame frame;
  struct canfd_frame frame_fd;
  int required_mtu;
  struct ifreq ifr;

  struct iovec iov;
  struct msghdr canmsg;
  char ctrlmsg[CMSG_SPACE(sizeof(struct timeval) + 3*sizeof(struct timespec) + sizeof(__u32))];
  struct canfd_frame frame_get;

  const char *ifname = "slcan0";

  if (s = socket(PF_CAN, SOCK_RAW, CAN_RAW) < 0)
  {
    perror("Error while opening socket");
    return -1;
  }

  strcpy(ifr.ifr_name, ifname);
  ioctl(s, SIOCGIFINDEX, &ifr);

  addr.can_family  = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  printf("%s at index %d\n", ifname, ifr.ifr_ifindex);

  if (bind(s, (struct sockaddr *) &addr, sizeof(addr)) < 0)
  {
    perror("Error in socket bind");
    return -2;
  }

  // Initialize NMT Services
  LogInfo("Initialize NMT Services");
  // Reset Communication
  frame.can_id  = 0x000;
  frame.can_dlc = 2;
  frame.data[0] = 0x81;
  frame.data[1] = 0x00;
  write(s, &frame, sizeof(struct can_frame));
  LogInfo("Reset Communication");
  sleep(2);

  // Start Remote Node
  frame.can_id  = 0x000;
  frame.can_dlc = 2;
  frame.data[0] = 0x01;
  frame.data[1] = 0x00;
  write(s, &frame, sizeof(struct can_frame));
  LogInfo("Start Remote Node");
  sleep(1);

  // Velocity Control mode Initialize
  for (int i=0; i<Id_length; i++)
  {
    // Modes of operation (PVM)
    frame.can_id  = COB_ID_Rx[i][1];
    frame.can_dlc = 3;
    frame.data[0] = 0x00;
    frame.data[1] = 0x00;
    frame.data[2] = 0x03;
    write(s, &frame, sizeof(struct can_frame));
    LogInfo("Velocitiy mode initialized");
    sleep(1);

    // Shutdown Controlword
    frame.can_id  = COB_ID_Rx[i][0];
    frame.can_dlc = 2;
    frame.data[0] = 0x06;
    frame.data[1] = 0x00;
    write(s, &frame, sizeof(struct can_frame));
    LogInfo("Shutdown controlword");
    sleep(1);

    // Enable Controlword
    frame.can_id  = COB_ID_Rx[i][0];
    frame.can_dlc = 2;
    frame.data[0] = 0x0F;
    frame.data[1] = 0x00;
    write(s, &frame, sizeof(struct can_frame));
    LogInfo("Enable controlword");
    sleep(1);
  }

  ros::Subscriber sub = nh.subscribe("cmd_vel", 10, commandCallback);
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("wheel_encoder/odom", 10);
  ros::Rate loop_rate(100);

  iov.iov_base = &frame_get;
  canmsg.msg_name = &addr;
  canmsg.msg_iov = &iov;
  canmsg.msg_iovlen = 1;
  canmsg.msg_control = &ctrlmsg;

  iov.iov_len = sizeof(frame_get);
  canmsg.msg_namelen = sizeof(addr);
  canmsg.msg_controllen = sizeof(ctrlmsg);
  canmsg.msg_flags = 0;
  sleep(1);

  int nnbytes = 0;
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 5000;
  setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);

  for (int i=0; i<3; i++)
  {
    recvmsg(s, &canmsg, 0);
  }

  tf::Transform odom_transform;
  odom_transform.setIdentity();

  ros::Time last_odom_publish_time = ros::Time::now();

  while (ros::ok())
  {
    for (int i=0; i<Id_length;i++)
    {
      frame.can_id  = COB_ID_Tx[i][1] | CAN_RTR_FLAG;
      write(s, &frame, sizeof(struct can_frame));
      nnbytes = recvmsg(s,&canmsg, 0);
      vel[i] = hexarray_to_double(frame_get.data,4);
    }

    ros::Time current_time = ros::Time::now();

    double linear_vel_x  = param_FK_linear  * ( vel[0] - vel[1] - vel[2] + vel[3]);
    double linear_vel_y  = param_FK_linear  * (-vel[0] - vel[1] + vel[2] + vel[3]);
    double angular_vel_z = param_FK_angular * (-vel[0] - vel[1] - vel[2] - vel[3]);

    double step_time = current_time.toSec() - last_odom_publish_time.toSec();
    last_odom_publish_time = current_time;

    nav_msgs::Odometry odom_msg = computeOdometry(linear_vel_x, linear_vel_y, angular_vel_z, step_time);
    odom_pub.publish(odom_msg);

    while (setVel_lock) {}
    setVel_lock = true;
    for (int i = 0; i < Id_length; i++) {
      // Convert dec to hex
      vel_desired_hex[i] = dec_to_hex(setVel[i], 4);
    }
    setVel_lock = false;

    for (int i = 0; i < Id_length; i++) {
      // Send Profile Velocity control command to EPOS
      frame.can_id = COB_ID_Rx[i][3];
      frame.can_dlc = 6;
      data = stringappend(Control_Enable, vel_desired_hex[i]);
      hexstring2data((char *) data.c_str(), frame.data, 8);
      write(s, &frame, sizeof(struct can_frame));
      ros::Duration(0.00001).sleep();
    }

    ros::spinOnce();

    loop_rate.sleep();

  } // end while (ros::ok())

  // Reset Remote Node
  frame.can_id  = 0x000;
  frame.can_dlc = 2;
  frame.data[0] = 0x81;
  frame.data[1] = 0x00;
  write(s, &frame, sizeof(struct can_frame));
  LogInfo("Reset Remote Node");
  sleep(2);

  // Stop Remote Node
  frame.can_id  = 0x000;
  frame.can_dlc = 2;
  frame.data[0] = 0x02;
  frame.data[1] = 0x00;
  write(s, &frame, sizeof(struct can_frame));
  LogInfo("Stop Remote Node");

  return 0;

} // end main
