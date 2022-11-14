#include "ros/ros.h"
#include "math.h"
#include "fiducial_msgs/FiducialTransformArray.h"
#include "fiducial_msgs/Fiducial.h"
#include "geometry_msgs/Twist.h"
//#include "q2e_msgs/EulerRotation.h"

//ros::Publisher* euler_xyz;  // euler_xyz -> publish(euler);
//q2e::euler_rotation euler;

double QX = 0;
double QY = 0;
double QZ = 0;
double QW = 0;

double t0 = 0;
double t1 = 0;
double t2 = 0;
double t3 = 0;
double t4 = 0;

double roll = 0;
double pitch = 0;
double yaw = 0;


void Quaternion_callback(const fiducial_msgs::FiducialTransformArray &msg)
{

  for(int i=0; i<msg.transforms.size() ; i++)
  {
    //ROS_INFO("MARKER_ID : %d", msg.transforms[i].fiducial_id);
    //ROS_INFO("TRANS_X : %f", msg.transforms[i].transform.translation.x);
    //ROS_INFO("TRANS_Y : %f", msg.transforms[i].transform.translation.y);
    //ROS_INFO("TRANS_Z : %f", msg.transforms[i].transform.translation.z);
    //ROS_INFO("QUATER_X : %f", msg.transforms[i].transform.rotation.x);
    //ROS_INFO("QUATER_Y : %f", msg.transforms[i].transform.rotation.y);
    //ROS_INFO("QUATER_Z : %f", msg.transforms[i].transform.rotation.z);
    //ROS_INFO("QUATER_W : %f", msg.transforms[i].transform.rotation.w);
    //ROS_INFO("====================");

    QX = msg.transforms[i].transform.rotation.x;
    QY = msg.transforms[i].transform.rotation.y;
    QZ = msg.transforms[i].transform.rotation.z;
    QW = msg.transforms[i].transform.rotation.w;

  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "q2e");

  ros::NodeHandle nh;
  ros::Rate loop_rate(10);

  ros::Subscriber Quaternion_sub = nh.subscribe("/fiducial_transforms", 1000, Quaternion_callback);

  //ros::Publisher Euler_pub = n.advertise<q2e::euler_rotation>("fiducial_euler", 1000);
  //euler_xyz = &Euler_pub;

  while (ros::ok())
  {
    t0 = 2.0 * (QW * QX + QY * QZ);
    t1 = 1.0 - 2.0 * (QX * QX + QY *QY);
    roll = atan2(t0, t1) * 57.29577951323;
    t2 = 2.0 * (QW * QY - QZ * QX);

    if (t2 > 1.0) t2 = 1.0; 
    else t2 = t2;
    if (t2 < -1.0) t2 < -1.0;
    else t2 = t2;

    pitch = asin(t2) * 57.29577951323;
    t3 = 2.0 * (QW * QZ + QX * QY);
    t4 = 1.0 - 2.0 * (QY * QY + QZ *QZ);
    yaw = atan2(t3, t4) * 57.29577951323;


    ROS_INFO("roll : %f", roll);
    ROS_INFO("pitch : %f", pitch);
    ROS_INFO("yaw : %f", yaw);

    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}
