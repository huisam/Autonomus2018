/*
 * Translates sensor_msgs/NavSat{Fix,Status} into nav_msgs/Odometry using UTM
 */

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <gps_common/conversions.h>
#include <nav_msgs/Odometry.h>
#include "nmea_navsat_driver/MyMsg.h"
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include "gps_common/Initial_UTM.h"

using namespace gps_common;

static double base_longitude = 126.7689;
static double base_latitude = 37.2323;
static ros::Publisher odom_pub;
std::string frame_id, child_frame_id;
static double rot_cov;
static bool cov_flag;
static const double cov_max = 99999;
//
static double easting_shift, northing_shift;
//
static double x,y,z,w;
static bool parameter;

static class Initial Initpose;

void callback2(const nmea_navsat_driver::MyMsgPtr& GPSmsg)
{
  ros::NodeHandle node;
  geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(GPSmsg->Course);

  x = q.x;
  y = q.y;
  z = q.z;
  w = q.w;
  
}
void callback(const sensor_msgs::NavSatFixConstPtr& fix) {
  ros::NodeHandle node;
  if (fix->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
    ROS_INFO("No fix.");
    return;
  }

  if (fix->header.stamp == ros::Time(0)) {
    return;
  }

  double northing, easting;
  std::string zone;

  LLtoUTM(fix->latitude, fix->longitude, northing, easting, zone);

  if (odom_pub) {
    nav_msgs::Odometry odom;
    odom.header.stamp = fix->header.stamp;

    if (frame_id.empty())
      odom.header.frame_id = fix->header.frame_id;
    else    
      odom.header.frame_id = frame_id;

  
    if(!Initpose.testExist())
    {
      Initpose.input(easting, northing);
    }
    else
    {
     node.getParam("GPS_Odometry_initial",parameter);
     Initpose.paraminput(parameter);
     node.setParam("GPS_Odometry_initial",false);
    }

    odom.child_frame_id = child_frame_id;
 //   node.getParam("/gps/easting_shift", easting_shift);   // easting_shift 됨값 세팅해주면 됨
 //   node.getParam("/gps/northing_shift", northing_shift); // northing_shift 값 세팅해주면 됨
    odom.pose.pose.position.x = easting - Initpose.getX();
    odom.pose.pose.position.y = northing - Initpose.getY();
    odom.pose.pose.position.z = 0;
    
    ros::Subscriber GPS_sub = node.subscribe("raw/GPS",10,callback2);
    ros::spinOnce();

    odom.pose.pose.orientation.x = x;
    odom.pose.pose.orientation.y = y;
    odom.pose.pose.orientation.z = z;
    odom.pose.pose.orientation.w = w;
    
    // Use ENU covariance to build XYZRPY covariance
    boost::array<double, 36> covariance = {{
      fix->position_covariance[0], 0, 0, 0, 0, 0,
      0, fix->position_covariance[4], 0, 0, 0, 0,
      0, 0, fix->position_covariance[8], 0, 0, 0,
      0, 0, 0, rot_cov, 0, 0,
      0, 0, 0, 0, rot_cov, 0,
      0, 0, 0, 0, 0, rot_cov
    }};
     odom.pose.covariance = covariance;

    //boost::array<double, 36> covariance = fix->position_covariance;
//    
    node.getParam("GPS/cov/flag",cov_flag);
//
    /*임의로 covariance 세팅해서 사용하면 됨 */
    if(cov_flag)
    {
      ROS_INFO("hihihihihihi");
      boost::array<double, 36> covariance2 = {{
      cov_max, 0, 0, 0, 0, 0,
      0, cov_max, 0, 0, 0, 0,
      0, 0, cov_max, 0, 0, 0,
      0, 0, 0, cov_max, 0, 0,
      0, 0, 0, 0, cov_max, 0,
      0, 0, 0, 0, 0, cov_max
    }};
    odom.pose.covariance = covariance2;
    }
 //

  

  
        


    odom_pub.publish(odom);
  }
}


int main (int argc, char **argv) {
  ros::init(argc, argv, "utm_odometry_node");
  ros::NodeHandle priv_node("~");
  ros::NodeHandle node;
  priv_node.param<std::string>("frame_id", frame_id, "odom_combined");
  priv_node.param<std::string>("child_frame_id", child_frame_id, "base_footprint");
  priv_node.param<double>("rot_covariance", rot_cov, 99999.0);
  priv_node.param<bool>("GPS/cov/flag",cov_flag,false);
  odom_pub = node.advertise<nav_msgs::Odometry>("odom/GPS", 10);

  ros::Subscriber fix_sub = node.subscribe("fix", 10, callback);
  ros::Subscriber GPS_sub = node.subscribe("raw/GPS",10,callback2);
  ros::spin();
}

