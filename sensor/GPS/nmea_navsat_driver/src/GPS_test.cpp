#include "ros/ros.h"                           
#include <sensor_msgs/NavSatFix.h>
#include "nmea_navsat_driver/nmea_msg.h"

int main(int argc, char **argv)                 
{
  ros::init(argc, argv, "GPS_Sensor");    
  ros::NodeHandle nh;                          

  ros::Publisher GPS_pub = nh.advertise<nmea_navsat_driver::nmea_msg>("Nmea_Publish",100);
  ros::Publisher ros_tutorial_pub = nh.advertise<sensor_msgs::NavSatFix>("fix", 100);

  // 루프 주기를 설정한다. "10" 이라는 것은 10Hz를 말하는 것으로 0.1초 간격으로 반복된다
  ros::Rate loop_rate(10);

  sensor_msgs::NavSatFix test;
  nmea_navsat_driver::nmea_msg test2;
  test2.data = "$GPRMC,92204.999,A,4250.5589,S,14718.5084,E,10,-62";


  int count = 0;                            
    test.header.seq = 1;
    test.header.stamp.sec = 22;
    test.status.status = 0;   // Valid
    test.status.service = 1;  // GPS
    test.latitude = 37.2323083333;
    test.longitude = 126.768930833;
    test.altitude = 0;
    test.header.frame_id = "/gps";
    test.position_covariance_type = 1;
  while (ros::ok())
  {
   
    test.latitude += 0.0000000001;
    ros_tutorial_pub.publish(test);          // 메시지를 발행한다. 약 0.1초 간격으로 발행된다
    GPS_pub.publish(test2);
    loop_rate.sleep();                      // 위에서 정한 루프 주기에 따라 슬립에 들어간다

    ++count;                                // count 변수 1씩 증가
  }

  return 0;
}
