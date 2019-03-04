// Header for Ros
#include"ros/ros.h"
#include"nmea_navsat_driver/nmea_msg.h"
#include"nmea_navsat_driver/MyMsg.h"
// Use String 함수
#include<string>
#include<sstream>


// Subscribe 후 Publish하기 위한 callback 함수
void msgCallback(const nmea_navsat_driver::nmea_msg::ConstPtr& msg2){
    // GPS Node로부터 Subscribe
    std::string raw = msg2->data;
    // Publish하기 위한 변수선언
    ros::NodeHandle nh;
    ros::Publisher GPS_publisher = nh.advertise<nmea_navsat_driver::MyMsg>("raw/GPS",100);
    nmea_navsat_driver::MyMsg msg;
    // Data Pharse 변수선언
    std::stringstream ss(raw);
    std::string data;
    // 속도 변환하기 위한 변수선언
    double speed;
    double course;

    std::getline(ss,data,','); // first
        // testing Code : ROS_INFO("%s",data.c_str());
    if(data.compare("$GPRMC")){ // GPRMC Data Format이 아니면 종료
    //  ROS_INFO("WRONG DATA FORMAT");
        return;
        }
    std::getline(ss,data,','); // second
    
    std::getline(ss,data,','); // third
    if(data.compare("A")){
        return;
    }
    
    std::getline(ss,data,','); // fourth
    
    std::getline(ss,data,','); // fivth
    
    std::getline(ss,data,','); // sixth
 
    std::getline(ss,data,','); // seventh
    
    std::getline(ss,data,','); // eigth
    speed = atof(data.c_str());
    speed *= 1.852;
    msg.Speed = speed;
    
    std::getline(ss,data,','); // ninth
    course = atof(data.c_str());
    msg.Course = course;
    


    GPS_publisher.publish(msg); // parse            
    return;
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "GPS_subscriber");
    ros::NodeHandle nh;
    ros::Subscriber nmea_subscriber = nh.subscribe("Nmea_Publish",100,msgCallback);
    ros::Publisher GPS_publisher = nh.advertise<nmea_navsat_driver::MyMsg>("raw/GPS",100);

    ros::spin();
         
    return 0;
}