/*
 * obstacle_flag.cpp
 *
 *  Created on: 2018-05-03
 *  Author: HwiJin Hong
 *
*/
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <obstacle_detector/Obstacles.h>
#include "obstacle_flag/data.h"
#include <cmath>
#include <thread>
#include <vector>
#include <algorithm>


/* U_turn check */
#define Laser_Filter_start 195
#define Laser_Filter_end 330
#define frequency 30

#define test_uturn
#define test_dynamic
#define test_park

static double inf = std::numeric_limits<double>::infinity();
static bool dynamic_obstacle, u_turn;
static bool is_park;

static active active;
static dynamic dynamic_param;
static priv priv;
static u_turn_param u_turn_param;
static park park;

using namespace obstacle_detector;

void obstaclecheck(const obstacle_detector::Obstacles::ConstPtr &object)
{
    ros::NodeHandle Node;
    Node.getParam("obstacle_flag/active/dynamic",active.active_dynamic);
    if(!active.active_dynamic)
        return;

    if(object->segments.empty())
        return;
    
    double delta_y;
    bool dynamic_obstacle = false;

    /* find arry num */
    for(int i=0; i<object->segments.size(); i++)
    {

        if( (fabs(object->segments.at(i).first_point.y - object->segments.at(i).last_point.y) > dynamic_param.min_y_distance)  && 
            (fabs(object->segments.at(i).first_point.y - object->segments.at(i).last_point.y) < dynamic_param.max_y_distance) &&
            (fabs(object->segments.at(i).first_point.x - object->segments.at(i).last_point.x) < dynamic_param.max_x_width) && (fabs(object->segments.at(i).first_point.x) > dynamic_param.min_x_obstacle) 
            && (fabs(object->segments.at(i).last_point.x) > dynamic_param.min_x_obstacle) )
        {
            priv.priv_data_first_y.push_back(object->segments.at(i).first_point.y);
        }
    }
    /* 이전 data와 비교해서 delta_y 추출 */
    if( priv.priv_data_first_y.size() == dynamic_param.sample_size )
    {

        delta_y =  fabs( priv.priv_data_first_y.front() - priv.priv_data_first_y.back());
        priv.delta_y.push_back(delta_y);
           
        priv.priv_data_first_y.clear();
        priv.priv_data_first_y.resize(0);
    }
    /* delta y에 대한 샘플링 시작 */
    if(priv.delta_y.size() == dynamic_param.size_N) 
    {
        for(std::vector<double>::size_type i = 0; i < priv.delta_y.size(); i++)
        {
            ROS_INFO("delta[%ld] y : %lf",i,priv.delta_y.at(i));
            if(priv.delta_y[i] > dynamic_param.min_delta_y && priv.delta_y[i] < dynamic_param.max_delta_y)
                dynamic_obstacle = true;
            else
            {
                dynamic_obstacle = false;
                break;
            }
        }
        priv.delta_y.clear();
        priv.delta_y.resize(0);
    }


    Node.setParam("hl_controller/movingobj",dynamic_obstacle);

    #ifdef test_dynamic
        if(dynamic_obstacle)
          {  ROS_INFO("detected_dynamic");
          }
    #endif


}

bool checkUTurn(std::vector<float>ranges, int number)
{
    int offset;
    int min_number = number - u_turn_param.check_UTurn;
   /*  detect size 크기 조절 */
    if( /*ranges[number+max_range] == inf || ranges[number-max_range] == inf &&*/
        (ranges[min_number] == inf) )
        {}  
    else
        return false; 
    /* detection */
    for(offset=0; offset<=u_turn_param.check_UTurn; offset++)
    {
        if( (ranges[number+offset] != inf) && 
            (fabs(ranges[number]-ranges[number+offset]) < u_turn_param.UTurn_distance_point) )
                continue;
            
        else
            return false;
    }
    /* 이전 상태와 range값 비교해서 최대한 붙어 있게 설정 */
    if(u_turn_param.before_detect == 0)
        u_turn_param.before_detect = number;
    if((fabs(ranges[number]-ranges[u_turn_param.before_detect]) > 0.1))
        return false;
    u_turn_param.before_detect = number;
    return true;
}

void uturncall(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    ros::NodeHandle Node;
    Node.getParam("obstacle_flag/active/u_turn",active.active_uturn);
    if(!active.active_uturn)
        return;
    int count=0;
    for(int i=Laser_Filter_start; i<=Laser_Filter_end;i++)
    {
        if(checkUTurn(scan->ranges,i))
        {      
            u_turn = true;   
            count++;
            #ifdef test_uturn
            if(u_turn)
            {
                count++;
                ROS_INFO("range[%d] : %lf",i, scan->ranges[i]);
                ROS_INFO("Detected u_turn");
            }   
            #endif
        }
        else
        {
            u_turn = false;
        }
    }
    if(count>=u_turn_param.number_of_uturn)
        u_turn = true;
    else
        u_turn = false;

    #ifdef test_uturn
        ROS_INFO("count : %d",count);
    #endif

    Node.setParam("hl_controller/uturn",u_turn);
}

struct Box{
    struct Pos{
        Pos() : x(0.0), y(0.0) {}
        Pos(double x, double y): x(x), y(y) {}
        double x, y;
    };
    Box(const std::vector<double>& pos){
        if (pos.size() != 8) { //assert
            ROS_ERROR("Box() failed because of the lack pos information : %ld", pos.size());
            exit(-1);
        }

        for (int i = 0 ; i < 8 ; ++i){ //save
            if (i%2) posAry[(i/2)].y = pos[i];
            else posAry[(i/2)].x = pos[i];
        }

        std::sort(posAry, posAry+4, [](const Pos& f, const Pos& s){
            return f.x < s.x;
        }); //sort by x
        
        //sort by y for [0] [1], [2] [3]. 
        //then the result in posAry would be top-left, bottom-left, bottom-right, bottom-up
        if (posAry[0].y < posAry[1].y) std::swap(posAry[0], posAry[1]);
        if (posAry[2].y > posAry[3].y) std::swap(posAry[2], posAry[3]);

        for (int i = 0 ; i < 4; ++i)
            ROS_INFO("result of square : %lf, %lf", posAry[i].x, posAry[i].y);
    }
    
    bool in(double x, double y){
        //we can divide square to the triangle of two.
        //then, when the pos is in the triangles, we can say that the pos is in the triangle.
        //the condition of the point and triangle is based on the area of triangle
        static auto calc_triangle_area = [](Pos f, Pos s, Pos t)->double { //first, second, third
            double area = fabs((f.x*(s.y-t.y) + s.x*(t.y-f.y) + t.x*(f.y-s.y))/2);
            return area;
        };
        static double EPSILON = 0.00001;

        Pos p(x,y);
        //0,1,2 2,3,0
        double area_with_point1 = 
            calc_triangle_area(p, posAry[0], posAry[1]) +
            calc_triangle_area(p, posAry[1], posAry[2]) +
            calc_triangle_area(p, posAry[2], posAry[0]);
        double area_without_point1 = 
            calc_triangle_area(posAry[0], posAry[1], posAry[2]);
        if (fabs(area_with_point1 - area_without_point1) <= EPSILON) return true; //the point is in first triangle

        double area_with_point2 = 
            calc_triangle_area(p, posAry[2], posAry[3]) +
            calc_triangle_area(p, posAry[3], posAry[0]) +
            calc_triangle_area(p, posAry[0], posAry[2]);
        double area_without_point2 = 
            calc_triangle_area(posAry[2], posAry[3], posAry[0]);
        if (fabs(area_with_point2 - area_without_point2) <= EPSILON) return true; //the point is in first triangle
        return false;
    }
public:
    Pos posAry[4];
};

Box *boxPary[2]={nullptr, nullptr};
int min_sample_cnt;
#define NEAR_BOX_IDX 0
#define FAR_BOX_IDX 1

void parkcall(const obstacle_detector::Obstacles::ConstPtr &object)
{
    ros::NodeHandle Node;
    Node.getParam("obstacle_flag/active/park",active.active_park);

    if(!active.active_park)
        return;

    if(object->segments.empty())
        return;
    
    size_t in_nearbox_cnt = 0, in_farbox_cnt = 0;

    //check for lines
    for (auto& line : object->segments){
        //is the point is in nearbox?
        if (boxPary[NEAR_BOX_IDX]->in(line.first_point.x, line.first_point.y) ||
                boxPary[NEAR_BOX_IDX]->in(line.last_point.x, line.last_point.y)) 
            in_nearbox_cnt++;
        else if (boxPary[FAR_BOX_IDX]->in(line.first_point.x, line.first_point.y) ||
                boxPary[FAR_BOX_IDX]->in(line.last_point.x, line.last_point.y)) 
            in_farbox_cnt++;
    }

    //check for circles
    for (auto& circle : object->circles){
        if (boxPary[NEAR_BOX_IDX]->in(circle.center.x, circle.center.y)) 
            in_nearbox_cnt++;
        else if (boxPary[FAR_BOX_IDX]->in(circle.center.x, circle.center.y))  
            in_farbox_cnt++;
    }

    bool near = false, far = false;
    if (in_nearbox_cnt >= min_sample_cnt) {near = true; ROS_INFO("near parking point detected! %ld", in_nearbox_cnt);} 
    if (in_farbox_cnt >= min_sample_cnt) {far = true; ROS_INFO("far parking point detected! %ld", in_farbox_cnt);}
    Node.setParam("hl_controller/parking_near", near);
    Node.setParam("hl_controller/parking_far", far);
    if ((!near) && (!far)) ROS_INFO("no parking point...%ld %ld", in_nearbox_cnt, in_farbox_cnt);
}

void subscribepark()
{
    ros::NodeHandle Node;
    ros::NodeHandle n("~");
    std::vector<double> near_pos, far_pos;

    if(!n.getParam("park/min_sample_cnt", min_sample_cnt)){
        ROS_ERROR("get min_sample_cnt failed... exit the program");
        exit(-1);
    }
    if (!n.getParam("park/near_box", near_pos)) {
        ROS_ERROR("get near box coordinate failed... exit the program");
        exit(-1);
    }
    if (!n.getParam("park/far_box", far_pos)) {
        ROS_ERROR("get far box coordinate failed... exit the program");
        exit(-1);
    }
    boxPary[NEAR_BOX_IDX] = new Box(near_pos); boxPary[FAR_BOX_IDX] = new Box(far_pos);   
    ros::Subscriber scan_sub = Node.subscribe("park_obstacle",100,parkcall);
    ros::Rate loop_rate(frequency);

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void subscribeuturn()
{
    ros::NodeHandle Node;
    ros::NodeHandle n("~");

    n.getParam("u_turn/UTurn_distance_point",u_turn_param.UTurn_distance_point);
    n.getParam("u_turn/check_UTurn",u_turn_param.check_UTurn);
    n.getParam("u_turn/number_of_uturn",u_turn_param.number_of_uturn);
    ros::Subscriber scan_sub = Node.subscribe("uturn_scan",100,uturncall);
    ros::Rate loop_rate(frequency);

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "obstacle_flag");
    ros::NodeHandle Node;
    ros::NodeHandle n("~");

    n.param<bool>("hl_controller/movingobj",dynamic_obstacle,false);
    n.param<bool>("hl_controller/uturn",u_turn,false);
    n.param<bool>("h1_controoller/parking",is_park,false);


    n.getParam("dynamic/size_N",dynamic_param.size_N);
    n.getParam("dynamic/min_y_distance",dynamic_param.min_y_distance);
    n.getParam("dynamic/max_y_distance",dynamic_param.max_y_distance);
    n.getParam("dynamic/min_x_obstacle",dynamic_param.min_x_obstacle);
    n.getParam("dynamic/max_x_width",dynamic_param.max_x_width);
    n.getParam("dynamic/min_delta_y",dynamic_param.min_delta_y);
    n.getParam("dynamic/max_delta_y",dynamic_param.max_delta_y);
    n.getParam("dynamic/sample_size",dynamic_param.sample_size);

    
    ros::Subscriber sub = Node.subscribe("dynamic_obstacle",100,obstaclecheck);

    std::thread dynamicnode(subscribeuturn);
    dynamicnode.detach();

    std::thread parking(subscribepark);
    parking.detach();

    ros::Rate loop_rate(frequency);

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}