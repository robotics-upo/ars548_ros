/**
 * @file ars548_driver_filter.cpp
 * @brief This is an example of usage of the driver. In this case we subscribe to the object message that the driver sends us and we filter the data by its velocity.
*/

#include <ros/ros.h>
#include <sensor_msgs/point_cloud2_iterator.h> 
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/tf.h>
#include "ars548_messages/ObjectList.h"
#include <cmath>

#define MINIMUM_VELOCITY 2
#define DEFAULT_FRAME_ID "ARS_548" 
#define POINTCLOUD_HEIGHT 1
#define SIZE 1000

class ARS548DriverFilter
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    float min_velocity;
    std::string frame_ID;
    
    sensor_msgs::PointCloud2 filtered_cloud_msgObj;
    geometry_msgs::PoseArray filtered_cloud_Direction;
    
    ros::Subscriber subscription_;
    ros::Publisher pubObjFilter;
    ros::Publisher pubDirFilter;
    
    sensor_msgs::PointCloud2Modifier modifierObject;

    void fillDirectionHeader(geometry_msgs::PoseArray &cloud_Direction){
        cloud_Direction.header.frame_id = this->frame_ID;
        cloud_Direction.header.stamp = ros::Time::now();
    }
    
    void fillDirectionMessage(geometry_msgs::PoseArray &cloud_Direction, const ars548_messages::ObjectList::ConstPtr& object_List, uint32_t i, int pos_index){
        tf::Quaternion q;
        float yaw;
        cloud_Direction.poses[pos_index].position.x = double(object_List->objectlist_objects[i].u_position_x);
        cloud_Direction.poses[pos_index].position.y = double(object_List->objectlist_objects[i].u_position_y);
        cloud_Direction.poses[pos_index].position.z = double(object_List->objectlist_objects[i].u_position_z);
        yaw = atan2(object_List->objectlist_objects[i].f_dynamics_relvel_y, object_List->objectlist_objects[i].f_dynamics_relvel_x);   
        q.setRPY(0,0,yaw);
        cloud_Direction.poses[pos_index].orientation.x = q.x();
        cloud_Direction.poses[pos_index].orientation.y = q.y();
        cloud_Direction.poses[pos_index].orientation.z = q.z();
        cloud_Direction.poses[pos_index].orientation.w = q.w();
    }
    
    void fillCloudMessage(sensor_msgs::PointCloud2 &cloud_msg){
        cloud_msg.header.frame_id = this->frame_ID;
        cloud_msg.header.stamp = ros::Time::now();
        cloud_msg.is_dense = false;
        cloud_msg.is_bigendian = false;
        cloud_msg.height = POINTCLOUD_HEIGHT;
    }

    void topic_callback(const ars548_messages::ObjectList::ConstPtr& msg)
    {
        modifierObject.resize(msg->objectlist_numofobjects);
        filtered_cloud_Direction.poses.resize(msg->objectlist_numofobjects);
        
        fillCloudMessage(filtered_cloud_msgObj);
        fillDirectionHeader(filtered_cloud_Direction);
        
        sensor_msgs::PointCloud2Iterator<float> iter_x(filtered_cloud_msgObj, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(filtered_cloud_msgObj, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(filtered_cloud_msgObj, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_vx(filtered_cloud_msgObj, "vx");
        sensor_msgs::PointCloud2Iterator<float> iter_vy(filtered_cloud_msgObj, "vy");
        
        int direction_operator = 0;
        
        for(uint32_t i = 0; i < msg->objectlist_numofobjects; i++)
        {
            float vx = msg->objectlist_objects[i].f_dynamics_absvel_x;
            float vy = msg->objectlist_objects[i].f_dynamics_absvel_y;
            float velocity = std::sqrt(vx*vx + vy*vy);
          
            if (velocity >= min_velocity)
            {
                *iter_x = msg->objectlist_objects[i].u_position_x;
                *iter_y = msg->objectlist_objects[i].u_position_y;
                *iter_z = msg->objectlist_objects[i].u_position_z;
                *iter_vx = msg->objectlist_objects[i].f_dynamics_absvel_x;
                *iter_vy = msg->objectlist_objects[i].f_dynamics_absvel_y;
                
                ++iter_vx;
                ++iter_vy;
                ++iter_x;
                ++iter_y;
                ++iter_z;
                
                fillDirectionMessage(filtered_cloud_Direction, msg, i, direction_operator);
                ++direction_operator;
            }    
        }
        
        modifierObject.resize(direction_operator);
        filtered_cloud_Direction.poses.resize(direction_operator);
        
        pubObjFilter.publish(filtered_cloud_msgObj);
        pubDirFilter.publish(filtered_cloud_Direction);
    }

public:
    ARS548DriverFilter() : private_nh_("~"), modifierObject(filtered_cloud_msgObj)
    {
        private_nh_.param<std::string>("frameID", frame_ID, DEFAULT_FRAME_ID);
        
        double min_v;
        private_nh_.param("minimum", min_v, (double)MINIMUM_VELOCITY);
        min_velocity = (float)min_v;
        
        modifierObject.setPointCloud2Fields(5,
            "x", 1, sensor_msgs::PointField::FLOAT32,
            "y", 1, sensor_msgs::PointField::FLOAT32,
            "z", 1, sensor_msgs::PointField::FLOAT32,
            "vx", 1, sensor_msgs::PointField::FLOAT32,
            "vy", 1, sensor_msgs::PointField::FLOAT32
        );
        modifierObject.reserve(SIZE);
        filtered_cloud_Direction.poses.reserve(SIZE);
        
        pubObjFilter = private_nh_.advertise<sensor_msgs::PointCloud2>("PointCloudObjectFiltered", 10);
        pubDirFilter = private_nh_.advertise<geometry_msgs::PoseArray>("DirectionFiltered", 10);    
        
        // El driver publica ObjectList en su propio namespace (/radar/ObjectList)
        // en vez de hardcodear la ruta, usamos nh_ estandar y remapeamos en el launch file.
        subscription_ = nh_.subscribe("ObjectList", 10, &ARS548DriverFilter::topic_callback, this);
        
        ROS_INFO("ARS548 filter initialized: Minimum velocity: %f, Frame ID: %s", min_velocity, frame_ID.c_str());
    }
};

int main(int argc, char* argv[]){
    ros::init(argc, argv, "ars548_driver_filter");
    ARS548DriverFilter filter;
    ros::spin();
    return 0;
}