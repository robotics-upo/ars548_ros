/**
 * @file ars548_driver_filter.cpp
 * @brief This is an example of usage of the driver. In this case we subscribe to the object message that the driver sends us and we filter the data by its velocity.
*/

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp> 
#include "ars548_messages/msg/object_list.hpp"
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using std::placeholders::_1;
#define MINIMUM_VELOCITY 2
#define DEFAULT_FRAME_ID "ARS_548" 
#define POINTCLOUD_HEIGHT 1
#define POINTCLOUD_WIDTH 1000
#define SIZE 1000

class ars548_driver_filter : public rclcpp::Node
{
   
    private:
    float min_velocity;
    std::string frame_ID;
    sensor_msgs::msg::PointCloud2 filtered_cloud_msgObj;
    geometry_msgs::msg::PoseArray filtered_cloud_Direction;
    rclcpp::Subscription<ars548_messages::msg::ObjectList>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubObjFilter;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pubDirFilter;
    sensor_msgs::PointCloud2Modifier modifierObject;
    
    void fillDirectionHeader(geometry_msgs::msg::PoseArray &cloud_Direction){
        cloud_Direction.header = std_msgs::msg::Header();
        cloud_Direction.header.frame_id=this->frame_ID;
        std::chrono::time_point<std::chrono::system_clock> now=std::chrono::system_clock::now();
        auto duration=now.time_since_epoch();
        cloud_Direction.header.stamp.nanosec=std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
        cloud_Direction.header.stamp.sec=std::chrono::duration_cast<std::chrono::seconds>(duration).count();
    }
    void fillDirectionMessage(geometry_msgs::msg::PoseArray &cloud_Direction,ars548_messages::msg::ObjectList::SharedPtr object_List,u_int32_t i){
        tf2::Quaternion q;
        float yaw;
        cloud_Direction.poses[i].position.x = double(object_List->objectlist_objects[i].u_position_x);
        cloud_Direction.poses[i].position.y = double(object_List->objectlist_objects[i].u_position_y);
        cloud_Direction.poses[i].position.z = double(object_List->objectlist_objects[i].u_position_z);
        yaw = atan2(object_List->objectlist_objects[i].f_dynamics_relvel_y,object_List->objectlist_objects[i].f_dynamics_relvel_x);   
        q.setRPY(0,0,yaw);
        cloud_Direction.poses[i].orientation.x=q.x();
        cloud_Direction.poses[i].orientation.y=q.y();
        cloud_Direction.poses[i].orientation.z=q.z();
        cloud_Direction.poses[i].orientation.w=q.w();
    }
    
    void fillCloudMessage(sensor_msgs::msg::PointCloud2 &cloud_msg){
        cloud_msg.header=std_msgs::msg::Header();
        cloud_msg.header.frame_id=this->frame_ID;
        std::chrono::time_point<std::chrono::system_clock> now=std::chrono::system_clock::now();
        auto duration=now.time_since_epoch();
        cloud_msg.header.stamp.nanosec=std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
        cloud_msg.header.stamp.sec=std::chrono::duration_cast<std::chrono::seconds>(duration).count();
        cloud_msg.is_dense=false;
        cloud_msg.is_bigendian=false;
        cloud_msg.height=POINTCLOUD_HEIGHT;
    }

    void topic_callback(const ars548_messages::msg::ObjectList::SharedPtr msg)
    {
        modifierObject.resize(msg->objectlist_numofobjects);
        filtered_cloud_Direction.poses.resize(msg->objectlist_numofobjects);
        fillCloudMessage(filtered_cloud_msgObj);
        fillDirectionHeader(filtered_cloud_Direction);
        sensor_msgs::PointCloud2Iterator<float> iter_x(filtered_cloud_msgObj,"x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(filtered_cloud_msgObj,"y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(filtered_cloud_msgObj,"z");
        sensor_msgs::PointCloud2Iterator<float> iter_vx(filtered_cloud_msgObj,"vx");
        sensor_msgs::PointCloud2Iterator<float> iter_vy(filtered_cloud_msgObj,"vy");
        int direction_operator=0;
        for(int i=0;i<msg->objectlist_numofobjects;i++)
        {
            
            float vx,vy;
            vx=msg->objectlist_objects[i].f_dynamics_absvel_x;
            vy=msg->objectlist_objects[i].f_dynamics_absvel_y;
            float velocity= std::sqrt(vx*vx+vy*vy);
          
            if (velocity>=min_velocity)
            {
                
                *iter_x=msg->objectlist_objects[i].u_position_x;
                *iter_y=msg->objectlist_objects[i].u_position_y;
                *iter_z=msg->objectlist_objects[i].u_position_z;
                *iter_vx=msg->objectlist_objects[i].f_dynamics_absvel_x;
                *iter_vy=msg->objectlist_objects[i].f_dynamics_absvel_y;
                ++iter_vx;
                ++iter_vy;
                ++iter_x;
                ++iter_y;
                ++iter_z;
                fillDirectionMessage(filtered_cloud_Direction,msg,direction_operator);
                ++direction_operator;
            }    
        }
        modifierObject.resize(direction_operator);
        filtered_cloud_Direction.poses.resize(direction_operator);
        pubObjFilter->publish(filtered_cloud_msgObj);
        pubDirFilter->publish(filtered_cloud_Direction);
        
    }

    public:
    
    ars548_driver_filter(): Node("ars548_driver_filter"),  modifierObject(filtered_cloud_msgObj)
    {
        this->declare_parameter("frameID",DEFAULT_FRAME_ID);
        this->frame_ID=this->get_parameter("frameID").as_string();
        this->declare_parameter("minimum",MINIMUM_VELOCITY);
        this->min_velocity=this->get_parameter("minimum").as_int();
        modifierObject.setPointCloud2Fields(5,
            "x",1,sensor_msgs::msg::PointField::FLOAT32,
            "y",1,sensor_msgs::msg::PointField::FLOAT32,
            "z",1,sensor_msgs::msg::PointField::FLOAT32,
            "vx",1,sensor_msgs::msg::PointField::FLOAT32,
            "vy",1,sensor_msgs::msg::PointField::FLOAT32
        );
        modifierObject.reserve(SIZE);
        filtered_cloud_Direction.poses.reserve(SIZE);
        pubObjFilter=this->create_publisher<sensor_msgs::msg::PointCloud2>("PointCloudObjectFiltered",10);
        pubDirFilter=this->create_publisher<geometry_msgs::msg::PoseArray>("DirectionFiltered",10);    
        subscription_=this->create_subscription<ars548_messages::msg::ObjectList>("ObjectList",10,std::bind(&ars548_driver_filter::topic_callback,this,_1));
        RCLCPP_INFO(this->get_logger(),"ARS548 filter initialized: Minimum velocity: %f, Frame ID: %s",min_velocity, frame_ID.c_str());
    }
    
};

int main(int argc,char* argv[]){
   rclcpp::init(argc,argv);
   rclcpp::spin(std::make_shared<ars548_driver_filter>());
   rclcpp::shutdown();
   return 0;
}