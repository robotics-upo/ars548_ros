#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp> 
#include "ars548_messages/msg/object_list.hpp"
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using std::placeholders::_1;
#define MINIMUM_VELOCITY 2.0
#define DEFAULT_FRAME_ID "ARS_548" 
#define POINTCLOUD_HEIGHT 1
#define POINTCLOUD_WIDTH 1000
#define SIZE 1000

class ARS548FilterInterface : public rclcpp::Node 
{
protected:
  float min_velocity, min_velocity_sq;
  std::string frame_ID;
  sensor_msgs::msg::PointCloud2 filtered_cloud_msgObj;
  rclcpp::Subscription<ars548_messages::msg::ObjectList>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubObjFilter;
  sensor_msgs::PointCloud2Modifier modifierObject;
    
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

  virtual bool filterCondition(const ars548_messages::msg::Object &o) = 0;

  void topicCallback(const ars548_messages::msg::ObjectList::SharedPtr msg)
  {

    // RCLCPP_INFO(this->get_logger(),"ARS548FilterInterface Received ObjectList. Size: %d",
    //             (int)msg->objectlist_numofobjects);
    modifierObject.resize(msg->objectlist_numofobjects);
    fillCloudMessage(filtered_cloud_msgObj);
    sensor_msgs::PointCloud2Iterator<float> iter_x(filtered_cloud_msgObj,"x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(filtered_cloud_msgObj,"y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(filtered_cloud_msgObj,"z");
    sensor_msgs::PointCloud2Iterator<float> iter_vx(filtered_cloud_msgObj,"vx");
    sensor_msgs::PointCloud2Iterator<float> iter_vy(filtered_cloud_msgObj,"vy");
    int final_size=0;
    for(int i=0;i<msg->objectlist_numofobjects;i++)
      {

        if ( filterCondition(msg->objectlist_objects[i]))
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
            final_size++;
          }    
      }

    // RCLCPP_INFO(this->get_logger(),"ARS548FilterInterface Final . Filtered size: %d",
    //              final_size);
    modifierObject.resize(final_size);
    pubObjFilter->publish(filtered_cloud_msgObj);
  }

public:
    
  ARS548FilterInterface(const std::string &s): Node(s),  modifierObject(filtered_cloud_msgObj)
  {
    declare_parameter("frameID",DEFAULT_FRAME_ID);
    frame_ID=get_parameter("frameID").as_string();
    
    modifierObject.setPointCloud2Fields(5,
                                        "x",1,sensor_msgs::msg::PointField::FLOAT32,
                                        "y",1,sensor_msgs::msg::PointField::FLOAT32,
                                        "z",1,sensor_msgs::msg::PointField::FLOAT32,
                                        "vx",1,sensor_msgs::msg::PointField::FLOAT32,
                                        "vy",1,sensor_msgs::msg::PointField::FLOAT32
                                        );
    modifierObject.reserve(SIZE);
    pubObjFilter=create_publisher<sensor_msgs::msg::PointCloud2>("PointCloudObjectFiltered",10);
    subscription_=create_subscription<ars548_messages::msg::ObjectList>
      ("ObjectList",10,std::bind(&ARS548FilterInterface::topicCallback,this,_1));
  }
    
};

