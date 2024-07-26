#include <ars548_driver/ars548_filter_interface.hpp>


class ARS548FilterExample : public ARS548FilterInterface
{
public:
  ARS548FilterExample():ARS548FilterInterface("ars548_filter_example")
    {
        declare_parameter("minimum",MINIMUM_VELOCITY);
        min_velocity=get_parameter("minimum").as_double();
        min_velocity_sq = min_velocity * min_velocity;

        RCLCPP_INFO(this->get_logger(),"ARS548FilterExample initialized: Minimum velocity: %f, Frame ID: %s",min_velocity, frame_ID.c_str());
    }


  virtual bool filterCondition (const ars548_messages::msg::Object &o) {
    float vx = o.f_dynamics_absvel_x;
    float vy = o.f_dynamics_absvel_y;

    RCLCPP_INFO(this->get_logger(),"ARS548FilterExample condition: v_sq = %f, min_vel_sq = %f",
                vx*vx + vy*vy, min_velocity_sq);

    return (vx*vx + vy*vy) > min_velocity_sq;
  }

};
