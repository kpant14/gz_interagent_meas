/* Standard/Math library */
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <eigen3/Eigen/Eigen>
#include <cmath>

/* Gazebo library */
#include <gz/math.hh>
#include <gz/msgs.hh>
#include <gz/transport.hh>
#include <gz/msgs/pose_v.pb.h>

/* ROS2 library */
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"


using namespace std::chrono_literals;

/* Create a node that publishes Gazebo true position to the offboard detector */
class InterAgentRangePublisher : public rclcpp::Node
{
  public:
    InterAgentRangePublisher()
    : Node("interagent_meas_publisher")
    {
      this->declare_parameter("px4_ns", "px4");
      this->declare_parameter("cf_ns", "cf");
      this->declare_parameter("gz_world_name", "AbuDhabi");
      this->declare_parameter("gz_model_names", std::vector<std::string>{"x500_1"});
      this->declare_parameter("ros_ns", std::vector<std::string>{"px4_1"});

      std::string _px4_ns = this->get_parameter("px4_ns").as_string();
      std::string _world_name = this->get_parameter("gz_world_name").as_string();
      std::vector<std::string> _model_names = this->get_parameter("gz_model_names").as_string_array();
      std::vector<std::string> _ros_ns = this->get_parameter("ros_ns").as_string_array();

      RCLCPP_INFO(this->get_logger(),"World Name: %s", _world_name.c_str());
      for (u_int16_t i=0; i < _model_names.size(); i++)
      {
        RCLCPP_INFO(this->get_logger(),"Model Name: %s \n", _model_names[i].c_str());
        RCLCPP_INFO(this->get_logger(),"ROS Namespace: %s \n", _ros_ns[i].c_str());
      }
      std::string _world_pose_topic = "/world/" + _world_name + "/pose/info";
      if (!_node.Subscribe(_world_pose_topic, &InterAgentRangePublisher::poseInfoCallback, this)) {
            RCLCPP_ERROR(this->get_logger(),"failed to subscribe to %s", _world_pose_topic.c_str());
      }
      timer_ = this->create_wall_timer(
      300ms, std::bind(&InterAgentRangePublisher::timer_callback, this));
    }
  private:
    void poseInfoCallback(const gz::msgs::Pose_V &_pose)
    {   
      _pose_msg = _pose;
    }
    void timer_callback()
    {
        std::vector<std::string>  _model_names = this->get_parameter("gz_model_names").as_string_array();
        std::vector<std::string>  _ros_ns = this->get_parameter("ros_ns").as_string_array();

        size_t _num_models = _model_names.size();
        // Getting the position of each model entity 
        std::vector<gz::math::Vector3d> _model_positions_enu;
        for (u_int16_t i=0; i < _model_names.size(); i++)
        {
          gz::math::Vector3d _position_enu;
          for (int p = 0; p < _pose_msg.pose_size(); p++) {
            if (_pose_msg.pose(p).name() == _model_names[i]) {
              _position_enu.X(_pose_msg.pose(p).position().x());
              _position_enu.Y(_pose_msg.pose(p).position().y());
              _position_enu.Z(_pose_msg.pose(p).position().z());
              _model_positions_enu.push_back(_position_enu);
              break;
            }
          }
        }
       // Calculating distance from position of each model entity 
        for (u_int16_t i=0; i < _model_names.size(); i++)
        {
          if (_pub_dist_map.find(_model_names[i]) == _pub_dist_map.end())
          {
            _pub_dist_map[_model_names[i]] = this->create_publisher<std_msgs::msg::Float32MultiArray>(std::string{_ros_ns[i]}+"/detector/interagent_distances", 10);
          }
          if (_model_positions_enu.size()==_num_models)
          {
            std_msgs::msg::Float32MultiArray distances;
            for (u_int16_t j=0; j < _model_names.size(); j++)
            {
              double distance = _model_positions_enu[i].Distance(_model_positions_enu[j]);
              distances.data.push_back(distance);
            }
            _pub_dist_map[_model_names[i]]->publish(distances);
          }
        }
    }
    gz::msgs::Pose_V _pose_msg;
    gz::transport::Node _node;
    typedef rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr _dist_pub_ptr;
    rclcpp::TimerBase::SharedPtr timer_;
    std::map<std::string, _dist_pub_ptr> _pub_dist_map{};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InterAgentRangePublisher>());
  rclcpp::shutdown();
  return 0;
}
