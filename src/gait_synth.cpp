/* gait_synth.cpp
 * Author: Brett Downing <brettrd@brettrd.com>
 *
 * a synth for end-effectctor points
 * Later: request kinematic plausibility of poses
 * Later: request moment arms on servo shafts / force capability
 * composable nodes enabled
 */

#include <map>
#include <memory>
#include <string>
#include <math.h>

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Geometry> 
#include <Eigen/Dense>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

//#include <geometry_msgs/msg/pose.hpp>
//#include <geometry_msgs/msg/point.hpp>
//#include <geometry_msgs/msg/quaternion.hpp>


class gait_synth : public rclcpp::Node
{
  public:
    gait_synth(rclcpp::NodeOptions options);

  private:
    const int n_channels = 16;
    void onCommand(const geometry_msgs::msg::Twist::SharedPtr msg);
    rcl_interfaces::msg::SetParametersResult reconf_callback(const std::vector<rclcpp::Parameter> & parameters);

    OnSetParametersCallbackHandle::SharedPtr param_cb;

    int64_t n_points;
    double contact_radius;
    double contact_height;
    double contact_theta_offset;
    double step_size;
    double step_height;
    std::vector<double> step_phase_offsets;
    std::vector<double> step_duties;
    std::map<std::string, double> body_position;
    std::map<std::string, double> body_orientation;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub;

    double phase;

    Eigen::MatrixXd target_points;
    Eigen::MatrixXd contact_points;
    std::vector<bool> foot_last_swing;

};



gait_synth::gait_synth(rclcpp::NodeOptions options) : 
  Node("gait_synth", options),
  n_points(5),
  contact_radius(0.055),
  contact_height(0.090),
  contact_theta_offset(-1.6),
  step_size(0.015),
  step_height(-0.020),
  step_phase_offsets({0.0, 0.2, 0.4, 0.6, 0.8}),
  step_duties({0.8, 0.8, 0.8, 0.8, 0.8}),
  body_position({
    {"x", 0},
    {"y", 0},
    {"z", 0},
  }),
  body_orientation({
    {"roll", 0},
    {"pitch", 0},
    {"yaw", 0},
  }),
  phase(0)
{

  RCLCPP_INFO(this->get_logger(),"initializing");

  // initialise the parameters
  this->declare_parameter("n_points", n_points/*, "number of contact points to synthesize"*/);
  this->declare_parameter("contact_radius", contact_radius/*, "distance between contact point and contact centre (metres)"*/);
  this->declare_parameter("contact_height", contact_height/*, "distance between contact centre and the origin (metres)"*/);
  this->declare_parameter("contact_theta_offset", contact_theta_offset/*, "z angle of first contact point (radians)"*/);
  this->declare_parameter("step_size", step_size/*, "approx step size (metres)"*/);
  this->declare_parameter("step_height", step_height/*, "approx step height (metres)"*/);
  this->declare_parameter("step_phase_offsets", step_phase_offsets/*, "phase time raised (0-1)"*/);
  this->declare_parameter("step_duties", step_duties/*, "portion of phase planted (0-1)"*/);
  this->declare_parameters("body_position", body_position/*, "position of the body relative to the origin"*/);
  this->declare_parameters("body_orientation", body_orientation/*, "orientation of the body relative to the origin"*/);

  this->get_parameter("n_points", n_points);
  this->get_parameter("contact_radius", contact_radius);
  this->get_parameter("contact_height", contact_height);
  this->get_parameter("contact_theta_offset", contact_theta_offset);
  this->get_parameter("step_size", step_size);
  this->get_parameter("step_height", step_height);
  this->get_parameter("step_phase_offsets", step_phase_offsets);
  this->get_parameter("step_duties", step_duties);
  this->get_parameters("body_position", body_position);
  this->get_parameters("body_orientation", body_orientation);

  param_cb = this->add_on_set_parameters_callback(
    std::bind(&gait_synth::reconf_callback, this, std::placeholders::_1));


  // initialise the stance

  // convert the body pose into an Eigen format
  Eigen::Vector3d contact_centre(-body_position.at("x"), -body_position.at("y"), -body_position.at("z"));
  Eigen::Matrix3d contact_rotation = (
      Eigen::AngleAxisd(body_orientation.at("roll"), Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(body_orientation.at("pitch"), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(body_orientation.at("yaw"), Eigen::Vector3d::UnitZ())
    ).inverse().toRotationMatrix();

  contact_points = Eigen::MatrixXd(3,n_points);
  foot_last_swing.resize(n_points, false);
  for(unsigned int n=0; n<n_points; n++)
  {
    Eigen::Vector3d neutral_point =
      contact_centre + (
        contact_rotation *
        Eigen::AngleAxisd( contact_theta_offset + (2*M_PI*n/(double)n_points), Eigen::Vector3d::UnitZ()) *
        Eigen::Vector3d(contact_radius,0,contact_height)
      );
      
    contact_points.block(0,n,3,1) = neutral_point;
    foot_last_swing[n]=false;
  }



  RCLCPP_INFO(this->get_logger(),"starting");

  // initialise the subscriptions
  pose_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("placements", 10);

  twist_sub = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 1, std::bind(&gait_synth::onCommand, this, std::placeholders::_1));

}



void gait_synth::onCommand(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  auto output = std::make_unique<geometry_msgs::msg::PoseArray>();
  output->header.frame_id = "palm";
  rclcpp::Duration msg_advance(0,500*1000*1000);
  output->header.stamp= this->get_clock()->now() + msg_advance;
  output->poses.clear();
  output->poses.resize(n_points);

  // convert the message into an Eigen format
  Eigen::Vector3d msg_pos(msg->linear.x, msg->linear.y, msg->linear.z);
  Eigen::Matrix3d msg_rot = (
    Eigen::AngleAxisd(msg->angular.x, Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd(msg->angular.y,  Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(msg->angular.z, Eigen::Vector3d::UnitZ())
    ).toRotationMatrix();

  // convert the body pose into an Eigen format
  Eigen::Vector3d contact_centre(-body_position.at("x"), -body_position.at("y"), -body_position.at("z"));
  Eigen::Matrix3d contact_rotation = (
      Eigen::AngleAxisd(body_orientation.at("roll"), Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(body_orientation.at("pitch"), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(body_orientation.at("yaw"), Eigen::Vector3d::UnitZ())
    ).inverse().toRotationMatrix();

  //advance the phase of the gait by some amount
  double phase_advance = (
      msg_pos.norm() + std::abs(contact_radius * msg->angular.z)
    ) / step_size;
  phase = std::fmod(phase + phase_advance, 1.0);
  if(phase < 0) phase += 1;

  RCLCPP_INFO(this->get_logger(),"phase = '%f'", phase);


  // vector to
  Eigen::Vector3d step_vector(0, 0, step_height);




  for(unsigned int n=0; n<n_points; n++)
  {
    double point_phase = std::fmod(phase + step_phase_offsets[n], 1.0);
    if(point_phase < 0) point_phase += 1;
    bool foot_swing = (point_phase > step_duties[n]);


    Eigen::Vector3d neutral_point =
      contact_centre + (
        contact_rotation *
        Eigen::AngleAxisd( contact_theta_offset + (2*M_PI*n/(double)n_points), Eigen::Vector3d::UnitZ()) *
        Eigen::Vector3d(contact_radius,0,contact_height)
      );
    
    
    // move the contact point according to the message
    Eigen::Vector3d prev_contact = contact_points.block(0,n,3,1);
    contact_points.block(0,n,3,1) = (msg_rot * prev_contact) + msg_pos;


    Eigen::Vector3d target_point = contact_points.block(0,n,3,1);


    if(foot_swing){

      if(!foot_last_swing[n])
      {
        foot_last_swing[n] = true;
        RCLCPP_INFO(this->get_logger(),"foot '%d' lift", n);
      }
      RCLCPP_INFO(this->get_logger(),"foot '%d' swinging", n);

      double swing_duty = 1.0 - step_duties[n];

      double dst_ratio = (point_phase - step_duties[n]) / swing_duty;
      double src_ratio = 1.0 - dst_ratio;
      double lift_ratio = 4 * dst_ratio * src_ratio;

      Eigen::Vector3d src_point = target_point; //previous contact
      Eigen::Vector3d dst_point = neutral_point; // add a bit in the direction of travel

      target_point = 
        (src_ratio * src_point) +
        (dst_ratio * dst_point) +
        (lift_ratio * step_vector);

    }
    else
    {
      if(foot_last_swing[n])
      {
        foot_last_swing[n] = false;
        RCLCPP_INFO(this->get_logger(),"foot '%d' drop", n);

        contact_points.block(0,n,3,1) = neutral_point;
        target_point = contact_points.block(0,n,3,1);
      }
      RCLCPP_INFO(this->get_logger(),"foot '%d' planted", n);

    }

    output->poses[n].position.x = target_point(0);
    output->poses[n].position.y = target_point(1);
    output->poses[n].position.z = target_point(2);



  }


  pose_pub->publish(std::move(output));

}


//relax the type safety a little for CLI users
rcl_interfaces::msg::SetParametersResult gait_synth::reconf_callback(const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & param : parameters)
  {

    if(param.get_name() == "n_points")
    {
      if(param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        n_points = param.as_double();
      if(param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        n_points = param.as_int();
    }
    if(param.get_name() == "contact_radius")
    {
      if(param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        contact_radius = param.as_double();
      if(param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        contact_radius = param.as_int();
    }
    if(param.get_name() == "contact_height")
    {
      if(param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        contact_height = param.as_double();
      if(param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        contact_height = param.as_int();
    }
    if(param.get_name() == "contact_theta_offset")
    {
      if(param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        contact_theta_offset = param.as_double();
      if(param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        contact_theta_offset = param.as_int();
    }
    if(param.get_name() == "step_size")
    {
      if(param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        step_size = param.as_double();
      if(param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        step_size = param.as_int();
    }
    if(param.get_name() == "step_height")
    {
      if(param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        step_height = param.as_double();
      if(param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        step_height = param.as_int();
    }


    if(param.get_name() == "step_phase_offsets")
    {
      if(param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY)
        step_phase_offsets = param.as_double_array();
      if(param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY)
        step_phase_offsets = std::vector<double>(param.as_integer_array().begin(), param.as_integer_array().end());
      if((unsigned int) n_points < step_phase_offsets.size())
        RCLCPP_WARN(this->get_logger(), "phase offset terms don't match n_points");
    }
    if(param.get_name() == "step_duties")
    {
      if(param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY)
        step_duties = param.as_double_array();
      if(param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY)
        step_duties = std::vector<double>(param.as_integer_array().begin(), param.as_integer_array().end());
      if((unsigned int) n_points < step_duties.size())
        RCLCPP_WARN(this->get_logger(), "phase duty terms don't match n_points");
    }


    if(param.get_name() == "body_position.x")
    {
      if(param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        body_position.at("x") = param.as_double();
      if(param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        body_position.at("x") = param.as_double();
    }
    if(param.get_name() == "body_position.y")
    {
      if(param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        body_position.at("y") = param.as_double();
      if(param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        body_position.at("y") = param.as_double();
    }
    if(param.get_name() == "body_position.z")
    {
      if(param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        body_position.at("z") = param.as_double();
      if(param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        body_position.at("z") = param.as_double();
    }

    if(param.get_name() == "body_orientation.roll")
    {
      if(param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        body_position.at("roll") = param.as_double();
      if(param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        body_position.at("roll") = param.as_double();
    }

    if(param.get_name() == "body_orientation.pitch")
    {
      if(param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        body_position.at("pitch") = param.as_double();
      if(param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        body_position.at("pitch") = param.as_double();
    }

    if(param.get_name() == "body_orientation.yaw")
    {
      if(param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        body_position.at("yaw") = param.as_double();
      if(param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        body_position.at("yaw") = param.as_double();
    }

  }

  return result;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<gait_synth>(options));
  rclcpp::shutdown();
  return 0;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(gait_synth)
