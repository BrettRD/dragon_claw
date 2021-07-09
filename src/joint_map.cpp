/* joint_map.cpp
 * Author: Brett Downing <brettrd@brettrd.com>
 *
 * a simple mixer to map a joint_state_message to a PWM output
 * this is indended to drive servos from a pca9685 node.
 * composable nodes enabled
 */


#include <rclcpp/rclcpp.hpp>

#include <map>
#include <memory>
#include <string>

#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>


class joint_map : public rclcpp::Node
{
  public:
    joint_map(rclcpp::NodeOptions options);

  private:
    const int n_channels = 16;
    void onCommand(const sensor_msgs::msg::JointState::SharedPtr msg);
    rcl_interfaces::msg::SetParametersResult reconf_callback(const std::vector<rclcpp::Parameter> & parameters);

    OnSetParametersCallbackHandle::SharedPtr param_cb;

    std::vector<std::string> joint_list;
    std::vector<double> param_channel_bias;
    std::vector<double> param_channel_position_gain;
    std::vector<double> param_channel_velocity_gain;
    std::vector<double> param_channel_effort_gain;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pwm_pub;

};



joint_map::joint_map(rclcpp::NodeOptions options) : 
  Node("joint_map", options)
{
  RCLCPP_INFO(this->get_logger(),"initializing");

  this->declare_parameter("joints", joint_list);
  this->declare_parameter("bias", param_channel_bias);
  this->declare_parameter("position", param_channel_position_gain);
  this->declare_parameter("velocity", param_channel_velocity_gain);
  this->declare_parameter("effort", param_channel_effort_gain);

  //XXX get_parameter is overly sensitive to int/double type safety
  this->get_parameter("joints", joint_list);
  this->get_parameter("bias", param_channel_bias);
  this->get_parameter("position", param_channel_position_gain);
  this->get_parameter("velocity", param_channel_velocity_gain);
  this->get_parameter("effort", param_channel_effort_gain);

  param_cb = this->add_on_set_parameters_callback(
    std::bind(&joint_map::reconf_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(),"starting");

  pwm_pub = this->create_publisher<std_msgs::msg::Int32MultiArray>("command", 10);

  joint_sub = this->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 1, std::bind(&joint_map::onCommand, this, std::placeholders::_1));
}

void joint_map::onCommand(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  auto output = std::make_unique<std_msgs::msg::Int32MultiArray>();
  output->data.clear();
  output->data.resize(joint_list.size(), -1);

  for(unsigned int name_idx=0; name_idx < msg->name.size(); name_idx++)
  {
    for(unsigned int joint_idx=0; joint_idx < joint_list.size(); joint_idx++)
    {
      if( msg->name[name_idx] == joint_list[joint_idx])
      {
        double tmp = param_channel_bias[joint_idx];
        if(param_channel_position_gain.size() > joint_idx &&  msg->position.size() > name_idx )
        {
          tmp += param_channel_position_gain[joint_idx] * msg->position[name_idx];
        }
        if(param_channel_velocity_gain.size() > joint_idx &&  msg->velocity.size() > name_idx )
        {
          tmp += param_channel_velocity_gain[joint_idx] * msg->velocity[name_idx];
        }
        if(param_channel_effort_gain.size() > joint_idx &&  msg->effort.size() > name_idx )
        {
          tmp += param_channel_effort_gain[joint_idx] * msg->effort[name_idx];
        }
        output->data[joint_idx] = tmp;
      }
    }
  }

  pwm_pub->publish(std::move(output));
}


//relax the type safety a little for CLI users
rcl_interfaces::msg::SetParametersResult joint_map::reconf_callback(const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & param : parameters)
  {

    if(param.get_name() == "joints")
    {
      if(param.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
        joint_list = param.as_string_array();
    }
    if(param.get_name() == "bias")
    {
      if(param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY)
        param_channel_bias = param.as_double_array();
      if(param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY)
        param_channel_bias = std::vector<double>(param.as_integer_array().begin(), param.as_integer_array().end());
      if(joint_list.size() < param_channel_bias.size())
        RCLCPP_WARN(this->get_logger(),"bias terms exceed channel count");
    }
    if(param.get_name() == "position")
    {
      if(param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY)
        param_channel_position_gain = param.as_double_array();
      if(param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY)
        param_channel_position_gain = std::vector<double>(param.as_integer_array().begin(), param.as_integer_array().end());
      if(joint_list.size() < param_channel_position_gain.size())
        RCLCPP_WARN(this->get_logger(),"position terms exceed channel count");
    }
    if(param.get_name() == "velocity")
    {
      if(param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY)
        param_channel_velocity_gain = param.as_double_array();
      if(param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY)
        param_channel_velocity_gain = std::vector<double>(param.as_integer_array().begin(), param.as_integer_array().end());
      if(joint_list.size() < param_channel_velocity_gain.size())
        RCLCPP_WARN(this->get_logger(),"velocity terms exceed channel count");
    }
    if(param.get_name() == "effort")
    {
      if(param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY)
        param_channel_effort_gain = param.as_double_array();
      if(param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY)
        param_channel_effort_gain = std::vector<double>(param.as_integer_array().begin(), param.as_integer_array().end());
      if(joint_list.size() < param_channel_effort_gain.size())
        RCLCPP_WARN(this->get_logger(),"effort terms exceed channel count");
    }

  }
  return result;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<joint_map>(options));
  rclcpp::shutdown();
  return 0;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(joint_map)
