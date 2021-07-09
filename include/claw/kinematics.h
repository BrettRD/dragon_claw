#include <rclcpp/rclcpp.hpp>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <Eigen/Dense>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <urdf/model.h>


namespace claw
{
  class kinematics : public rclcpp::Node
  {
  public:
    kinematics(rclcpp::NodeOptions options);

    // subscription callbacks
    void new_model_cb(const std_msgs::msg::String::SharedPtr msg);
    void pose_cb(const geometry_msgs::msg::PoseArray::SharedPtr msg);

    //generate new kinematic trees
    bool load_model(std::string urdf_string);

    // find joint angles
    bool calculate_pose(
      std::shared_ptr<KDL::ChainIkSolverPos> ik_pos,
      KDL::JntArray &initial_pose,
      KDL::Frame &target,
      KDL::JntArray &final_pose);

    bool tip_location(
      std::shared_ptr<KDL::ChainFkSolverPos> fk_pos,
      KDL::JntArray &pose,
      KDL::Frame &location);

    //find_limit(chain, initial_pose, target_vector, &max_pose);
    bool collect_joint_limits(
      urdf::Model &robot_model,
      KDL::Chain &kdl_chain,
      KDL::JntArray &joint_min, 
      KDL::JntArray &joint_max,
      KDL::JntArray &joint_vel);


  private:

    // recursion and resolution limits for numeric IK

    unsigned int maxIterations;
    double epsilon;
    int num_legs;
    Eigen::MatrixXd task_space_weights;
    Eigen::MatrixXd joint_space_weights;



    // kinematics trees built from the urdf

    KDL::Tree tree;
    std::vector<KDL::Chain> leg_chains;
    std::vector<KDL::JntArray> leg_joint_lim_min;
    std::vector<KDL::JntArray> leg_joint_lim_max;
    std::vector<KDL::JntArray> leg_joint_lim_vel;
    
    std::vector<std::shared_ptr<KDL::ChainFkSolverPos_recursive> > fk_solver;  //passed to ik_solver_pos by reference
    std::vector<std::shared_ptr<KDL::ChainIkSolverVel_pinv> > ik_solver_vel;
    //std::vector<std::shared_ptr<KDL::ChainIkSolverVel_wdls> > ik_solver_vel;  //passed to ik_solver_pos by reference
    std::vector<std::shared_ptr<KDL::ChainIkSolverPos_NR_JL> > ik_solver_pos;

    rclcpp::CallbackGroup::SharedPtr cb_group;


    // subscription topic to be notified of changes to the robot
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_robot_description;
    // subscription for kinematic tip poses
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_placements;
    // publisher for joint states to reach the placements
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_targets;

    std::string last_urdf_string; // complete robot description xml
    urdf::Model model;  // parsed urdf model (used for extracting joint limits)

    // parameters for the names of the parts of interest
    std::string model_root_name;
    std::string model_tip_prefix;
  };
}
