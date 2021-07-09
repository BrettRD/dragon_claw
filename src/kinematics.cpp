#include <claw/kinematics.h>





namespace claw
{



kinematics::kinematics(rclcpp::NodeOptions options):
  Node("kinematics", options)
{
/*
  task_space_weights = Eigen::MatrixXd::Identity(6,6);
  //task_space_weights = Eigen::MatrixXd(6,3);
  //task_space_weights = Eigen::MatrixXd(3,6);
//  task_space_weights << 1,0,0, 0,0,0,
//                        0,1,0, 0,0,0,
//                        0,0,1, 0,0,0,
//                        0,0,0, 0,0,0,
//                        0,0,0, 0,0,0,
//                        0,0,0, 0,0,0;
  // setting only last 3 coordinates to become valuable
  task_space_weights(0,0) = 1.0;
  task_space_weights(1,1) = 1.0;
  task_space_weights(2,2) = 1.0;
  task_space_weights(3,3) = 0.0;
  task_space_weights(4,4) = 0.0;
  task_space_weights(5,5) = 0.0;

  std::cout << task_space_weights;
  std::cout << "\n";
  
  joint_space_weights = Eigen::MatrixXd::Identity(6,6);
  std::cout << joint_space_weights;
  
  //parameters for which joints to care about

*/


  // parameters to turn the kinematics engine
  maxIterations = 200;   // unoptimised openscad converged in 5 steps
  epsilon = 1e-4;  // robot is in millimeters, try for 100um
  model_root_name = "palm";
  model_tip_prefix = "claw_point_";
  num_legs = 5;


  cb_group = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  auto sub_opts = rclcpp::SubscriptionOptions();
  sub_opts.callback_group = cb_group;

  // subscribe to /robot_description with QoS relaible and transient local to recover the latest message
  sub_robot_description = this->create_subscription<std_msgs::msg::String>(
    "robot_description",
    rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&kinematics::new_model_cb, this, std::placeholders::_1),
    sub_opts
  );

  // subscribe to ~/placements with default QoS for desired poses
  sub_placements = this->create_subscription<geometry_msgs::msg::PoseArray>(
    "placements",
    rclcpp::SensorDataQoS(),
    std::bind(&kinematics::pose_cb, this, std::placeholders::_1),
    sub_opts
  );

  pub_joint_targets = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);


}

/*  
 *  process new robot descriptions as they arrive
 *  
 */
void kinematics::new_model_cb(const std_msgs::msg::String::SharedPtr msg)
{
  load_model(msg->data);
}

void kinematics::pose_cb(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  sensor_msgs::msg::JointState joint_states;
  joint_states.header.stamp = msg-> header.stamp;
  joint_states.header.frame_id = msg-> header.frame_id;

  for(unsigned int i=0; i<msg->poses.size(); i++)
  {
    int n_joints = leg_chains[i].getNrOfJoints();

    KDL::JntArray initial_pose = KDL::JntArray(n_joints);
    KDL::Frame target = KDL::Frame( KDL::Vector(
      msg->poses[i].position.x,
      msg->poses[i].position.y,
      msg->poses[i].position.z));
    KDL::JntArray final_pose = KDL::JntArray(n_joints);
    for(int joint_idx=0; joint_idx<n_joints; joint_idx++)
    {
      initial_pose(joint_idx) = 0;
    }

    RCLCPP_INFO(this->get_logger(), "got cartesian pose %f, %f, %f",
      msg->poses[i].position.x,
      msg->poses[i].position.y,
      msg->poses[i].position.z);

    bool pose_ok = calculate_pose(ik_solver_pos[i], initial_pose, target, final_pose);
    //if(pose_ok)
    {
      unsigned int jnt_idx = 0;
      for(unsigned int seg_idx = 0; seg_idx<leg_chains[i].getNrOfSegments(); seg_idx++)
      {
        const KDL::Joint & kdl_joint = leg_chains[i].getSegment(seg_idx).getJoint();
        if(kdl_joint.getType() != KDL::Joint::None)
        {
          joint_states.name.push_back(kdl_joint.getName());
          if(pose_ok)
          {
            joint_states.position.push_back(final_pose(jnt_idx));
          }
          else
          {
            joint_states.position.push_back(initial_pose(jnt_idx));
          }
        }
        jnt_idx++;
      }
    }

    //publish joint_states
  }
  pub_joint_targets->publish(joint_states);
}



/*  
 *  load robot descriptions into KDL trees
 *  
 */
bool kinematics::load_model(std::string urdf_string)
{

  if (!model.initString(urdf_string)){
    RCLCPP_ERROR(this->get_logger(), "Failed to parse urdf file");
    return -1;
  }

  if (!kdl_parser::treeFromUrdfModel(model, tree)){
    RCLCPP_ERROR(this->get_logger(), "Failed to construct kdl tree");
    return false;
  }
  
  // XXX compute num_legs from the URDF
  
  leg_chains.resize(num_legs);
  fk_solver.resize(num_legs);
  ik_solver_vel.resize(num_legs);
  ik_solver_pos.resize(num_legs);
  leg_joint_lim_min.resize(num_legs);
  leg_joint_lim_max.resize(num_legs);
  leg_joint_lim_vel.resize(num_legs);
  //int v_error;
  for (int i=0; i<num_legs; i++)
  {
    std::string tip_name = model_tip_prefix + std::to_string(i);

    //build a kinematic chain for one leg
    if (!tree.getChain(model_root_name, tip_name, leg_chains[i])) {
      RCLCPP_ERROR(this->get_logger(), "could not generate kdl chain from '%s' to '%s'", model_root_name.c_str(), tip_name.c_str());
      return false;
    }

    // read the urdf joint limits separately
    if(!collect_joint_limits(
      model,
      leg_chains[i],
      leg_joint_lim_min[i], 
      leg_joint_lim_max[i],
      leg_joint_lim_vel[i]))
    {
      RCLCPP_ERROR(this->get_logger(), "could not collect limits in chain from '%s' to '%s'", model_root_name.c_str(), tip_name.c_str());
      return false;
    }



    // initialise the solvers for each leg
    

    fk_solver[i] = std::make_shared<KDL::ChainFkSolverPos_recursive>(leg_chains[i]);
    /*
    ik_solver_vel[i] = std::make_shared<KDL::ChainIkSolverVel_wdls>(leg_chains[i]);
    v_error = ik_solver_vel[i]->setWeightTS(task_space_weights);
     //int v_error = ik_solver_vel[i]->setWeightTS();
    switch(v_error)
    {
      case KDL::SolverI::E_NOERROR:
      break;
      case KDL::SolverI::E_NOT_IMPLEMENTED:
        RCLCPP_ERROR(this->get_logger(), "setWeightTS: not implemented");
        break;
      case KDL::SolverI::E_NOT_UP_TO_DATE:
        RCLCPP_ERROR(this->get_logger(), "setWeightTS: angry solver wants an update");
        break;
      case KDL::SolverI::E_SIZE_MISMATCH:
        RCLCPP_ERROR(this->get_logger(), "setWeightTS: size mismatch");
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "setWeightTS: misc error %d", v_error);
    }
    v_error = ik_solver_vel[i]->setWeightJS(joint_space_weights);
    switch(v_error)
    {
      case KDL::SolverI::E_NOERROR:
      break;
      case KDL::SolverI::E_NOT_IMPLEMENTED:
        RCLCPP_ERROR(this->get_logger(), "setWeightJS: not implemented");
        break;
      case KDL::SolverI::E_NOT_UP_TO_DATE:
        RCLCPP_ERROR(this->get_logger(), "setWeightJS: angry solver wants an update");
        break;
      case KDL::SolverI::E_SIZE_MISMATCH:
        RCLCPP_ERROR(this->get_logger(), "setWeightJS: size mismatch");
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "setWeightJS: misc error %d", v_error);
    }

    ik_solver_vel[i]->updateInternalDataStructures();
    */
    ik_solver_vel[i] = std::make_shared<KDL::ChainIkSolverVel_pinv>(leg_chains[i]);
    ik_solver_pos[i] = std::make_shared<KDL::ChainIkSolverPos_NR_JL>(leg_chains[i],
      leg_joint_lim_min[i], leg_joint_lim_max[i],
      *(fk_solver[i]), *(ik_solver_vel[i]), maxIterations, epsilon);


    //try to pose each chain
    int n_joints = leg_chains[i].getNrOfJoints();

    KDL::Frame tip_frame;
    KDL::JntArray test_pose = KDL::JntArray(n_joints);
    KDL::JntArray test_pose_out = KDL::JntArray(n_joints);
    test_pose(0) = 0;
    test_pose(1) = 0;
    test_pose(2) = 0;
    if(! tip_location(fk_solver[i], test_pose, tip_frame))
    {
      RCLCPP_ERROR(this->get_logger(), "test_pose failed for '%s'", tip_name.c_str());
    }
    else
    {
      KDL::Vector tip_coords = tip_frame * KDL::Vector::Zero();
      KDL::Frame tip_pos_frame = KDL::Frame(tip_coords);
      RCLCPP_INFO(this->get_logger(), "test_pose for '%s' is at %f, %f, %f ", tip_name.c_str(), tip_coords.x(), tip_coords.y(), tip_coords.z());

      if(calculate_pose(ik_solver_pos[i], test_pose, tip_pos_frame, test_pose_out))
      {
        RCLCPP_INFO(this->get_logger(), "calculate pose ok for '%s'", tip_name.c_str());
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "calculate pose failed for '%s'", tip_name.c_str());
      }
    }






  }
  return true;

}

/*  
 *  compute the joint angles to place an end effector at the target point in body coordinates
 *  simple wrap over CartToJnt
 */
bool kinematics::calculate_pose(
  std::shared_ptr<KDL::ChainIkSolverPos> ik_pos,
  KDL::JntArray &initial_pose,
  KDL::Frame &target,
  KDL::JntArray &final_pose
)
{
  // XXX does this Frame need to be composed with the chain root?
  // XXX is this implicitly responsive to motion of the root segment?
  int ik_error = ik_pos->CartToJnt(initial_pose, target, final_pose);

  if(KDL::SolverI::E_NOERROR == ik_error)
  {
    return true;
  }

  switch(ik_error)
  {
    case KDL::SolverI::E_MAX_ITERATIONS_EXCEEDED:
      RCLCPP_ERROR(this->get_logger(), "calculate_pose: could not find solution");
      break;
    case KDL::SolverI::E_NOT_UP_TO_DATE:
      RCLCPP_ERROR(this->get_logger(), "calculate_pose: angry solver wants an update");
      break;
    case KDL::SolverI::E_SIZE_MISMATCH:
      RCLCPP_ERROR(this->get_logger(), "calculate_pose: vector size mismatch");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "calculate_pose: misc error");
  }
  return false;
}

/*  
 *  compute the joint angles to place an end effector at the target point in body coordinates
 *  simple wrap over JntToCart
 */
bool kinematics::tip_location(
  std::shared_ptr<KDL::ChainFkSolverPos> fk_pos,
  KDL::JntArray &pose,
  KDL::Frame &location
)
{
  int fk_error = fk_pos->JntToCart(pose, location);
  if(KDL::SolverI::E_NOERROR == fk_error)
  {
    return true;
  }
  switch(fk_error)
  {
    case KDL::SolverI::E_SIZE_MISMATCH:
      RCLCPP_ERROR(this->get_logger(), "tip_location: number of joints in pose differs from joints in chain");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "tip_location: misc error");
  }
  return false;
}




//kinematics::find_limit(KDL::Chain, KDL::JntArray initial_pose, KDL::Vector target_vector, KDL::JntArray &max_pose){}


// walks through all chain segments, looks for KDL joints,
//   matches them to URDF joints, and collects limit information
//   http://lists.ros.org/lurker/message/20110405.025629.4938c5a6.nl.html
bool kinematics::collect_joint_limits(
  urdf::Model &robot_model,
  KDL::Chain &kdl_chain,
  KDL::JntArray &joint_min, 
  KDL::JntArray &joint_max,
  KDL::JntArray &joint_vel)
{
  int n_segments = kdl_chain.getNrOfSegments();
  int n_joints = kdl_chain.getNrOfJoints();

  //RCLCPP_INFO(this->get_logger(),  "chain has %d joints", n_joints);

  joint_min.resize(n_joints);
  joint_max.resize(n_joints);
  joint_vel.resize(n_joints);

  std::shared_ptr<const urdf::Joint> urdf_joint;

  int jnt_idx = 0;

  for (int seg_idx = 0; seg_idx<n_segments; seg_idx++)
  {
    const KDL::Joint & kdl_joint = kdl_chain.getSegment(seg_idx).getJoint();

    if (kdl_joint.getType() != KDL::Joint::None)
    {
      urdf_joint = robot_model.getJoint(kdl_joint.getName().c_str());
      // check joint was found in the URDF model of the robot
      if (!urdf_joint)
      {
        RCLCPP_ERROR(this->get_logger(), "Joint '%s' was not found in the URDF", urdf_joint->name.c_str());
        return false;
      }

      // extract joint information
      switch(urdf_joint->type)
      {
        case urdf::Joint::REVOLUTE:
        case urdf::Joint::PRISMATIC:
        {
          if(urdf_joint->limits)
          {
            joint_min(jnt_idx) = urdf_joint->limits->lower;
            joint_max(jnt_idx) = urdf_joint->limits->upper;
            joint_vel(jnt_idx) = urdf_joint->limits->velocity;
          }
          else
          {
            RCLCPP_ERROR(this->get_logger(), "Joint '%s' requires limits", urdf_joint->name.c_str());
            return false;
          }
        }
        break;

        case urdf::Joint::CONTINUOUS:
        {
          joint_min(jnt_idx) = -M_PI;
          joint_max(jnt_idx) = M_PI;
          joint_vel(jnt_idx) = 0;
          if(urdf_joint->limits)
          {
            joint_vel(jnt_idx) = urdf_joint->limits->velocity;
          }
        }
        break;

        case urdf::Joint::PLANAR:
        case urdf::Joint::FLOATING:
        {
          if(urdf_joint->limits)
          {
            joint_min(jnt_idx) = urdf_joint->limits->lower;
            joint_max(jnt_idx) = urdf_joint->limits->upper;
            joint_vel(jnt_idx) = urdf_joint->limits->velocity;
          }
          else{
            joint_min(jnt_idx) = -1e9;
            joint_max(jnt_idx) = 1e9;
            joint_vel(jnt_idx) = 0;
          }
        }
        break;

        case urdf::Joint::FIXED:
        {
          joint_min(jnt_idx) = 0;
          joint_max(jnt_idx) = 0;
          joint_vel(jnt_idx) = 0;
        }
        break;

        case urdf::Joint::UNKNOWN:
        default:
        {
          RCLCPP_ERROR(this->get_logger(), "Joint '%s' has unknown type", urdf_joint->name.c_str());
          return false;
        }
        break;
      }
      //increment the joint index for every kdl joint
      jnt_idx++;
    }
  }  
  return true;
}









}


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<claw::kinematics>(options));
  rclcpp::shutdown();
  return 0;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(claw::kinematics)
