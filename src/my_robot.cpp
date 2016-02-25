#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

class MyRobot : public hardware_interface::RobotHW
{
public:
  MyRobot() 
  {
    // connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle_a("A", &pos[0], &vel[0], &eff[0]);
    jnt_state_interface.registerHandle(state_handle_a);

    hardware_interface::JointStateHandle state_handle_b("B", &pos[1], &vel[1], &eff[1]);
    jnt_state_interface.registerHandle(state_handle_b);

    registerInterface(&jnt_state_interface);

    // connect and register the joint position interface
    hardware_interface::JointHandle pos_handle_a(jnt_state_interface.getHandle("A"), &cmd[0]);
    jnt_pos_interface.registerHandle(pos_handle_a);

    hardware_interface::JointHandle pos_handle_b(jnt_state_interface.getHandle("B"), &cmd[1]);
    jnt_pos_interface.registerHandle(pos_handle_b);

    registerInterface(&jnt_pos_interface);
  }

  void write()
  {
  }
  
  void read()
  {
    pos[0] = cmd[0];
    pos[1] = cmd[1];
  }

  ros::Time get_time()
  {
    prev_update_time = curr_update_time;
    curr_update_time = ros::Time::now();
    return curr_update_time;
  }

  ros::Duration get_period()
  {
    return curr_update_time - prev_update_time;
  }

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  double cmd[2];
  double pos[2];
  double vel[2];
  double eff[2];

  ros::Time curr_update_time, prev_update_time;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_robot_hw");
  ros::NodeHandle nh;

  ros::Rate loop_rate(10);

  ros::AsyncSpinner spinner(2);
  spinner.start();

  MyRobot robot;
  controller_manager::ControllerManager cm(&robot, nh);

  while (ros::ok())
  {
    robot.read();
    cm.update(robot.get_time(), robot.get_period());
    robot.write();
    loop_rate.sleep();
  }

  return 0;
}
