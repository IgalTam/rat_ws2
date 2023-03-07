
/* Author: Sam Cole
   Desc:   Rover Arm Turret hw interface
*/

#include <rat_control/rat_hw_interface.h>

namespace rat_ns
{
RatHWInterface::RatHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model)
  : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
{
  telemetry_sub = nh.subscribe("/roboclaw_telemetry", 1, &RatHWInterface::telemetryCallback, this);
  cmd_pub = nh.advertise<rat_control::armCmd>("/roboclaw_cmd", 1);
  cmd_sub = nh.subscribe("/roboclaw_cmd", 1, &RatHWInterface::cmdCallback, this);

  // unused read to resolve overload
  // PENDING TROUBLESHOOTING
  // ROS_INFO("overloaded read message\n");
  // ros::topic::waitForMessage<rat_control::armCmd>("/roboclaw_cmd", nh, ros::Duration());

  ROS_INFO("ratHWInterface Constructed!\n");

}

void RatHWInterface::telemetryCallback(const rat_control::ratTelemetry::ConstPtr &msg)
{
// float32[4] angle # degrees
// #float32[6] vel # deg/s
// #float32[6] current # amps
// #time armReadTimestamp 
// time startSyncTime 
// # uint32 isrTicks # this would overflow if the robot is left on for 497 days straight at 100 hz 
// # uint8 bufferHealth

  for (int i=0; i<num_joints_; i++)
  {
    joint_position_[i] = msg->angle[i];
    //joint_velocity_[i] = msg->vel[i];
  }




}

void RatHWInterface::cmdCallback(const rat_control::armCmd::ConstPtr &msg) {
  /* callback on reading from roboclaw_cmd topic, updates Moveit upon successful movement*/

  /* wait for move to finish */
  for (int j=0; j < 1000; j++) {
    ;
  }

  ROS_INFO("Transcribing physical position...");

    for (int i=0; i<num_joints_; i++)
  {
    joint_position_[i] = msg->position_rads[i];
  }

  ROS_INFO("Yielding to Moveit planning...");


  /* wait for move to finish */
  for (int j=0; j < 1000; j++) {
    ;
  }

}


void RatHWInterface::init()
{
  // Call parent class version of this function
  GenericHWInterface::init();

  // // Resize vectors
  // joint_position_prev_.resize(num_joints_, 0.0);

  ROS_INFO("RatHWInterface Ready.");
}

void RatHWInterface::read(ros::Duration& elapsed_time)
{
  // No need to read since the roboclaw driver node updates "/roboclaw_telemetry" 
  // and we have a subscriber to that

  // adding in "/roboclaw_telemetry" read to indicate completion of move to Moveit
  // the actual read value is unimportant
  // PENDING TROUBLESHOOTING
  // ros::NodeHandle dummy;

  // ROS_INFO("waiting on RatHWInterface read op\n");
  // try {
  //   ros::topic::waitForMessage<rat_control::armCmd>("/roboclaw_cmd", &dummy, ros::Duration());
  // } catch(int dum) {
  //   ;
  // }
  // ros::topic::waitForMessage<rat_control::armCmd>("/roboclaw_cmd");



  ros::spinOnce(); // this spin will trigger ros to check all callbacks and update state vecs
}

void RatHWInterface::write(ros::Duration& elapsed_time)
{
  // armCmd msg:
  // float32[4] position_rads # pos in rads, converted in subscriber.py to encoder counts
  // int32[4] speed 
  // int32[4] accel_deccel 

  // Safety
  // enforceLimits(elapsed_time);
  
  // static ensures that this is only created once and persists between function calls
  static rat_control::armCmd arm_cmd_msg;

  for (int i=0; i<num_joints_; i++)
  {
    arm_cmd_msg.position_rads[i]=joint_position_command_[i];
  }
  
  cmd_pub.publish(arm_cmd_msg);

}

void RatHWInterface::enforceLimits(ros::Duration& period)
{
  // // Enforces position and velocity
  // pos_jnt_sat_interface_.enforceLimits(period);
}


}  // namespace ros_ns
