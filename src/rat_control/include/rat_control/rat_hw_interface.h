
/* Author: Sam Cole
   Desc:   Rover Arm Turret interface
*/

#ifndef RAT_HW_INTERFACE_H
#define RAT_HW_INTERFACE_H

#include <ros_control_boilerplate/generic_hw_interface.h>
#include <rat_control/armCmd.h>
#include <rat_control/ratTelemetry.h>

namespace rat_ns
{
/** \brief Hardware interface for a robot */
class RatHWInterface : public ros_control_boilerplate::GenericHWInterface
{
public:
  /**
   * \brief Constructor
   * \param nh - Node handle for topics.
   */
  RatHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model = NULL);

  /** \brief Initialize the robot hardware interface */
  virtual void init();

  /** \brief Read the state from the robot hardware. */
  virtual void read(ros::Duration& elapsed_time);

  /** \brief Write the command to the robot hardware. */
  virtual void write(ros::Duration& elapsed_time);

  /** \breif Enforce limits for all values before writing */
  virtual void enforceLimits(ros::Duration& period);

protected:
  ros::Subscriber telemetry_sub;
  void telemetryCallback(const rat_control::ratTelemetry::ConstPtr &msg);
  ros::Publisher cmd_pub;

};  // class

}  // namespace rat_ns

#endif
