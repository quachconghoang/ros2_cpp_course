#ifndef ACKERMAN_H
#define ACKERMAN_H

#include "controller.h" 

/*!
 *  \brief     Ackerman Class
 *  \details  provides control over the characteristics of platforms that use ackerman steering such as Audi R8. 
 * It's methods inclue  reaching goals, estimating the time and distance to goals, and reading vehicle odometry (pose, velocity etc.).
 * It derives its atrubutes from Controller,
 * 
 * 
 *  \author    Wil
 *  \version   1.0
 *  \date      2024-03-26
 *  \pre       none
 *  \bug       none reported as of 2024-03-26
 */

// Ackerman inherits from Controller base class.
class Ackerman : public Controller {
public:
  
  //  constructor setsup 
  Ackerman();

  /*!
  this function uses ackerman steering to move and contol the audi R8 platform

  @param [in] origin of the audi R8
  @param [in] goal to drive to
  
  Setter for goal, the function will update internal variables asscoiated with @sa timeToGoal
  and @sa distanceToGoal
  @return goal reachable
  @sa controllerinterface.h
  */
  bool setGoal(pfms::geometry_msgs::Point goal);
  

  /**
   * Initialization:
    Computes the steering angle based on the current platform odometry and a target goal.
    Checks the origin-to-destination relationship and estimates time and platform goal pose.
    Distance Tracking:
    Updates the total distance traveled by the platform by adding the distance to the current goal.
    Approaching Goal:
    Enters a loop until the platform is approaching the goal:
    Sends drive commands (throttle, steering angle) to the platform.
    Checks if the distance to the goal is less than a threshold (0.55 in this case).
    Arrived at Goal:
    Enters another loop until the platform has arrived at the goal:
    Sends brake commands to stop the platform.
    Checks if the distance to the goal is within a tolerance value.
    Stopping the Platform:
    Continues another loop until the platform velocity is very low (less than 0.1 m/s).
    Sends brake commands to stop the platform.
    Final Check:
    Calculates the distance to the goal again.
    If the distance is still within the tolerance value, returns true and updates the total time in motion.
    Otherwise, returns false.
  Setter for action to reach goal, the function will update internal variables asscoiated with @sa timeToGoal
  and @sa distanceToGoal
  @param [in] current_platform_odometry_
  @param [in] platform_type_

  @return goal is sucessfully reached
  */
  bool reachGoal(void);

  /**
  Getter for distance to be travelled to reach goal, updates at the platform moves to current goal
  @return current distance to be travelled to goal [m]
  */

  bool setTolerance(double tolerance);

  double distanceToGoal(void);
  double timeToGoal(void);
  double platformDistanceTravelled(void);
  double timeInMotion(void);



  pfms::nav_msgs::Odometry getOdometry(void);

private:
  // setting up attributes needed only by Ackerman to operate.

  double Steering_angle_; // holds the Current steering angle.
  double max_brake_force_; // hold value for highest brake torque that can be applied.
  double max_throttle_; // hold throttle value for accelrating.
  
  pfms::commands::Ackerman command_; //!< Ackerman command for platform control.
  Audi audi_; // Instance of the Audi class 
};

#endif // ACKERMAN_H
