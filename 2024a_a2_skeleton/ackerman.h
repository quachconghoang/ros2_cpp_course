#ifndef ACKERMAN_H
#define ACKERMAN_H

#include "controller.h"
#include "audi.h"

class Ackerman: public Controller
{
public:
  //Default constructor should set all sensor attributes to a default value
    Ackerman();

  // Please Add Doxygen comments to these functions
    void run(void);

    bool checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                  pfms::geometry_msgs::Point goal,
                                  double& distance,
                                  double& time,
                                  pfms::nav_msgs::Odometry& estimatedGoalPose);

protected:
    bool reachGoal(void);
    bool calcNewGoal(void);

private:
    // setting up attributes needed only by Ackerman to operate.
    double Steering_angle_; // holds the Current steering angle.
    double max_brake_force_; // hold value for highest brake torque that can be applied.
    double max_throttle_; // hold throttle value for accelrating.

    // Estimated time to drive to the goal.
    double estimated_time_to_goal_;
    // Distance of platform to the goal.
    double distance_platform_to_goal_;
    // Total distance travelled by the vehicle since started.
//    double distance_travelled_by_platform_;
    // Total time in seconds the platform has been in motion.
//    double time_in_motion_by_platform_;

    pfms::commands::Ackerman command_; //!< Ackerman command for platform control.
    Audi audi_; // Instance of the Audi class
    pfms::nav_msgs::Odometry current_platform_odometry_;
    // Estimated pose of the vehicle when the goal is reached.
    pfms::nav_msgs::Odometry estimated_platform_goal_pose_;
};

#endif // ACKERMAN_H
