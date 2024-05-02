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
    bool checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                  pfms::geometry_msgs::Point goal,
                                  double& distance,
                                  double& time,
                                  pfms::nav_msgs::Odometry& estimatedGoalPose);

protected:
    bool reachGoal(void);
    bool calcNewGoal(void);
    Audi * audi_; // Instance of the Audi class
private:
    // setting up attributes needed only by Ackerman to operate.
    double Steering_angle_; // holds the Current steering angle.
    double max_brake_force_; // hold value for highest brake torque that can be applied.
    double max_throttle_; // hold throttle value for accelrating.

    pfms::commands::Ackerman command_; //!< Ackerman command for platform control.

};

#endif // ACKERMAN_H
