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
    bool setGoals(std::vector<pfms::geometry_msgs::Point> goals);
    void run(void);
    pfms::PlatformType getPlatformType()  {return pfms::PlatformType::ACKERMAN;}

protected:
    // setting up attributes needed only by Ackerman to operate.

//    double Steering_angle_; // holds the Current steering angle.
//    double max_brake_force_; // hold value for highest brake torque that can be applied.
//    double max_throttle_; // hold throttle value for accelrating.
//
//    pfms::commands::Ackerman command_; //!< Ackerman command for platform control.
//    Audi audi_; // Instance of the Audi class
};

#endif // ACKERMAN_H
