#ifndef QUADCOPTER_H
#define QUADCOPTER_H

#include "controller.h"

class Quadcopter: public Controller
{
public:
  //Default constructor - should set all sensor attributes to a default value
  Quadcopter();

  void run(void);

  bool setGoals(std::vector<pfms::geometry_msgs::Point> goals);

  pfms::PlatformType getPlatformType(void) {
      return pfms::PlatformType::QUADCOPTER;
  }

};

#endif // QUADCOPTER_H
