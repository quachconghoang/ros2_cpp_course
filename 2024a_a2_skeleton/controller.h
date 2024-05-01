#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "controllerinterface.h"
#include <cmath>
#include <pfmsconnector.h>
#include <thread>

class Controller: public ControllerInterface
{
public:
//Default constructors should set all attributes to a default value
    Controller();

    // Inherited from ControllerInterface
    // Please Add Doxygen comments to these functions
    void run(void);

    pfms::PlatformStatus status(void);

    bool setGoals(std::vector<pfms::geometry_msgs::Point> goals);

    bool checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                   pfms::geometry_msgs::Point goal,
                                   double& distance,
                                   double& time,
                                   pfms::nav_msgs::Odometry& estimatedGoalPose);

    pfms::PlatformType getPlatformType(void);

    double distanceToGoal(void);

    double timeToGoal(void);

    bool setTolerance(double tolerance);

    double distanceTravelled(void);

    double timeTravelled(void);

    pfms::nav_msgs::Odometry getOdometry(void);

    std::vector<pfms::geometry_msgs::Point> getObstacles();

protected:

};

#endif // CONTROLLER_H
