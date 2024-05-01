#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "controllerinterface.h"
#include <cmath>
#include <pfmsconnector.h>
#include <thread>

//! Information about the goal for the platform
struct GoalStats {
    //! location of goal
    pfms::geometry_msgs::Point location;
    //! distance to goal
    double distance;
    //! time to goal
    double time;
};

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
    pfms::PlatformType getPlatformType(void);

    double distanceToGoal(void);
    double timeToGoal(void);
    bool setTolerance(double tolerance);
    double distanceTravelled(void);
    double timeTravelled(void);

    pfms::nav_msgs::Odometry getOdometry(void);
    std::vector<pfms::geometry_msgs::Point> getObstacles();

protected:
    /* Support for RUN function */
    virtual bool reachGoal(void) = 0;
    virtual bool calcNewGoal(void) = 0;
    /**
    * Checks if the goal has been reached.
    *
    * Update own odometry before calling!
    * @return true if the goal is reached
    */
    bool goalReached();

    std::unique_ptr<PfmsConnector> pfmsConnectorPtr_; //!< The pipe to communicate
    pfms::nav_msgs::Odometry odo_;//!< The current pose of platform
    std::vector<pfms::geometry_msgs::Point> goals_queue_; //!< The goals to reach

    //stats
    GoalStats goal_; //!< The current goal
    pfms::PlatformType type_; //!< The type of platform

    double distance_travelled_; //!< Total distance travelled for this program run
    double time_travelled_; //!< Total time spent travelling for this program run
    double tolerance_; //!< Radius of tolerance
    long unsigned int cmd_pipe_seq_; //!<The sequence number of the command
};

#endif // CONTROLLER_H
