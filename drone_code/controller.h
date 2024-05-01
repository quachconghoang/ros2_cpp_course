#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "controllerinterface.h"
#include <cmath>
#include <pfmsconnector.h>

//! Information about the goal for the platform
struct GoalStats {
    //! location of goal
    pfms::geometry_msgs::Point location;
    //! distance to goal
    double distance;
    //! time to goal
    double time;
};

/**
 * \brief Shared functionality/base class for platform controllers
 *
 * Platforms need to implement:
 * - Controller::calcNewGoal (and updating GoalStats)
 * - ControllerInterface::reachGoal (and updating PlatformStats)
 * - ControllerInterface::checkOriginToDestination
 * - ControllerInterface::getPlatformType
 * - ControllerInterface::getOdometry (and updating PlatformStats.odo)
 */
class Controller: public ControllerInterface
{
public:
  /**
   * Default Controller constructor, sets odometry and metrics to initial 0
   */
    Controller();

  /**
   * Instructs the underlying platform to recalcuate a goal, and set any internal variables as needed
   *
   * Called when goal or tolerance changes
   * @return Whether goal is reachable by the platform
   */
    virtual bool calcNewGoal() = 0;

  //ControllerInterface functions (all doxygen comments in the files)
    bool setGoal(pfms::geometry_msgs::Point goal);

    // 1st Implements
    bool setGoals(std::vector<pfms::geometry_msgs::Point> goals);

    // 2nd Reuse goal reached - override in Vehicle CLASS (3D for drone and 2D for car - or including the 3D pose for car as well)

    // 3rd Implements the run function in Vehicle CLASS
//    void run(void);

    // 4th Implements the status function
    pfms::PlatformStatus status(void);

    // 5th Implements the getObstacles function
    std::vector<pfms::geometry_msgs::Point> getObstacles(void);






  pfms::PlatformType getPlatformType(void);

  bool setTolerance(double tolerance);

  double distanceTravelled(void);

  double timeTravelled(void);

  double distanceToGoal(void);

  double timeToGoal(void);

  /**
   * Updates the internal odometry
   *
   * Sometimes the pipes can give all zeros on opening, this has a little extra logic to ensure only valid data is
   * accepted
   */
  pfms::nav_msgs::Odometry getOdometry(void);


protected:
  /**
   * Checks if the goal has been reached.
   *
   * Update own odometry before calling!
   * @return true if the goal is reached
   */
  bool goalReached();

    std::vector<GoalStats> goals_raw; //!< The goals to reach
    std::vector<GoalStats> goals_optimizes; //!< The goals with optimized path

  std::unique_ptr<PfmsConnector> pfmsConnectorPtr_; //!< The pipe to communicate
  pfms::nav_msgs::Odometry odo_;//!< The current pose of platform

  //stats
  GoalStats goal_;
  pfms::PlatformType type_;


  double distance_travelled_; //!< Total distance travelled for this program run
  double time_travelled_; //!< Total time spent travelling for this program run
  double tolerance_; //!< Radius of tolerance
  long unsigned int cmd_pipe_seq_; //!<The sequence number of the command

};

#endif // CONTROLLER_H
