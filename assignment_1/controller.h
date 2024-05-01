#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "audi.h" 
#include "controllerinterface.h"
#include "pfmsconnector.h" 

/*!
 *  \brief     Controller Base Class
 *  \details This Base class inherits from ControllerInterface and provides the base implementations for controlling the platform that a non spesific to their individual handling charicterisctics.
 * its implments functions for setting the goal co ordinates, pcalcualting the distances to the goal from the platform and getting the odomerty status of the platform. 
 * 
 * 
 *  \author    Wil
 *  \version   1.0
 *  \date      2024-03-26
 *  \pre       none
 *  \bug       none reported as of 2024-03-26
 */

  // controller inherits from Controller Interface  class.
class Controller : public ControllerInterface {
public:
  

    Controller();
    ~Controller();

   /**
  Checks whether the platform can travel between origin and destination via constant steering angle
  @param[in] origin The origin pose, specified as odometry for the platform
  @param[in] goal The destination point for the platform
  @param[in|out] distance The distance [m] the platform will need to travel between origin and destination. If destination unreachable distance = -1
  @param[in|out] time The time [s] the platform will need to travel between origin and destination, If destination unreachable time = -1
  @param[in|out] estimatedGoalPose The estimated goal pose when reaching goal
  @return bool indicating the platform can reach the destination from origin supplied
  @sa audi.h
  */
    bool checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                    pfms::geometry_msgs::Point goal,
                                      double& distance,
                                        double& time,
                                          pfms::nav_msgs::Odometry& estimatedGoalPose);

    // geta the platform type
    pfms::PlatformType getPlatformType(void);

private:
    Audi audi_;

protected:
    // these protected members are only avalible to derived classes Ackerman and SkidSteer
    
    /**
     * based on the virtuals from the controller interface class
    @sa controllerinterface.h
    */
    
    // Tolerance for stopping platform within reaching desired the goal
    double tolerance_value_;
    // Estimated time to drive to the goal.
    double estimated_time_to_goal_;
    // Distance of platform to the goal.
    double distance_platform_to_goal_;
    // Total distance travelled by the vehicle since started.
    double distance_travelled_by_platform_;
    // Total time in seconds the platform has been in motion.
    double time_in_motion_by_platform_;

    // Stores platform type ACKERMAN/SKIDSTEER
    pfms::PlatformType platform_type_;
    // The goal position for the platform (x,y,z). This is specified from the struct point in pmfs/geometery_msg namespaces
    pfms::geometry_msgs::Point target_goal_;
    // Current odometry information of platform.
    pfms::nav_msgs::Odometry current_platform_odometry_;
    // Estimated pose of the vehicle when the goal is reached.
    pfms::nav_msgs::Odometry estimated_platform_goal_pose_;
    //PfmsConnector communication with ROS.
    std::shared_ptr<PfmsConnector> ROS_communication_;

};

#endif // CONTROLLER_H