
#include "audi.h" 
#include "controller.h" // link to its head file

// makin' the Constructor for the Controller class.
Controller::Controller() {

    // make a new Audi instance to work with
    Audi audi_ = Audi();

}

Controller::~Controller() {}


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
bool Controller::checkOriginToDestination(pfms::nav_msgs::Odometry origin, 
                                            pfms::geometry_msgs::Point goal, 
                                                double& distance, 
                                                    double& time, 
                                                        pfms::nav_msgs::Odometry& estimatedGoalPose) {  


        //calling upon the methord to carryout the check on the audi platform                             
    return audi_.checkOriginToDestination(current_platform_odometry_, target_goal_, distance_platform_to_goal_, estimated_time_to_goal_, estimated_platform_goal_pose_);
}


pfms::PlatformType Controller::getPlatformType(void) { 

    // retriving the type of platform currently in use.
    return platform_type_;

}
