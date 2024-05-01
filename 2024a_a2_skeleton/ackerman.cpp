#include "ackerman.h"

Ackerman::Ackerman() {

    // Default constructor should set all sensor attributes to a default value
}

void Ackerman::run(void) {
    // Implement this function
    std::cout<<"Ackerman::run"<<std::endl;
}

bool Ackerman::reachGoal(void) {
    // Implement this function
    std::cout<<"Ackerman::reachGoal"<<std::endl;
    return true;
}

bool Ackerman::calcNewGoal() {
    // Implement this function
    std::cout<<"Ackerman::calcNewGoal"<<std::endl;
    return true;
}

bool Ackerman::checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                          pfms::geometry_msgs::Point goal,
                                          double& distance,
                                          double& time,
                                          pfms::nav_msgs::Odometry& estimatedGoalPose) {


    //calling upon the methord to carryout the check on the audi platform
    return audi_.checkOriginToDestination(odo_, goal_.location, distance_platform_to_goal_, estimated_time_to_goal_, estimated_platform_goal_pose_);
}
