#include "controller.h"

Controller::Controller() {
    // Set all attributes to default values
    std::cout<<"Controller default constructor"<<std::endl;
}

void Controller::run(void) {
    // Implement this function
    std::cout<<"Controller::run"<<std::endl;
}

pfms::PlatformStatus Controller::status(void) {
    // Implement this function
    std::cout<<"Controller::status"<<std::endl;
    return pfms::PlatformStatus::IDLE;
}

bool Controller::setGoals(std::vector<pfms::geometry_msgs::Point> goals) {
    // Implement this function
    std::cout<<"Controller::setGoals"<<std::endl;
    return true;
}

bool Controller::checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                          pfms::geometry_msgs::Point goal,
                                          double& distance,
                                          double& time,
                                          pfms::nav_msgs::Odometry& estimatedGoalPose) {
    // Implement this function
    std::cout<<"Controller::checkOriginToDestination"<<std::endl;
    return true;
}

// NEED OVERWRITE in ACKERMAN and QUADCOPTER
pfms::PlatformType Controller::getPlatformType(void) {
    // Implement this function
    std::cout<<"Controller::getPlatformType"<<std::endl;
    return pfms::PlatformType::ACKERMAN;
}

// Can Implement in Controller
double Controller::distanceToGoal(void) {
    std::cout << "Controller::distanceToGoal" << std::endl;
    return 0.0;

}

// Not sure
double Controller::timeToGoal(void) {
    std::cout << "Controller::timeToGoal" << std::endl;
    return 0.0;
}

// Can Implement in Controller
bool Controller::setTolerance(double tolerance) {
    std::cout << "Controller::setTolerance" << std::endl;
    return true;
}

// Can Implement in Controller
double Controller::distanceTravelled(void) {
    std::cout << "Controller::distanceTravelled" << std::endl;
    return 0.0;
}

// Can Implement in Controller
double Controller::timeTravelled(void) {
    std::cout << "Controller::timeTravelled" << std::endl;
    return 0.0;
}

// Can Implement in Controller
pfms::nav_msgs::Odometry Controller::getOdometry(void) {
    std::cout << "Controller::getOdometry" << std::endl;
    return pfms::nav_msgs::Odometry();
}

// Can Implement in Controller
std::vector<pfms::geometry_msgs::Point> Controller::getObstacles() {
    std::cout << "Controller::getObstacles" << std::endl;
    return std::vector<pfms::geometry_msgs::Point>();
}

// Path: missioninterface.h
