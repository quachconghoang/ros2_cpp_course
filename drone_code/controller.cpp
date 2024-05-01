#include "controller.h"
#include <cmath>


///////////////////////////////////////////////////////////////
//! @todo
//! TASK 1 - Initialisation
//!
//! Is there anything we need to initialise in the Constructor?

Controller::Controller() :
    odo_{0,0,0,0,0,0},
    distance_travelled_(0),
    time_travelled_(0),
    cmd_pipe_seq_(0)
{
    // We create a pointer to the PfmConnector here in the constructor, so we can OPEN connection ONLY once
    // As they are part of base class
    pfmsConnectorPtr_ = std::make_unique<PfmsConnector>();

    //We set the internal variables of time/distance for goal to zero
    goal_.time=0;
    goal_.distance=0;
};


bool Controller::setGoal(pfms::geometry_msgs::Point goal) {
  goal_.location = goal;
  return calcNewGoal();
}

bool Controller::setGoals(std::vector<pfms::geometry_msgs::Point> goals)
{
    // Store in Buffer


    return 0;
}
bool Controller::setTolerance(double tolerance) {
  tolerance_ = tolerance;
  return true;
}

double Controller::distanceToGoal(void) {
    return goal_.distance;
}
double Controller::timeToGoal(void) {
    return goal_.time;
}
double Controller::distanceTravelled(void) {
    return distance_travelled_;
}
double Controller::timeTravelled(void) {
    return time_travelled_;
}

bool Controller::goalReached() {
    double dx = goal_.location.x - odo_.position.x;
    double dy = goal_.location.y - odo_.position.y;

    return (pow(pow(dx,2)+pow(dy,2),0.5) < tolerance_);
}


pfms::nav_msgs::Odometry Controller::getOdometry(void){
    pfmsConnectorPtr_->read(odo_,type_);
    return odo_;
}

pfms::PlatformType Controller::getPlatformType(void){
    return type_;
}

// new functions implementation
// Run function implementation
void Controller::run(void){
//    // Get the current odometry
//    pfms::nav_msgs::Odometry current_odometry = getOdometry();
//
//    // Check if the goal has been reached
//    if(goalReached()){
//        // If the goal has been reached, we need to calculate a new goal
//        calcNewGoal();
//    }
//
//    // Calculate the distance travelled
//    double dx = current_odometry.position.x - odo_.position.x;
//    double dy = current_odometry.position.y - odo_.position.y;
//    double distance = pow(pow(dx,2)+pow(dy,2),0.5);
//
//    // Update the distance travelled
//    distance_travelled_ += distance;
//
//    // Update the time travelled
//    time_travelled_ += 0.1;
}

// Status function implementation
pfms::PlatformStatus Controller::status(void){
    // Get the current odometry
    pfms::PlatformStatus status;

    return status;
}

std::vector<pfms::geometry_msgs::Point> Controller::getObstacles(void){
    std::vector<pfms::geometry_msgs::Point> obstacles;
    return obstacles;

}

