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
    goals_raw.clear();
    for(auto points : goals)
    {
        GoalStats t_g;
        t_g.location = points;
        goals_raw.push_back(t_g);
    }

    std::cout << "Set " << goals_raw.size() << " goals" << std::endl;

    return true;
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
//    std::cout<< "General Goal reached" << std::endl;
    double dx = goal_.location.x - odo_.position.x;
    double dy = goal_.location.y - odo_.position.y;
//    double dz = goal_.location.z - odo_.position.z;

//    return (pow(pow(dx,2)+pow(dy,2)+pow(dz,2),0.5) < tolerance_);
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
// Run function implementation: check goals_raw, move to goal, update distance/time
//void Controller::run(void){
//    // Get current goal
//    if(goals_raw.empty()){
//        std::cout << "No goals to reach" << std::endl;
//        return;
//    }
//
//    // Move to goal
//    while(!goals_raw.empty()){
//        // Get current goal
//        goal_ = goals_raw.front();
//        goals_raw.erase(goals_raw.begin());
//        // Move to goal
//        std::cout << "Moving to goal: " << goal_.location.x << " " << goal_.location.y << " " << goal_.location.z << std::endl;
//        while(!goalReached())
//        {
//            pfmsConnectorPtr_->write(goal_,cmd_pipe_seq_);
//            odo_ = getOdometry();
//            distance_travelled_ += goal_.distance;
//            time_travelled_ += goal_.time;
//        }
//    }
//}

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

