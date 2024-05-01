#include "controller.h"

Controller::Controller():
        odo_{0,0,0,0,0,0},
        distance_travelled_(0),
        time_travelled_(0),
        cmd_pipe_seq_(0){
    // Set all attributes to default values
    pfmsConnectorPtr_ = std::make_unique<PfmsConnector>();
    goal_.time=0;
    goal_.distance=0;
}

void Controller::run(void) {
    // Implement this function
    std::cout<<"Controller::run"<<std::endl;
    for (auto goal : goals_queue_){
        std::cout<< "Reaching goal: " << goal.x << " " << goal.y << " " << goal.z << std::endl;
        goal_.location = goal;
        bool reachable = true;
//        bool reachable = calcNewGoal();
        if(! reachable){
            std::cout << "Goal not reachable" << std::endl;
            continue;
        }
        goal_.distance = distanceToGoal();
        goal_.time = timeToGoal();

        if(reachable){
            std::cout<< "Reaching ...\n";
            bool reached = reachGoal();
        }
    }
}

pfms::PlatformStatus Controller::status(void) {
    // Implement this function
    std::cout<<"Controller::status"<<std::endl;
    return pfms::PlatformStatus::IDLE;
}

bool Controller::setGoals(std::vector<pfms::geometry_msgs::Point> goals) {
    // Store in Buffer
    goals_queue_.clear();
    for(auto points : goals){
       goals_queue_.push_back(points);
    }
    std::cout << "Set " << goals_queue_.size() << " goals" << std::endl;
    return true;
}

double Controller::distanceToGoal(void) {
    return goal_.distance;

}

double Controller::timeToGoal(void) {
    return goal_.time;
}

// Can Implement in Controller
bool Controller::setTolerance(double tolerance) {
    tolerance_ = tolerance;
    return true;
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


pfms::nav_msgs::Odometry Controller::getOdometry(void) {
    pfmsConnectorPtr_->read(odo_,type_);
    return odo_;
}

pfms::PlatformType Controller::getPlatformType(void){
    return type_;
}


// Can Implement in Controller
std::vector<pfms::geometry_msgs::Point> Controller::getObstacles() {
    std::cout << "Controller::getObstacles" << std::endl;
    return std::vector<pfms::geometry_msgs::Point>();
}

// Path: missioninterface.h
