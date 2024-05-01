#include "controller.h"

using std::cout;
using std::endl;

Controller::Controller():
        odo_{0,0,0,0,0,0},
        distance_travelled_(0),
        time_travelled_(0),
        cmd_pipe_seq_(0){
    // Set all attributes to default values
    pfmsConnectorPtr_ = std::make_unique<PfmsConnector>();
    goal_.time=0;
    goal_.distance=0;
    goal_index_ = 0;
}

Controller::~Controller() {
    pfmsConnectorPtr_.reset();
//    running_=false;
//    set_Goals_Thread_->join();
    reach_Goals_Thread_->join();
//    delete set_Goals_Thread_;
    delete reach_Goals_Thread_;

}

void Controller::run(void) {
    // Implement this function
    std::cout<<"Controller::run By thread ?!"<<std::endl;
    reach_Goals_Thread_ = new std::thread(&Controller::reachGoalsQueue, this);
//    reachGoalsQueue();
}

unsigned int Controller::checkGoalsProgress() {
    // Implement this function
    uint progress = uint(double(goal_index_*100)/goals_queue_.size());
    if(progress == 100) {
        std::cout << "Clear Cache" << std::endl;
        std::unique_lock<std::mutex> lock(goals_queue_Mutex_);
        goals_queue_.clear();
        goal_index_ = 0;
        lock.unlock();
        // KILL ME

    }

    return progress;
}

void Controller::reachGoalsQueue() {
    while (goal_index_ < goals_queue_.size()) {
        auto goal = goals_queue_.at(goal_index_);
        std::cout<< "Reaching goal: " << goal.x << " " << goal.y << " " << goal.z << std::endl;
        goal_.location = goal;
        bool reachable = calcNewGoal();
        if(! reachable){
            goal_index_ ++;
            cout << "This goal not reachable, skip, " << checkGoalsProgress()  << "%"  << endl;
            continue;
        }
        goal_.distance = distanceToGoal();
        goal_.time = timeToGoal();
        if(reachable){
            cout<< "Reaching a reachable goal ...\n";
            bool reached = reachGoal();
            if(reached) {
                goal_index_ ++;
                cout<< "Goal reached, " << checkGoalsProgress() << "%" << endl;
            }
        }
    }
}

pfms::PlatformStatus Controller::status(void) {
    // Implement this function
    return pfms::PlatformStatus::IDLE;
}

bool Controller::setGoals(std::vector<pfms::geometry_msgs::Point> goals) {
    // Store in Buffer
    std::unique_lock<std::mutex> lock(goals_queue_Mutex_);
    for(auto points : goals){
       goals_queue_.push_back(points);
    }
    cout << "Add " << goals.size() << " goals, current goal size is " << goals_queue_.size() << endl;
    lock.unlock();
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
