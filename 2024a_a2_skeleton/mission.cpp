#include "mission.h"
#include "controller.h"

Mission::Mission(std::vector<ControllerInterface*> controllers) {
    controllers_ = controllers;
}

void Mission::setGoals(std::vector<pfms::geometry_msgs::Point> goals, pfms::PlatformType platform) {
    for (auto controller : controllers_) {
        auto type = controller->getPlatformType();
        if (type == platform){
            controller->setGoals(goals);
            if(platform==pfms::PlatformType::ACKERMAN) std::cout << "Setting goals for ACKERMAN" << std::endl;
            if(platform==pfms::PlatformType::QUADCOPTER) std::cout << "Setting goals for QUADCOPTER" << std::endl;
        }
    }
}

bool Mission::run() {
    std::cout << "Mission::Run dummy" << std::endl;
    for (auto controller : controllers_)
        controller->run();
    std::cout << "CHECK THREADING" << std::endl;
    return true;
}

std::vector<unsigned int> Mission::status(void) {
    std::vector<unsigned int> _status;
    for (auto interface : controllers_)
    {
        // convert controller interface to controller
        Controller * c = dynamic_cast<Controller*>(interface);
        _status.push_back(c->checkGoalsProgress());
    }
    return _status;
}

void Mission::setMissionObjective(mission::Objective objective) {
    std::cout << "Mission::setMissionObjective dummy" << std::endl;
}

std::vector<double> Mission::getDistanceTravelled() {
    std::cout << "Mission::getDistanceTravelled dummy" << std::endl;
    std::vector<double> distanceTravelled;
    return distanceTravelled;
}

std::vector<double> Mission::getTimeMoving() {
    std::cout << "Mission::getTimeMoving dummy" << std::endl;
    std::vector<double> timeMoving;
    return timeMoving;
}

std::vector<std::pair<int, int>> Mission::getPlatformGoalAssociation() {
    std::cout << "Mission::getPlatformGoalAssociation dummy" << std::endl;
    std::vector<std::pair<int, int>> platformGoalAssociation;
    return platformGoalAssociation;
}