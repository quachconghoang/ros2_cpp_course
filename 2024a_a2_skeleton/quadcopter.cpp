#include "quadcopter.h"

Quadcopter::Quadcopter():Controller()
{
    // Default constructor should set all sensor attributes to a default value
}

bool Quadcopter::setGoals(std::vector<pfms::geometry_msgs::Point> goals) {
    // Implement this function
    std::cout<<"Quadcopter::setGoals"<<std::endl;
    return true;
}

void Quadcopter::run(void) {
    // Implement this function
    std::cout<<"Quadcopter::run"<<std::endl;
}