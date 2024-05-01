#include "mission.h"
#include "ackerman.h"
#include "skidsteer.h"
#include <vector>
#include "controller.h"

// this is a constructor for Mission class.
Mission::Mission(std::vector<ControllerInterface*> controllers) {
    // make a platform_controllers_ member variable contianing controllers vector.
    platform_controllers_ = controllers;
}

// Method to set the goals for the mission.
void Mission::setGoals(std::vector<pfms::geometry_msgs::Point> goals) {
    // make a mission_goals_ member variable containing goals vector.
    mission_goals_ = goals;
}

// Start the mission
bool Mission::runMission() {
    // default the mission complete to false has it has just started
    bool mission_complete = false;
    int count = 0; // a counter for reached goals.


    for(int i = 0; i < mission_goals_.size(); i++){
        // Check the type of platform and set the goal accordingly.
        //mission_goals_ is vector containing each 'Point' the car must drive too. 'Point' is an cobject contians x,y,z co-ordinates
        if(platform_ID_.at(i) == 0){
            //If the platform_ID_ at index i is 0, it sets the goal for the akerman platform (platform_controllers_[0]), sets a tolerance of 0.5, 
            //and checks if the platform reaches the goal within the tolerance. If so, it increments count.
            platform_controllers_[0]->setGoal(mission_goals_[i]);
            platform_controllers_[0]->setTolerance(0.5);
            bool reachedCurrentGoal = platform_controllers_[0]->reachGoal();
            if(reachedCurrentGoal){ //initializes an integer variable count to keep track of the number of reached goals within the tolerance. 
                count++;
            }
        }
        else if(platform_ID_[i] == 1){
            // If the platform_ID_ at index i is 1, it sets the goal for the skidsteer platform (platform_controllers_[1]), sets a tolerance of 0.5, and checks if the platform reaches the goal within the tolerance. If so, it increments count.
            platform_controllers_[1]->setGoal(mission_goals_[i]);
            platform_controllers_[1]->setTolerance(0.5);
            bool reachedCurrentGoal = platform_controllers_[1]->reachGoal();
            if(reachedCurrentGoal){ //If reachedCurrentGoal is true for reaching goal it increments count.
                count++;
            }
        }
    }

    // After processing all goals, it checks if count equals the total number of mission goals. 
    //If so, it sets mission_complete to true.
    if(count == mission_goals_.size()){
        mission_complete = true;
    }
    return mission_complete; // Return the value mission completion status.
}

// this method sets up the mission objective.
void Mission::setMissionObjective(mission::Objective objective) {
    //saving it in a new member variable mission_objective_.
    mission_objective_ = objective;
}

// The purpose of this method is to calculate and return a vector of distances traveled by different platforms in a mission.
std::vector<double> Mission::getDistanceTravelled() { //The method initializes an empty std::vector<double> called platformDistanceTravelled to use as a container.
    std::vector<double> platformDistanceTravelled;

    //It iterates over the platform_ID_ vector and retrieves the distance traveled by calling the platformDistanceTravelled() method on the corresponding platform_controllers_[i].
    //The calculated distance is then added to the platformDistanceTravelled vector.

    for(int i = 0; i < platform_ID_.size(); i++){ //
        platformDistanceTravelled.push_back(platform_controllers_[i]->platformDistanceTravelled());
    }
    return platformDistanceTravelled; // Return the populated vector
}

// The purpose of this method is to calculate and return a vector of time spent in motion for each platform in a mission
std::vector<double> Mission::getTimeMoving() {
    
    // create a vector container to store moving times for each platform as a double
    std::vector<double> platform_moving_times;

    // iterates over each platform in platform_ID and put their moving time in vector 'platform_moving_times'

    for(int i = 0; i < platform_ID_.size(); i++){
        platform_moving_times.push_back(platform_controllers_[i]->timeInMotion());
    }
    return platform_moving_times; // Returns collected times.
}

// The purpose of this method is to determine the association between platform types and their corresponding goals.
std::vector<unsigned int> Mission::getPlatformGoalAssociation() {
    std::vector<unsigned int> platforms;

    // Iterate through goals in mission_goals_ and controllers in platform_controllers_ to associate platforms with goals.
    for (int i = 0; i < mission_goals_.size(); i++) {
        for(int j = 0; j < platform_controllers_.size(); j++){
            if(platform_controllers_[j]->getPlatformType() == pfms::PlatformType::ACKERMAN){
                platform_ID_.push_back(0); // 0 means Ackerman.
            }
            else if (platform_controllers_[j]->getPlatformType() == pfms::PlatformType::SKIDSTEER){
                platform_ID_.push_back(1); // 1 means SkidSteer.
            }
        }
    }

    return platform_ID_; // Return the platform-goal associations.
}