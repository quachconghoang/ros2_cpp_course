#include "controller.h" // Include the header file for the Controller class.
#include "audi.h" // Include the header file for the Audi class.

// Constructor for the Controller class.
Controller::Controller() {

    // Create an Audi instance within the constructor.
    Audi audi_ = Audi();

}

// Destructor for the Controller class. It's currently empty because there's no dynamic allocation to clean up.
Controller::~Controller() {}

// Method to check the feasibility of reaching a destination from a given origin.
// This method also calculates the distance and time to the destination, and estimates the goal pose.
bool Controller::checkOriginToDestination(pfms::nav_msgs::Odometry origin, // Current origin position and orientation.
                                    pfms::geometry_msgs::Point goal, // Goal position.
                                    double& distance, // Output parameter for distance to the goal.
                                    double& time, // Output parameter for estimated time to the goal.
                                    pfms::nav_msgs::Odometry& estimatedGoalPose) {  // Output parameter for estimated goal pose.

    // Calls the checkOriginToDestination method of the audi_ member to perform the actual calculation.
    return audi_.checkOriginToDestination(currentOdometry_, goal_, distanceToGoal_, timeToGoal_, estimatedGoalPose_);
}

// Method to retrieve the platform type.
pfms::PlatformType Controller::getPlatformType(void) { //Define getPlatformType function

    // Returns the type of platform.
    return type_;

}

// Method to retrieve the platform status.
pfms::PlatformStatus Controller::status(void){ //Define getPlatformType

    // Returns the type of platform.
    return status_;

}
