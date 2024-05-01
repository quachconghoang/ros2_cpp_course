#include "ackerman.h"
#include <cmath>
#include "pfmsconnector.h" 
#include "audi.h"
#include "pfms_types.h"
#include "pfmshog.h"
#include <chrono>
#include <thread>

Ackerman::Ackerman() { // Constructor for the Ackerman class.

    type_ = pfms::PlatformType::ACKERMAN; // Initialize the platform type to Ackerman.
    pfmsConnectorPtr_ = std::make_shared<PfmsConnector>(); // Create a new PfmsConnector instance for platform communication.
    max_brake_torque_ = 8000.0; // Set the maximum braking torque in Newton-meters.
    default_throttle_ = 0.1; // Set the default throttle value, where 0.1 corresponds to a top speed of 2.91 m/s.
    Audi audi_ = Audi(); // Instantiate an Audi class object for vehicle operations.

}

// Attempts to set a new goal position. Returns true if successful.
bool Ackerman::setGoals(std::vector<pfms::geometry_msgs::Point> goals) { 

    bool throwAway = pfmsConnectorPtr_->read(currentOdometry_, type_); // Read the current odometry from the Audi vehicle.
    // Check if the goal is reachable from the current position.
    // If so, update the goal and related parameters.
    if(audi_.checkOriginToDestination(currentOdometry_, goal_, distanceToGoal_, timeToGoal_, estimatedGoalPose_)){ 
        goal_ = goal_; 
    }
    // Return true if the goal is reachable, false otherwise.
    return audi_.checkOriginToDestination(currentOdometry_, goal_, distanceToGoal_, timeToGoal_, estimatedGoalPose_); 

}

// Method to attempt reaching the set goal. Returns true if the goal is reached.
void Ackerman::run() {

    // Initial setup for reaching the goal, including reading current odometry and calculating steering.
    bool throwAway = pfmsConnectorPtr_->read(currentOdometry_, type_); 
    bool isSteer = audi_.computeSteering(currentOdometry_, goal_, steering_, distanceToGoal_); 
    // Check and compute the time and distance to the goal, updating relevant variables.
    audi_.checkOriginToDestination(currentOdometry_, goal_, distanceToGoal_, timeToGoal_, estimatedGoalPose_); 
    // Update the total distance travelled by adding the distance to the current goal.
    distanceTravelled_ = distanceTravelled_ + distanceToGoal_; 

    // Variables for controlling the movement towards the goal.
    unsigned long count = 1; // Used to count the number of commands sent.
    double distanceToGoal = 0.0; // Stores the dynamically computed distance to the goal.
    bool nearGoal = false; // Indicates if the vehicle is near the goal.
    bool reachedGoal = false; // Indicates if the goal has been reached within the set tolerance.
    bool stopped = false; // Indicates if the vehicle has stopped.
    auto start = std::chrono::high_resolution_clock::now(); // Start time for reaching the goal.

    // Loop to drive the vehicle near the goal.
    while(!nearGoal){ 
        // Construct a drive command with the calculated steering angle and default throttle.
        cmd_ = {count, 0.0, steering_, default_throttle_};
        pfmsConnectorPtr_->send(cmd_); // Send the drive command.
        count++; // Increment the command sequence number.
        // Read current odometry to update the distance to the goal.
        bool throwAway = pfmsConnectorPtr_->read(currentOdometry_, type_); 
        distanceToGoal = std::sqrt(std::pow(goal_.x - currentOdometry_.position.x, 2) + std::pow(goal_.y - currentOdometry_.position.y, 2)); 
        if(distanceToGoal < 0.55){ 
            nearGoal = true; // Check if near the goal.
        }
    }

    count = 1; // Reset the sequence number.

    // Loop to check if the vehicle has reached the goal.
    while(!reachedGoal){ 
        // Construct a brake command to slow down as the vehicle approaches the goal.
        cmd_ = {count, max_brake_torque_/2, steering_, 0.0}; 
        pfmsConnectorPtr_->send(cmd_);
        count++;
        bool throwAway = pfmsConnectorPtr_->read(currentOdometry_, type_);
        distanceToGoal = std::sqrt(std::pow(goal_.x - currentOdometry_.position.x, 2) + std::pow(goal_.y - currentOdometry_.position.y, 2));
        if(distanceToGoal < tolerance_){ // Check if within the goal tolerance.
            reachedGoal = true;
        }
    }

    count = 1;
    double velocity = 0.0; // Create variable for checking if velocity is zero.

    // Loop to ensure the vehicle has fully stopped.
    while(!stopped){ 
        // Construct a full brake command.
        cmd_ = {count, max_brake_torque_, steering_, 0.0}; 
        pfmsConnectorPtr_->send(cmd_);
        count++;
        bool throwAway = pfmsConnectorPtr_->read(currentOdometry_, type_);
        velocity = std::sqrt(std::pow(currentOdometry_.linear.x, 2) + std::pow(currentOdometry_.linear.y, 2)); 
        if(velocity < 1e-1){ // Check if the vehicle has stopped.
            stopped = true;
        }
    }

    // Additional braking to ensure the vehicle is completely stopped.
    for(int i = 0; i < 100; i++){ 
        cmd_ = {count, max_brake_torque_, steering_, 0.0};
        pfmsConnectorPtr_->send(cmd_);
        count++;
    }

    // Final check to confirm the goal was reached within tolerance.
    distanceToGoal = std::sqrt(std::pow(goal_.x - currentOdometry_.position.x, 2) + std::pow(goal_.y - currentOdometry_.position.y, 2));
    if(distanceToGoal <= tolerance_){ // check distance to goal is still within tolerance if so return true otherwise return false
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        double elapsedSeconds = elapsed.count();
        timeInMotion_ = timeInMotion_ + elapsedSeconds; // Update the total time in motion.
        //return true; // Goal reached successfully.
    }
    else{
        //return false; // Goal not reached.

    }
    
}

// Method to get the current distance to the goal.
double Ackerman::distanceToGoal() { 

    audi_.checkOriginToDestination(currentOdometry_, goal_, distanceToGoal_, timeToGoal_, estimatedGoalPose_); 
    return distanceToGoal_;

}

// Method to get the estimated time to reach the goal.
double Ackerman::timeToGoal(void) { 

    audi_.checkOriginToDestination(currentOdometry_, goal_, distanceToGoal_, timeToGoal_, estimatedGoalPose_); 
    return timeToGoal_;

}

// Method to set a tolerance value for reaching the goal. Returns true if successful.
bool Ackerman::setTolerance(double tolerance) { 

    if (tolerance < 0) {
    return false;
    }
    tolerance_ = tolerance;
    return true;

}

// Method to get overall distance travelled by the platform.
double Ackerman::distanceTravelled(void) { 

    return distanceTravelled_;

}

// Method to get overall time in motion for the platform.
double Ackerman::timeTravelled(void) {

    
    return timeInMotion_;

}

// Method to get current Odometry for the platform.
pfms::nav_msgs::Odometry Ackerman::getOdometry(void) {
    
    bool throwaway = pfmsConnectorPtr_->read(currentOdometry_, type_);
    return currentOdometry_;

}


