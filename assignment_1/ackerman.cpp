#include <cmath>
#include <chrono>
#include <thread>

#include "pfmshog.h"
#include "pfmsconnector.h" 
#include "pfms_types.h"

#include "audi.h"
#include "ackerman.h"


//constructing a new Ackerman object and assign values to the object's members

Ackerman::Ackerman() { 

    
    ROS_communication_ = std::make_shared<PfmsConnector>(); 
    platform_type_ = pfms::PlatformType::ACKERMAN; 
    
    max_throttle_ = 0.1;
    max_brake_force_ = 8000.0; 

    Audi audi_ = Audi(); 
    
}

// Attempts to set a new goal position. Returns true if successful.
bool Ackerman::setGoal(pfms::geometry_msgs::Point goal) { 

    bool readOdometry = ROS_communication_->read(current_platform_odometry_, platform_type_); // Read the current odometry from akerman platform.
    // the methord check is the the goal is out side the turning radius from the current position of the achman platform.
    // If it is it means it is reachable and it update the goal 
    if(audi_.checkOriginToDestination(current_platform_odometry_, goal, distance_platform_to_goal_, estimated_time_to_goal_, estimated_platform_goal_pose_)){ 
        target_goal_ = goal; 
    }
    // Returns if the goal is reachable
    return audi_.checkOriginToDestination(current_platform_odometry_, goal, distance_platform_to_goal_, estimated_time_to_goal_, estimated_platform_goal_pose_); 

}

//Driving to the goal

bool Ackerman::reachGoal(void) {
    
    bool readOdometry = ROS_communication_->read(current_platform_odometry_, platform_type_);
    bool currentSteering = audi_.computeSteering(current_platform_odometry_, target_goal_, Steering_angle_, distance_platform_to_goal_);
    audi_.checkOriginToDestination(current_platform_odometry_, target_goal_, distance_platform_to_goal_, estimated_time_to_goal_, estimated_platform_goal_pose_);

    distance_travelled_by_platform_ += distance_platform_to_goal_;

        bool approachingGoal = false; // boolean check if car is near  goal.
        bool arrivedAtGoal = false; // boolean check if car has reached goal in the set tolerance.
        bool stopped = false; // boolean check if car has stopped.
        double distanceToGoal = 0.0; // distance need to travel to the goal.

        unsigned long count = 1;
        auto start = std::chrono::high_resolution_clock::now(); // tracking drive time

    // Approching the goal:
    // A while loop runs until the vehicle is approaching the goal. Inside this loop:
    // It sends a drive command to the vehicle using the ROS_communication_->send(command_) function.
    // Reads the current odometry data.
    // Calculates the distance to the goal.
    // If the distance is less than a threshold (2.0) m, it sets approachingGoal to true.

    while (!approachingGoal) {
        
        command_ = {count, 0.0, Steering_angle_, max_throttle_};
        ROS_communication_->send(command_);
        count++; 
        readOdometry = ROS_communication_->read(current_platform_odometry_, platform_type_);
        distanceToGoal = std::sqrt(std::pow(target_goal_.x - current_platform_odometry_.position.x, 2) + std::pow(target_goal_.y - current_platform_odometry_.position.y, 2));
            
            if (distanceToGoal < 2.0) {
                
                approachingGoal = true; 
            }
    }

    count = 1;

    // Arriving at Goal:
    // while loop runs until the vehicle has arrived at the goal. 
    // Inside this loop:
    // It sends a braking command to the vehicle.
    // Reads the current odometry data.
    // Checks if the distance to the goal is within a tolerance value.
    // If so, it sets arrivedAtGoal to true.
    
    while (!arrivedAtGoal) {
        
        command_ = {count, max_brake_force_ / 20, Steering_angle_, 0.0};
        ROS_communication_->send(command_);
        count++;
        readOdometry = ROS_communication_->read(current_platform_odometry_, platform_type_);
        distanceToGoal = std::sqrt(std::pow(target_goal_.x - current_platform_odometry_.position.x, 2) + std::pow(target_goal_.y - current_platform_odometry_.position.y, 2));
        
            if (distanceToGoal < tolerance_value_) { 
                
                arrivedAtGoal = true;
            }
    }

    count = 1;

    //Stopping the Vehicle:
    // this while loop runs until the vehicle has stopped. Inside this loop:
    //It sends a stronger braking command.
    //Reads the current velocity from odometry data.
    //If the velocity is very low (less than 0.1), it sets stopped to true.


    double velocity = 0.0;

    while (!stopped) {
        command_ = {count, max_brake_force_, Steering_angle_, 0.0};
        ROS_communication_->send(command_);
        count++;
        readOdometry = ROS_communication_->read(current_platform_odometry_, platform_type_);
        velocity = std::sqrt(std::pow(current_platform_odometry_.linear.x, 2) + std::pow(current_platform_odometry_.linear.y, 2));
        
            if (velocity < 0.01) { 
            
                
                stopped = true;
            }
    }

    //finaly:
    // Calculates the distance to the goal again.
    // If the distance is still within the tolerance, it returns true (indicating successful goal reach) and updates the total time in motion.
    // Otherwise, it returns false.

    distanceToGoal = std::sqrt(std::pow(target_goal_.x - current_platform_odometry_.position.x, 2) + std::pow(target_goal_.y - current_platform_odometry_.position.y, 2));
    
    if (distanceToGoal <= tolerance_value_) { 

        //track time taken to drive to the goal
       
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        double elapsed_time_seconds = elapsed.count();
        time_in_motion_by_platform_ += elapsed_time_seconds; 
        
        return true;

    } 
    
    else {
        
        return false;
    }
}


double Ackerman::distanceToGoal() { 

    audi_.checkOriginToDestination(current_platform_odometry_, target_goal_, distance_platform_to_goal_, estimated_time_to_goal_, estimated_platform_goal_pose_); 
    
        return distance_platform_to_goal_;

}

double Ackerman::timeToGoal(void) { 

    audi_.checkOriginToDestination(current_platform_odometry_, target_goal_, distance_platform_to_goal_, estimated_time_to_goal_, estimated_platform_goal_pose_); 
    
        return estimated_time_to_goal_;

}

bool Ackerman::setTolerance(double tolerance) { 

    if (tolerance < 0) {
    
        return false;
    }
    
    tolerance_value_ = tolerance;
    
        return true;

}

double Ackerman::platformDistanceTravelled(void) { 

    return distance_travelled_by_platform_;

}

double Ackerman::timeInMotion(void) {

    return time_in_motion_by_platform_;

}

pfms::nav_msgs::Odometry Ackerman::getOdometry(void) {
    
    bool readOdometry = ROS_communication_->read(current_platform_odometry_, platform_type_);
    
        return current_platform_odometry_;

}