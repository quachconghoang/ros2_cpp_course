#include "ackerman.h"

Ackerman::Ackerman() {
    // Default constructor should set all sensor attributes to a default value
    max_throttle_ = 0.1;
    max_brake_force_ = 8000.0;
    tolerance_ = 0.5;
    type_ = pfms::PlatformType::ACKERMAN;
    audi_ = new Audi();
}

//void Ackerman::run(void) {
//    // Implement this function
//    std::cout<<"Ackerman::run"<<std::endl;
//}

bool Ackerman::reachGoal(void) {
    getOdometry();
    audi_->computeSteering(odo_,goal_.location,
                           Steering_angle_, goal_.distance);
    audi_->checkOriginToDestination(odo_, goal_.location,
                                    goal_.distance, goal_.time, goal_.estimated_pose);

//    distance_travelled_by_platform_ += distance_platform_to_goal_;
    bool approachingGoal = false; // boolean check if car is near  goal.
    bool arrivedAtGoal = false; // boolean check if car has reached goal in the set tolerance.
    bool stopped = false; // boolean check if car has stopped.
    double distanceToGoal = 0.0; // distance need to travel to the goal.

    unsigned long count = 1;
    auto start = std::chrono::high_resolution_clock::now(); // tracking drive time

    while (!approachingGoal) {
        command_ = {count, 0.0, Steering_angle_, max_throttle_};
        pfmsConnectorPtr_->send(command_);
        count++;
        getOdometry();
        distanceToGoal = std::sqrt(std::pow(goal_.location.x - odo_.position.x, 2) + std::pow(goal_.location.y - odo_.position.y, 2));

        if (distanceToGoal < 2.0) {
            approachingGoal = true;
        }
    }

    count = 1;

    while (!arrivedAtGoal) {
        command_ = {count, max_brake_force_ / 20, Steering_angle_, 0.0};
        pfmsConnectorPtr_->send(command_);
        count++;
        getOdometry();
        distanceToGoal = std::sqrt(std::pow(goal_.location.x - odo_.position.x, 2) +
                std::pow(goal_.location.y - odo_.position.y, 2));

        if (distanceToGoal < tolerance_) {
            arrivedAtGoal = true;
        }
    }

    count = 1;
    double velocity = 0.0;

    while (!stopped) {
        command_ = {count, max_brake_force_, Steering_angle_, 0.0};
        pfmsConnectorPtr_->send(command_);
        count++;
        getOdometry();
        velocity = std::sqrt(std::pow(odo_.linear.x, 2) + std::pow(odo_.linear.y, 2));

        if (velocity < 0.01) {
            stopped = true;
        }
    }

    //finaly:
    // Calculates the distance to the goal again.
    // If the distance is still within the tolerance, it returns true (indicating successful goal reach) and updates the total time in motion.
    // Otherwise, it returns false.

    distanceToGoal = std::sqrt(std::pow(goal_.location.x - odo_.position.x, 2) +
            std::pow(goal_.location.y - odo_.position.y, 2));

    if (distanceToGoal <= tolerance_) {
        //track time taken to drive to the goal
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        double elapsed_time_seconds = elapsed.count();
//        time_in_motion_by_platform_ += elapsed_time_seconds;
        return true;
    }
    return true;
}

bool Ackerman::calcNewGoal() {
    // Implement this function
    std::cout<<"Ackerman::calcNewGoal"<<std::endl;
    return true;
}

bool Ackerman::checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                          pfms::geometry_msgs::Point goal,
                                          double& distance,
                                          double& time,
                                          pfms::nav_msgs::Odometry& estimatedGoalPose) {

    return audi_->checkOriginToDestination(odo_,goal_.location,
                                           goal_.distance,goal_.time,
                                           goal_.estimated_pose);
}
