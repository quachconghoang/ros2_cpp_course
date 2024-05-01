#include "quadcopter.h"
#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>

Quadcopter::Quadcopter() {
    TARGET_SPEED = 0.4;
    tolerance_ = 0.5;
    type_ = pfms::PlatformType::QUADCOPTER;
}

// We delete the pipes here specifically, which forces them to close before the object is terminated
//Quadcopter::~Quadcopter(){
//    pfmsConnectorPtr_.reset();
//}



bool Quadcopter::checkOriginToDestination(pfms::nav_msgs::Odometry origin, pfms::geometry_msgs::Point goal,
                                          double& distance, double& time,
                                          pfms::nav_msgs::Odometry& estimatedGoalPose) {

    // Use pythagorean theorem to get direct distance to goal
    double dx = goal.x - origin.position.x;
    double dy = goal.y - origin.position.y;

    distance = std::hypot(dx, dy);
    time = distance / TARGET_SPEED;

    // The estimated goal pose would be the goal, at the angle we had at the origin
    // as we are not rotating the platform, simple moving it left/right and fwd/backward
    estimatedGoalPose.position.x = goal.x;
    estimatedGoalPose.position.y = goal.y;
    estimatedGoalPose.yaw = origin.yaw;
    estimatedGoalPose.linear.x = 0;
    estimatedGoalPose.linear.y = 0;

    return true;
}

bool Quadcopter::calcNewGoal(void) {

    getOdometry();//This will update internal copy of odometry, as well as return value if needed.

    pfms::nav_msgs::Odometry est_final_pos;

    if (!checkOriginToDestination(odo_, goal_.location, goal_.distance, goal_.time, est_final_pos))
        return false;

    // Calculate absolute travel angle required to reach goal
    double dx = goal_.location.x - odo_.position.x;
    double dy = goal_.location.y - odo_.position.y;
    target_angle_ = std::atan2(dy, dx);

    return true;
}

void Quadcopter::sendCmd(double turn_l_r, double move_l_r, double move_u_d, double move_f_b) {
    pfms::commands::Quadcopter cmd = {
            cmd_pipe_seq_++,
            turn_l_r,
            move_l_r,
            move_u_d,
            move_f_b,
    };
    pfmsConnectorPtr_->send(cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));//Small delay to ensure message sent
}

bool Quadcopter::reachGoal(void) {
    // this rasised the quadcoppter altertude to 1.5 m if it is below 1 m
    getOdometry();                      //checks Quadcopters current odometery information
    if(odo_.position.z < 1){            // if the Quadcopter's current alterdude is less than 1 m...
        while (odo_.position.z < (goal_.location.z - tolerance_))
        {
            getOdometry();              //checks Quadcopters odometery again to update the altiude to see if its under 1.5m..
            sendCmd(0, 0, 0.4, 0);      // send command to the drown to accellerate 0.4 m/s in the z direction
            std::this_thread::sleep_for(std::chrono::microseconds(50));  //delay the while loop iteration for 50 m/s to aloow the drone to respond
        }

    }
    sendCmd(0, 0, 0, 0);  // resetting drone speed to 0 in all directions

    calcNewGoal(); // account for any drift between setGoal call and now, by getting odo and angle to drive in
    pfms::nav_msgs::Odometry prev_odo = odo_;   //stores the current odometry of the quadcopter in variable "prev_odo"
    auto start_time = std::chrono::system_clock::now(); //stores current system time in "start_time" to time the quadcopters run towards the goal
    double estimated_time_to_reach_goal = goal_.time; //retrives the estimated time to goal from "goal_" and stores it in variable "estimated_time_to_reach_goal"
    // the current time coukd be compeqared to the estimated to to assist in the when the quadcoter should be slowing down when aproching the goal

    double decelerate_zone = 0.5; // declare varible decelerate_zone with  a value of 0.5 m
    double decelerate_scale; // decalre vaiarable scale unassigned for that rate at witch to slow down

    //Run below loop until we reach goal
    while (!goalReached()) {
        //Calculate distance to goal
        // calculates the distance between the quadcopter’s current x and y oordinates and the goal's x and y coordinates..
        //then stores the result in variable's dx and dy repectivly
        double dx = goal_.location.x - odo_.position.x;
        double dy = goal_.location.y - odo_.position.y;

        //get current orentation
        double yaw = odo_.yaw; // assigns to "yaw" the direction the quadcoppter is facing in RAD

        //calculate local coordiates of goal from perspective of quadcopter
        //Rotate the difference vector by the negative of the drone’s current yaw angle. This will transform the global coordinates into the drone’s local coordinate system.
        //yaw must be neagative as the coordiate system assumse anti-clockwise increase
        double local_x = dx * cos(-yaw) - dy * sin(-yaw);
        double local_y = dx * sin(-yaw) + dy * cos(-yaw);

        double local_dist = sqrt(pow(local_x , 2) + pow(local_y, 2));

        double globle_dist = sqrt(pow(dx, 2)) + pow(dy, 2);

        if(globle_dist < decelerate_zone)
        {
            decelerate_scale = 0.5;
        }
        else
        {
            decelerate_scale = 1;
        }

        //Normalise the componects to get a unit vector
        local_x = local_x / local_dist * TARGET_SPEED * decelerate_scale;
        local_y = local_y / local_dist * TARGET_SPEED * decelerate_scale;


        // Check the syntax of the command, for left/right and forward/backward
        sendCmd(0, local_y, 0, local_x);

        /////////////////////////////////////////////////////////////////


        // We check if we have not reached goal in close to anticipated time (2s margin)
        // if not reachGoal should be aborted
        //double time_since_starting = timeLapsed(start_time);

        // std::cout << "[est/cur] time to goal [" << estimated_time_to_reach_goal << "/" << time_since_starting << "]" << std::endl;


        //if(time_since_starting>(estimated_time_to_reach_goal+2.0)){
        // If we have not reached it in the designated time we abandon reaching goal
        // For TASK 3 and 4, consider if anything else needs updating
        // return false;
        // }

        if (timeLapsed(start_time) > (estimated_time_to_reach_goal + 12.0))
        {
            time_travelled_ += timeLapsed(start_time);
            sendCmd(0, 0, 0, 0);
            return false;
        }


        calcNewGoal(); //get odometry and update target angle of control

        double distance_increment = std::hypot(odo_.position.x - prev_odo.position.x, odo_.position.y - prev_odo.position.y);
        distance_travelled_ += distance_increment;
        prev_odo = odo_;

        ///////////////////////////////////////////////////////////////
        //! @todo
        //! TASK 4 - Update distance travelled
        //!
        //! While the estimated distance at the begining is a rough guide
        //! As the platform moves it could travel more, especially if it
        //! was going against wind (and in case of Ackerman if it was drifting)
        //! Your better of incremeting distance travelled as you go along
        //!
        //! We have access to following variables
        //! odo_ - current position of platform
        //! prev_odo - the previous position
        //! distance_travelled_ - the distance travelled thus far


        //////////////////////////////////////////////////////////////////
    }

    // Stop thq quadcopter immediately
    sendCmd(0, 0, 0, 0);

    time_travelled_ += timeLapsed(start_time);
    return true;


    calcNewGoal(); //get odometry and update distance to goal

    ///////////////////////////////////////////////////////////////
    //! @todo
    //! TASK 3 - Update time travelled
    //!
    //! We have access following variables
    //! start_time - the time reaching the goal was started
    //! time_travelled_ - the time we have travelled thus far
    //!
    //! Look at how we compute wether to abort reachGoal
    //! as you could use the same function
    //! Also, consider, if we aborted the function reachGoal, should we
    //! update the time travelled?

    //////////////////////////////////////////////////////////////////

    return true;
}


double Quadcopter::timeLapsed(std::chrono::time_point<std::chrono::system_clock> start_time){
    // Update time taken
    auto finish_time = std::chrono::system_clock::now();
    //std::chrono::seconds is integer for some reason, thus duration<double>
    auto time_taken = std::chrono::duration_cast<std::chrono::duration<double>>(finish_time - start_time);
    return time_taken.count();
}