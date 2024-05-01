#ifndef SKIDSTEER_H
#define SKIDSTEER_H

#include "controller.h" // Include the base controller class.
#include "pfmsconnector.h" // Include for platform management system connector.
#include "pfms_types.h" // Include for platform management system types.

/*!
 *  \brief     Skidsteer Class
 *  \details
 *  The SkidSteer class inherits from base classe Controller. 
 *  its responible for providing control over the characteristics of platforms that use skidsteer steering such as the husky. 
 *  It's methods inclue  reaching goals, estimating the time and distance to goals, and reading vehicle odometry (pose, velocity etc.).
 *  It derives its atrubutes from Controller,
 *  \author    wil
 *  \date      2023-03-23
 */


class SkidSteer : public Controller {
public:
    SkidSteer();

    // Methods specific to the SkidSteer class:

    /**
     * @brief Accepts the container of goals for the skid-steer to drive too.
     * @param goal The navigation goal point.
     * @return True if successful, false otherwise.
     */
    bool setGoal(pfms::geometry_msgs::Point goal);

     /**
     Getter for distance to be travelled to reach goal, updates at the platform moves to current goal
        @return distance to be travlled to goal [m]
    */
    double distanceToGoal(void);

    /**
        Reach reach goal - execute control to reach goal, blocking call until goal reached or abandoned
        @return goal reached (true - goal reached, false - goal abandoned : not reached)
    */
    bool reachGoal(void);

    /**
    Getter for time to reach goal, updates at the platform moves to current goal
    @return time to travel to SkidSteerGoal [s]
     */
    double timeToGoal(void);

    /**
    Set tolerance when reaching goal
    @return tolerance accepted [m]
    */
    bool setTolerance(double tolerance);

    /**
    returns total time in motion by platform, time when stationary not included
    @return total time in motion since started [s]
    */
    double timeInMotion(void);

    /**
    returns total distance travelled by platform
    @return total distance travelled since started [m]
    */
    double platformDistanceTravelled(void);


    /**
     * @brief Retrieves the current odometry of the skidsteer husky.
     * @return The current odometry.
     */
    pfms::nav_msgs::Odometry getOdometry(void);

private:
    // skid steer movment attributes

    double rotate_right_; // turn rate right (rad/s).
    double rotate_left_; // turn rate left (rad/s).
    double move_foward_; // velocity forward (m/s).
    double move_reverse_; // velocity reverse (m/s).

    //  structure for skidsteer commands.
    pfms::commands::SkidSteer command_;
};

#endif // SKIDSTEER_H