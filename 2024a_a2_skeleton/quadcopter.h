#ifndef QUADCOPTER_H
#define QUADCOPTER_H

#include "controller.h"

class Quadcopter: public Controller
{
public:
      //Default constructor - should set all sensor attributes to a default value
      Quadcopter();
      ~Quadcopter();

    bool checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                  pfms::geometry_msgs::Point goal,
                                  double& distance, double& time,
                                  pfms::nav_msgs::Odometry& estimatedGoalPose);
protected:
    bool reachGoal(void);
    bool calcNewGoal(void);
private:
    /// \brief timeLapsed
    /// \param start_time - the strat time from which time is lapsed
    /// \return time in seconds
    double timeLapsed(std::chrono::time_point<std::chrono::system_clock> start_time);
    void sendCmd(double turn_l_r, double move_l_r, double move_u_d, double move_f_b);
    //! Angle required for quadcopter to have a straight shot at the goal
    double target_angle_ = 0;
    double TARGET_SPEED;
};

#endif // QUADCOPTER_H
