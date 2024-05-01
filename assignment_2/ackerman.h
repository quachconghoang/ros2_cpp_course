#ifndef ACKERMAN_H
#define ACKERMAN_H

#include "controller.h" // Include the base Controller class.

// Ackerman class declaration, inheriting from the Controller class.
class Ackerman : public Controller {
public:
  // Default constructor - initializes the Ackerman controller with default values.
  Ackerman();

  // Method declarations for the Ackerman class.

  // Attempts to set the navigation goal and returns true if the goal is valid.
  bool setGoals(std::vector<pfms::geometry_msgs::Point> goals);

  // Initiates the process to reach the set goal. Returns true if the goal is successfully reached.
  void run (void);

  // Returns the current distance to the goal.
  double distanceToGoal(void);

  // Returns the estimated time required to reach the goal.
  double timeToGoal(void);

  // Sets the tolerance for reaching the goal. Returns true if the tolerance value is valid and set successfully.
  bool setTolerance(double tolerance);

  // Returns the total distance travelled by the Ackerman controller.
  double distanceTravelled(void);

  // Returns the total time in motion towards the goal.
  double timeTravelled(void);

  // Retrieves and returns the current odometry information.
  pfms::nav_msgs::Odometry getOdometry(void);

  std::vector<pfms::geometry_msgs::Point> getObstacles(void);

private:
  // Attributes specific to the Ackerman controller.

  double max_brake_torque_; // Maximum brake torque that can be applied.
  double default_throttle_; // Default throttle value to maintain speed.
  double steering_; // Current steering angle.
  pfms::commands::Ackerman cmd_; // Ackerman command structure for vehicle control.
  Audi audi_; // Instance of the Audi class for vehicle-specific functionalities.
};

#endif // ACKERMAN_H