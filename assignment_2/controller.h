#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "controllerinterface.h" // Includes the base interface for controller implementations.
#include "pfmsconnector.h" // Include for communication with the PFMS (Platform Management System).
#include "audi.h" // Include for specific functionalities related to Audi vehicles.

// The Controller class inherits from ControllerInterface and provides concrete implementations.
class Controller : public ControllerInterface {
public:
    // Constructor and destructor for managing resource allocation and cleanup.
    Controller();
    ~Controller();

    // Method to check if the destination is reachable from the current location and to estimate
    // the time and distance to reach the destination along with the final pose.
    bool checkOriginToDestination(pfms::nav_msgs::Odometry origin, 
                                    pfms::geometry_msgs::Point goal, 
                                    double &distance, 
                                    double &time, 
                                    pfms::nav_msgs::Odometry &estimatedGoalPose);

    // Retrieves the platform type (e.g., ACKERMAN, SKIDSTEER).
    pfms::PlatformType getPlatformType(void);


    // Retrieves the platform status (e.g., IDLE, RUNNING,  TAKEOFF, LANDING).
    pfms::PlatformStatus status(void);

private:
    Audi audi_; // Instance of Audi class for vehicle-specific operations.

protected:
    // Protected members are accessible in derived classes (like Ackerman and SkidSteer).
    
    // Stores the type of the vehicle platform (e.g., ACKERMAN, SKIDSTEER).
    pfms::PlatformType type_;

    // stores the platform status (e.g., IDLE, RUNNING,  TAKEOFF, LANDING).
    pfms::PlatformStatus status_;

    // Tolerance for reaching the goal, affecting when the goal is considered reached.
    double tolerance_;

    // The goal position to reach.
    pfms::geometry_msgs::Point goal_;

    // Current odometry information including position and velocity.
    pfms::nav_msgs::Odometry currentOdometry_;

    // Estimated time to reach the goal.
    double timeToGoal_;

    // Distance to the goal.
    double distanceToGoal_;

    // Estimated pose of the vehicle when the goal is reached.
    pfms::nav_msgs::Odometry estimatedGoalPose_;

    // Pointer to a PfmsConnector for communication with the platform management system.
    std::shared_ptr<PfmsConnector> pfmsConnectorPtr_;

    // Total distance travelled by the vehicle.
    double distanceTravelled_;

    // Total time the vehicle has been in motion.
    double timeInMotion_;
};

#endif // CONTROLLER_H