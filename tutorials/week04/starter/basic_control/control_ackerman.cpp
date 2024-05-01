// Helper utility to send comamnd to ugv
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This executable shows the use of the pipes library to send commands
// and receive odometry from the ugv platform

#include "pfms_types.h"
#include "pfmsconnector.h"
#include "audi.h"
#include <iostream>
#include <stdlib.h>
#include <thread>
#include <chrono>
#include <cmath>

using std::cout;
using std::endl;
using pfms::commands::Ackerman; // we state this here to be able to refer to Ackerman commands instead of the full name

int main(int argc, char *argv[]) {

    // You can use below as starting point to pass a goal for instance, think what you need to do to parse the input
    // if(argc !=3){
    // }
    // else{
    //     atof(argv[2]);
    // }

    //goal coordinates 
    double gx=10.0;
    double gy=5.0;
    
    //start coordinates 
    double sx= 0.0;
    double sy= 0.0;

    double brake = 0.0;
    double steering = 0.0;
    double throttle = 0.1;

    double STEERING_RATIO = 17.3;
    double LOCK_TO_LOCK_REVS = 3.2;
    double MAX_STEER_ANGLE = (M_PI * LOCK_TO_LOCK_REVS / STEERING_RATIO);
    double TRACK_WIDTH = 1.638; //m
    double WHEELBASE = 2.65; //m
    double MAX_BRAKE_TORQUE = 8000.0; //Nm
    double DEFAULT_THROTTLE = 0.1; //= vehicle top speed 2.91m/s

    double alpha= 0;

    double distance= pow(gx-sx,2.0) + pow(gy-sy,2.0);

    double gamma = atan(2*WHEELBASE*sin(alpha)/distance);

    //! Created a pointer to a Pipe 
    std::shared_ptr<PfmsConnector> pfmsConnectorPtr = std::make_shared<PfmsConnector>();
    pfms::nav_msgs::Odometry odo; // We will use this to store odometry
    pfms::PlatformType type = pfms::PlatformType::ACKERMAN;

    //! Our goal is here to keep driving the Ackerman platform towards the goal, stopping at the goal withon a tolerance
    //! We will use the odometry to check if we are close to the goal
    //! We will use the Ackerman command to drive the platform
    //! We will use the pfmsconnector to send commands and receive odometry data

    // This creates a command for the Ackerman platform (refer pfms_types.h for more details)
    unsigned int i=0;
    Ackerman cmd {
                i++, // This is the sequence number (refer to pfms_types.h)
                brake,
                steering,
                throttle,
                };

    // This sends the command to the platform
    pfmsConnectorPtr->send(cmd);
    //! This slows down the loop to 100Hz
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    //! This reads odometry from the platform
    bool OK  =  pfmsConnectorPtr->read(odo,type);

    if(OK){
        std::cout << 
            odo.time << " " <<
            odo.position.x << " " <<
            odo.position.y << " " <<
            odo.yaw << " " <<
            odo.linear.x << " " <<
            odo.linear.y << std::endl;
    }

    //! We can also send a goal to be visualised on rviz with the following command
    unsigned int j=0;
    pfms::geometry_msgs::Point pt{x,y};
    pfms::geometry_msgs::Goal goal{j++,pt};
    pfmsConnectorPtr->send(goal);

    //! Finally we can show use of Audi Library
    Audi audi;
    //audi.checkOriginToDestination - look at the function in audi.h

   return 0;
}
