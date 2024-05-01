#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

#include "quadcopter.h"
#include "pfms_types.h"

//using namespace std;
using namespace pfms::nav_msgs;

int main() {

    std::vector<pfms::geometry_msgs::Point> points;

    int num_points;
    std::cout << "Number of goals? ";
    std::cin >> num_points;
    points.reserve(num_points);

    for (int i = 0; i < num_points; i++) {
        pfms::geometry_msgs::Point pt;
        std::cout << "Goal " << i << " x? ";
        std::cin >> pt.x;
        std::cout << "Goal " << i << " y? ";
        std::cin >> pt.y;
        std::cout << "Goal " << i << " z? ";
        std::cin >> pt.z;
        points.push_back(pt);
    }

    //Set-up the controllers
    std::vector<ControllerInterface*> controllers;
//    controllers.push_back(new Ackerman());
    controllers.push_back(new Quadcopter());

    //Setting tolerance to reach goals
    controllers.at(0)->setTolerance(0.2);

    //Setting goals
    controllers.at(0)->setGoals(points);
    // Call Run function
    controllers.at(0)->run();


    for (auto pt : points) {
//        bool reachable = controllers.at(0)->setGoal(pt);
//        double dist = controllers.at(0)->distanceToGoal();
//        double t = controllers.at(0)->timeToGoal();
//        std::cout << "Controller: can reach goal " <<
//                             dist << "[m] " << t << "[s]" << std::endl;
//        if(reachable){
//            bool reached = controllers.at(0)->reachGoal();
//        }

        std::cout << "Controller: TOTAL distance travelled: " <<
                    controllers.at(0)->distanceTravelled() << "[m] " <<
                     " time travelled: " << controllers.at(0)->timeTravelled()  << "[s]" << std::endl;
    }
}
