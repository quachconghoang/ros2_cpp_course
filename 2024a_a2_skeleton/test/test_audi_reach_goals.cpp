#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <thread>
#include <cmath>
#include "pfms_types.h"

//Student defined libraries
#include "ackerman.h"
#include "mission.h"

// Some helper header for assembling messages and testing
#include "test_helper.h"
#include "pfmshog.h"

using namespace std;
using namespace pfms::nav_msgs;


///////////////////////////////////////////////////////////
// Unit Tests Start HERE
////////////////////////////////////////////////////////

TEST(AckermanReachGoals, ThreeGoals) {

    
    std::unique_ptr<PfmsHog> pfmsHogPtr = std::make_unique<PfmsHog>(pfms::PlatformType::ACKERMAN);
    {
        Odometry odo = populateOdo(0,2,0);
        pfmsHogPtr->teleport(odo);
    }

    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Ackerman());
    controllers.front()->setTolerance(0.5);

    double inc = 10.0;

    std::vector<pfms::geometry_msgs::Point> goals;
    // goals.push_back({ 10, 6});
    // goals.push_back({ 20, 2});
    // goals.push_back({ 10, 4});
    goals.push_back({ inc, -inc});
    goals.push_back({ 2*inc, -2*inc});
    goals.push_back({ 3*inc, -3*inc});
    // goals.push_back({ 3*inc, -6*inc});
    // goals.push_back({ 4*inc, -4*inc});

    for(auto goal: goals){
        goal.x += 0;
        goal.y += 2;
    }

    //We set the goal for the PfmsHog (Which will be used to check if the goal is reached or not)
    pfmsHogPtr->setGoals(goals);

    //We send the goals to the controller
    controllers.front()->setGoals(goals);

    // We have an estimated time to reach goal, which is conservative usually vehicles reaches it in
    // less time. Nevertheless we could use 180s as the max time to reach goal 
    //
    // We will loop until that time and if the goal is not reached until then (or we have status
    // indicating IDLE, we know it has been reached
    auto start_time = std::chrono::system_clock::now();
    double maxTime = 180.0;

    // The below should not block and we will be back and can check progress
    controllers.front()->run();
    bool OK =false; // This will indicate if mission is completed
    bool timeExceeded = false; // time exceeded


    while (!OK){

        auto current_time = std::chrono::system_clock::now();
        //std::chrono::seconds is integer for some reason, thus duration<double>
        auto time_taken = std::chrono::duration_cast<std::chrono::duration<double>>(current_time - start_time);

        if(time_taken.count()>(maxTime)){
            //We have now taken time
            timeExceeded=true;
            OK=true;
        }

        pfms::PlatformStatus status = controllers.front()->status();

        if(status == pfms::PlatformStatus::IDLE){
            //mission accomplished
            OK=true;
        }

        //Let's slow down this loop to 200ms (5Hz)
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    ASSERT_FALSE(timeExceeded); // time shoudl not be exceeded
    //We now check that the goal has been reached using the PfmsHog
    std::vector<double> distances;
    bool reachedCheck = pfmsHogPtr->checkGoalsReached(distances);
    std::cout << "Goal reach check:" << reachedCheck << std::endl;
    ASSERT_TRUE(reachedCheck);


    // We can also check the diatance to the goal reported by PfmsHog
    // Though this is done internally in checkGoalsReached
    ASSERT_EQ(distances.size(),3);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
