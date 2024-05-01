
#ifndef MISSION_H
#define MISSION_H


#include "missioninterface.h"
#include "controller.h"
#include "ackerman.h"
#include "skidsteer.h"
#include "pfmsconnector.h"

#include <vector>


/*!
 *  \brief     Mission Base Class
 *  \details
 * Mission Base Class, inherits from Mission Interface Class.
 * It carrys out a mission to drive to multiple goals within specified tolerance and shortest travel distance.
 * It comunicates to the coreect platform via controller interface pointers
 *  \author    Wil
 *  \date      2024-03-23
 */



class Mission: public MissionInterface
{
public:
   
  // this the constructor for the mission class
  Mission(std::vector<ControllerInterface*> controllers);

    /**
    Set Mission Objective - this methord passes the objective to to a member variable called mission_objective_
    */

    void setMissionObjective(mission::Objective objective);

    /**
    Set Goals - this methord stores the missions goals in a container vector mission_goals_ 
    @param[in] goals
    @param [out] mission_goals_
    */

    void setGoals(std::vector<pfms::geometry_msgs::Point> goals);

    /**
    run mission - this methord carrys out the mission
    checks the type of platform
    0 = ackerman
    1 = skidsteer
    sets the correct goals, tolerrance, and if it will reach goal for the correct platform
    alsoinitializes an integer variable count to keep track of the number of reached goals.
    @returns bool mission_complete status
    */

    bool runMission();

    /**
     * get time moving - The purpose of this method, getTimeMoving(), is to calculate and return a container vector called platform_moving_time.
     *  it holds time spent in motion for each platform in a mission.
     * @param[in] timeInMotion()

    @returns platform_moving_time
    */

    std::vector<double> getTimeMoving(void);

    /**
    get distance traveled- the purpose of this method is to calculate and return a vector of distances traveled by different platforms in a mission.
    it initializes an empty vector called platformDistanceTravelled to use as a container to hold doubles.
     
    @returns platformDistanceTravelled vector populated with the disance traveled by each platform.    
    */

    std::vector<double> getDistanceTravelled(void);

    /**
    getPlatformGoalAssociation() - The purpose of this method is to determine the association between platform types and their corresponding goals.
    Details:
    initializes an empty std::vector<unsigned int> called platforms.
    iterates over the goals stored in mission_goals_ vector 
    For each goal:
    It iterates over the platform controllers stored in platform_controllers_ vector
    For each platform controller:
    If the platform type is ACKERMAN, it associates the goal with the value 0.
    If the platform type is SKIDSTEER, it associates the goal with the value 1.
    @returns platform_ID_  vector populated with platform-goal associations
    */

    std::vector<unsigned int> getPlatformGoalAssociation(void);

private:
  std::vector<ControllerInterface*> platform_controllers_; //!< A private copy of ControllerInterfaces @sa ControllerInterface
  std::vector<pfms::geometry_msgs::Point> mission_goals_; //!< A private copy of goals
  mission::Objective mission_objective_;
  std::vector<unsigned int> platform_ID_;


};

#endif // RANGERFUSION_H
