#ifndef MISSION_H
#define MISSION_H

#include <vector>
#include "missioninterface.h"
#include "pfmsconnector.h"

class Mission: public MissionInterface
{
public:
    /**
    The Default constructor
    @sa ControllerInterface and @sa MissionInterface for more information
    */
    Mission(std::vector<ControllerInterface*> controllers);

    // inherited from MissionInterface: dummy headers
    // Please Add Doxygen comments to these functions
    void setGoals(std::vector<pfms::geometry_msgs::Point> goals, pfms::PlatformType platform) ;

    // CALL ALL CONTROLLERS for RUNNING
    bool run() ;

    // CALL ALL Vehicles CONTROLLERS for STATUS
    std::vector<unsigned int> status(void) ;

    // SIMILAR
    void setMissionObjective(mission::Objective objective);
    std::vector<double> getDistanceTravelled();
    std::vector<double> getTimeMoving();
    std::vector<std::pair<int, int>> getPlatformGoalAssociation();


private:
  std::vector<ControllerInterface*> controllers_; //!< A private copy of ControllerInterfaces @sa ControllerInterface
  std::vector<pfms::geometry_msgs::Point*> goals_; //!< A private copy of goals

};

#endif // RANGERFUSION_H
