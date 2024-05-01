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

  //from my friend chat
      // See MissionInterface for more information
      virtual void setGoals(std::vector<pfms::geometry_msgs::Point> goals, pfms::PlatformType platform) override;
      virtual bool run() override;
      virtual std::vector<unsigned int> status(void) override;
      virtual void setMissionObjective(mission::Objective objective) override;
      virtual std::vector<double> getDistanceTravelled() override;
      virtual std::vector<double> getTimeMoving() override;
      virtual std::vector<std::pair<int, int>> getPlatformGoalAssociation() override;
  //cheers chat


private:
  std::vector<ControllerInterface*> platform_controllers_; //!< A private copy of ControllerInterfaces @sa ControllerInterface
  std::vector<pfms::geometry_msgs::Point> mission_goals_; //!< A private copy of goals
  mission::Objective mission_objective_;
  std::vector<unsigned int> platform_ID_;
  

};

#endif // RANGERFUSION_H
