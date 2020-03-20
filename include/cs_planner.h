//
// Created by gzy on 3/19/20.
//

#ifndef SRC_CS_LOCAL_PLANNER_SRC_CS_PLANNER_H_
#define SRC_CS_LOCAL_PLANNER_SRC_CS_PLANNER_H_

#include <string>

#include <nav_core/base_local_planner.h>
#include <base_local_planner/local_planner_util.h>
#include <cs_local_planner/CSPlannerConfig.h>

namespace cs_local_planner {

class CSPlanner {
 public:
  CSPlanner(std::string name, base_local_planner::LocalPlannerUtil *planner_util);
  ~CSPlanner();

  double getSimPeriod() {return sim_period_;}
  void reconfigure(CSPlannerConfig &config);

 private:
  std::string name_;
  base_local_planner::LocalPlannerUtil *planner_util_;
  double sim_period_;
};

} // namespace cs_local_planner

#endif //SRC_CS_LOCAL_PLANNER_SRC_CS_PLANNER_H_
