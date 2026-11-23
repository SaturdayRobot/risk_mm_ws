#include "remani_adapter/planner.hpp"
#include <iostream>
int main(int argc,char**argv) {
 rclcpp::init(argc,argv);
 auto node=std::make_shared<rclcpp::Node>("coupled_check",rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
 int status=0;
 try {
  auto map=std::make_shared<GridMap>(); map->initMap(node);
  auto model=std::make_shared<remani_planner::MMConfig>(); model->setParam(node,map);
  risk_mm::RemaniPlanner planner(node,map,model);
  std::cout<<"ROS2 original REMANI + current URDF initialized; execution disabled"<<std::endl;
  bool run=false;node->get_parameter_or("test.run_plan",run,false);
  if(run)throw std::runtime_error("Planning requires a frozen, complete map and common validator; this initialization check does not synthesize a free map");
 }catch(const std::exception&e){std::cerr<<e.what()<<std::endl;status=1;}
 rclcpp::shutdown();return status;
}
