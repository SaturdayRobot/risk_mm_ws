#include "remani_adapter/planner.hpp"
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace risk_mm {
RemaniPlanner::RemaniPlanner(rclcpp::Node::SharedPtr node, const std::shared_ptr<GridMap>& map,
    const std::shared_ptr<remani_planner::MMConfig>& model, double sample_s) : sample_s_(sample_s) {
  if (!std::isfinite(sample_s_) || sample_s_ <= 0) throw std::invalid_argument("sample interval");
  optimizer_.setParam(node,map,model);
}
Plan RemaniPlanner::plan(const Task& task) {
  const auto begin=std::chrono::steady_clock::now();
  Plan result; result.task=task;
  auto fail=[&](const std::string& why) {
    result.failure=why;result.points.clear();
    result.planning_wall_s=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
    return result;
  };
  if(task.task_id.empty())return fail("missing task identity");
  for(auto q : {task.start, task.goal})for(double x:q)if(!std::isfinite(x))return fail("nonfinite state");
  Eigen::MatrixXd head=Eigen::MatrixXd::Zero(8,4),tail=head;
  for(int i=0;i<8;++i) {int j=i<2?i:i+1;head(i,0)=task.start[j];tail(i,0)=task.goal[j];}
  // The original flatness model represents yaw through tangent and direction.
  std::vector<std::vector<Eigen::VectorXd>> paths;
  std::vector<std::vector<double>> yaws;
  std::vector<poly_traj::MinSnapOpt<8>> seeds;
  std::vector<int> directions;
  optimizer_.astarWithMinTraj(head,tail,task.start[2],0,false,task.goal[2],false,
                            Eigen::Vector2d::Zero(),0,paths,yaws,seeds,directions);
  if(seeds.empty() || seeds.size()!=directions.size())return fail("original search failed");
  std::vector<Eigen::MatrixXd> heads,tails,inners,control,waypoints;
  std::vector<Eigen::VectorXd> durations,opt_times,ee;
  optimizer_.clear_resize_Cps_container(seeds.size());
  for(size_t i=0;i<seeds.size();++i) {
    auto traj=seeds[i].getTraj(directions[i]);int n=traj.getPieceNum();
    Eigen::MatrixXd h(8,4),t(8,4);
    h<<traj.getJuncPos(0),traj.getJuncVel(0),traj.getJuncAcc(0),traj.getJuncJerk(0);
    t<<traj.getJuncPos(n),traj.getJuncVel(n),traj.getJuncAcc(n),traj.getJuncJerk(n);
    heads.push_back(h);tails.push_back(t);durations.push_back(traj.getDurations());
    inners.push_back(traj.getPositions().block(0,1,8,n-1));
    optimizer_.setControlPoints(i,seeds[i].getInitConstrainPoints(optimizer_.get_cps_num_prePiece_()));
  }
  if(!optimizer_.OptimizeTrajectory_lbfgs(heads,tails,inners,durations,directions,control,waypoints,opt_times,ee))
    return fail("original optimizer rejected trajectory");
  double offset=0;
  const auto& optimized=*optimizer_.getMinSnapOptContainerPtr();
  for(size_t i=0;i<optimized.size();++i) {
    auto trajectory=optimized[i].getTraj(directions[i]);
    double duration=trajectory.getTotalDuration();
    if(!std::isfinite(duration)||duration<=0)return fail("invalid trajectory duration");
    int samples=static_cast<int>(std::ceil(duration/sample_s_));
    for(int k=(i?1:0);k<=samples;++k) {
      double t=duration*k/samples;auto pos=trajectory.getPos(t);
      Point p; p.time_s=offset+t;
      for(int j=0;j<8;++j)p.position[j<2?j:j+1]=pos[j];
      p.position[2]=trajectory.getCarAngle(t);
      for(double v:p.position)if(!std::isfinite(v))return fail("nonfinite trajectory sample");
      result.points.push_back(p);
    }
    offset+=duration;
  }
  result.planning_wall_s=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
  return result; // Candidate only: common validator/executor integration is separate.
}
} // namespace risk_mm
