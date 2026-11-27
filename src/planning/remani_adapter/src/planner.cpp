#include "remani_adapter/planner.hpp"
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <algorithm>

namespace risk_mm {
namespace {
class RemaniTrajectory final : public Trajectory {
  std::vector<poly_traj::Trajectory<7>> segments_;
  std::vector<double> boundaries_{0.};
  double initial_yaw_;
  double planning_yaw_in_base_;
 public:
  RemaniTrajectory(std::vector<poly_traj::Trajectory<7>> segments,double yaw,double planning_yaw_in_base)
    : segments_(std::move(segments)),initial_yaw_(yaw),planning_yaw_in_base_(planning_yaw_in_base) {
    double offset=0.;
    for(const auto &segment:segments_) {
      for(double time:segment.getDurations()) {offset+=time;boundaries_.push_back(offset);}
    }
  }
  double duration() const override {return boundaries_.back();}
  std::vector<double> boundaries() const override {return boundaries_;}
  TrajectoryState evaluate(double time_s) const override {
    if(!std::isfinite(time_s)||time_s<0.||time_s>duration()+1e-8)
      throw std::out_of_range("trajectory time");
    double local=std::min(time_s,duration());size_t index=0;
    while(index+1<segments_.size() && local>segments_[index].getTotalDuration()) {
      local-=segments_[index].getTotalDuration();++index;
    }
    const auto &segment=segments_[index];
    const auto p=segment.getPos(local),v=segment.getVel(local),a=segment.getAcc(local);
    const auto jerk=segment.getJer(local);
    TrajectoryState state;
    for(int j=0;j<8;++j) {
      const int k=j<2?j:j+1;state.position[k]=p[j];state.velocity[k]=v[j];state.acceleration[k]=a[j];
      state.jerk[k]=jerk[j];
    }
    // At a stop, heading is the interior tangent limit, rather than atan2(0,0).
    double heading_time=local;
    if(v.head(2).norm()<1e-6) {
      const double epsilon=std::min(.01,segment.getTotalDuration()/100.);
      heading_time=std::clamp(local+(local<epsilon?epsilon:-epsilon),0.,segment.getTotalDuration());
    }
    const double planning_yaw=segment.getVel(heading_time).head(2).norm()>1e-12?
      segment.getCarAngle(heading_time):initial_yaw_;
    state.position[2]=std::remainder(planning_yaw-planning_yaw_in_base_,2.*M_PI);
    state.velocity[2]=segment.getCarOmega(local);
    state.acceleration[2]=segment.getCarDomega(local);
    state.forward_velocity=std::cos(planning_yaw_in_base_)*segment.getCarVel(local);
    return state;
  }
};
class StartupRampTrajectory final : public Trajectory {
  std::shared_ptr<const Trajectory> continuation_;
  std::array<double,9> start_{};
  double heading_{0.},planning_yaw_in_base_{0.},duration_{0.},distance_{0.};
 public:
  StartupRampTrajectory(std::shared_ptr<const Trajectory> continuation,std::array<double,9> start,
      double heading,double planning_yaw_in_base,double duration,double distance)
      :continuation_(std::move(continuation)),start_(start),heading_(heading),
       planning_yaw_in_base_(planning_yaw_in_base),duration_(duration),distance_(distance) {}
  double duration() const override {return duration_+continuation_->duration();}
  TrajectoryState evaluate(double time_s) const override {
    if(!std::isfinite(time_s)||time_s<0.||time_s>duration()+1e-8)
      throw std::out_of_range("startup ramp trajectory time");
    time_s=std::clamp(time_s,0.,duration());
    if(time_s>=duration_)return continuation_->evaluate(time_s-duration_);
    const double s=time_s/duration_;
    const double position_shape=2.*s*s*s-s*s*s*s;
    const double velocity_shape=(6.*s*s-4.*s*s*s)/duration_;
    const double acceleration_shape=(12.*s-12.*s*s)/(duration_*duration_);
    const double jerk_shape=(12.-24.*s)/(duration_*duration_*duration_);
    const double c=std::cos(heading_),sn=std::sin(heading_);
    TrajectoryState state;state.position=start_;
    state.position[0]+=distance_*c*position_shape;state.position[1]+=distance_*sn*position_shape;
    state.velocity[0]=distance_*c*velocity_shape;state.velocity[1]=distance_*sn*velocity_shape;
    state.acceleration[0]=distance_*c*acceleration_shape;state.acceleration[1]=distance_*sn*acceleration_shape;
    state.jerk[0]=distance_*c*jerk_shape;state.jerk[1]=distance_*sn*jerk_shape;
    state.forward_velocity=std::cos(planning_yaw_in_base_)*distance_*velocity_shape;
    return state;
  }
  std::vector<double> boundaries() const override {
    std::vector<double> result{0.,duration_};
    for(double value:continuation_->boundaries())if(value>1e-10)result.push_back(duration_+value);
    std::sort(result.begin(),result.end());
    result.erase(std::unique(result.begin(),result.end()),result.end());
    return result;
  }
};
}
RemaniPlanner::RemaniPlanner(rclcpp::Node::SharedPtr node, const std::shared_ptr<GridMap>& map,
    const std::shared_ptr<remani_planner::MMConfig>& model, double sample_s) : sample_s_(sample_s) {
  if (!std::isfinite(sample_s_) || sample_s_ <= 0) throw std::invalid_argument("sample interval");
  node->get_parameter_or("mm.mobile_base_non_singul_vel",tangent_speed_,.02);
  node->get_parameter_or("mm.planning_frame_yaw_in_base_rad",planning_yaw_in_base_,0.);
  if(!std::isfinite(planning_yaw_in_base_) || std::abs(std::sin(planning_yaw_in_base_))>1e-8)
    throw std::invalid_argument("planning frame must preserve the longitudinal base axis");
  if(!std::isfinite(tangent_speed_) || tangent_speed_<=0.)throw std::invalid_argument("non-singular tangent velocity required");
  optimizer_.setParam(node,map,model);
  int configured_seed=0;
  if(node->get_parameter("search.random_seed",configured_seed)) {
    if(configured_seed<0)throw std::invalid_argument("search.random_seed must be nonnegative");
    search_seed_=static_cast<std::uint32_t>(configured_seed);fixed_search_seed_=true;
  }
}
Plan RemaniPlanner::plan(const Task& task) {
  return solve(task,{},0.);
}
Plan RemaniPlanner::replan(const Task& task,std::shared_ptr<const Trajectory> reference,double reference_time) {
  return solve(task,std::move(reference),reference_time);
}
Plan RemaniPlanner::solve(const Task& task,std::shared_ptr<const Trajectory> reference,double reference_time) {
  const auto begin=std::chrono::steady_clock::now();
  Plan result; result.task=task;
  auto fail=[&](const std::string& why) {
    result.failure=why;result.points.clear();
    result.planning_wall_s=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
    return result;
  };
  if(task.task_id.empty())return fail("missing task identity");
  for(auto q : {task.start, task.goal})for(double x:q)if(!std::isfinite(x))return fail("nonfinite state");
  // Normalize ROS yaw before entering the original search. The OMPL boundary
  // itself also enforces its own SO(2) representation convention.
  auto normalized_yaw=[](double value) {
    double angle=std::remainder(value,2.*M_PI);
    return angle>=M_PI?angle-2.*M_PI:angle;
  };
  const double start_yaw=normalized_yaw(task.start[2]+planning_yaw_in_base_);
  const double goal_yaw=normalized_yaw(task.goal[2]+planning_yaw_in_base_);
  Task planning_task=task;
  const bool use_startup_ramp=forward_only_ && !task.moving_start && !reference;
  constexpr double startup_ramp_duration_s=.2;
  const double startup_ramp_distance_m=use_startup_ramp?.5*tangent_speed_*startup_ramp_duration_s:0.;
  if(use_startup_ramp) {
    planning_task.start[0]+=startup_ramp_distance_m*std::cos(start_yaw);
    planning_task.start[1]+=startup_ramp_distance_m*std::sin(start_yaw);
    planning_task.start_velocity.fill(0.);planning_task.start_acceleration.fill(0.);planning_task.start_jerk.fill(0.);
    planning_task.start_velocity[0]=tangent_speed_*std::cos(start_yaw);
    planning_task.start_velocity[1]=tangent_speed_*std::sin(start_yaw);
    planning_task.moving_start=true;
    result.task.require_start_boundary=true;
    result.start_transition_duration_s=startup_ramp_duration_s;
    result.start_transition_distance_m=startup_ramp_distance_m;
  }
  if(!reference && fixed_search_seed_)optimizer_.setRandomSeed(search_seed_);
  Eigen::MatrixXd head=Eigen::MatrixXd::Zero(8,4),tail=head;
  for(int i=0;i<8;++i) {
    int j=i<2?i:i+1;head(i,0)=planning_task.start[j];tail(i,0)=planning_task.goal[j];
    if(planning_task.moving_start){head(i,1)=planning_task.start_velocity[j];head(i,2)=planning_task.start_acceleration[j];head(i,3)=planning_task.start_jerk[j];}
  }
  // Match the upstream planner_manager boundary contract: the endpoint tangent
  // encodes the requested yaw with non-singular velocity. A zero tail velocity
  // loses that yaw constraint and creates unbounded four-wheel steering near a
  // stop. The execution layer explicitly brakes after this polynomial endpoint.
  const double tail_speed=task.stop_at_goal?tangent_speed_:task.goal_tangent_speed;
  if(!std::isfinite(tail_speed) || tail_speed<=0.)return fail("positive terminal tangent speed required");
  tail(0,1)=tail_speed*std::cos(goal_yaw);
  tail(1,1)=tail_speed*std::sin(goal_yaw);
  // The original flatness model represents yaw through tangent and direction.
  std::vector<std::vector<Eigen::VectorXd>> paths;
  std::vector<std::vector<double>> yaws;
  std::vector<poly_traj::MinSnapOpt<8>> seeds;
  std::vector<int> directions;
  const int start_direction=planning_task.moving_start?
      (std::cos(start_yaw)*planning_task.start_velocity[0]+std::sin(start_yaw)*planning_task.start_velocity[1]>=0.?1:-1):0;
  int search_status=remani_planner::KinoAstar::REACH_END;
  if(reference) {
    const double remaining=reference->duration()-reference_time;
    if(reference_time<0. || remaining<.1)return fail("warm reference exhausted");
    const auto end=reference->evaluate(reference->duration());
    const int direction=std::cos(goal_yaw)*end.velocity[0]+std::sin(goal_yaw)*end.velocity[1]>=0.?1:-1;
    if(start_direction && direction!=start_direction)return fail("warm reference changes gear");
    tail.col(1).head(2)*=direction;
    std::vector<double> knots;double last=reference_time;
    // A nearly consumed first piece makes the MINCO solve ill-conditioned.
    // Merge sub-0.1 s seed intervals; the resulting candidate is revalidated.
    for(double t:reference->boundaries())if(t-last>.1 && t<reference->duration()-.1){knots.push_back(t);last=t;}
    knots.push_back(reference->duration());
    if(knots.size()<2)knots={reference_time+remaining*.5,reference->duration()};
    const int pieces=static_cast<int>(knots.size());
    Eigen::MatrixXd inner(8,pieces-1);
    Eigen::VectorXd durations(pieces);double previous=reference_time;
    for(int i=0;i<pieces;++i){durations[i]=knots[i]-previous;previous=knots[i];}
    for(int i=1;i<pieces;++i) {
      const auto state=reference->evaluate(knots[i-1]);
      for(int j=0;j<8;++j)inner(j,i-1)=state.position[j<2?j:j+1];
    }
    // MinSnapOpt owns a banded-system buffer and must not be copied/moved.
    seeds.resize(1);auto &seed=seeds.front();seed.reset(head,tail,pieces);
    seed.generate(inner,durations);
    directions.push_back(direction);result.warm_started=true;
  }else search_status=optimizer_.astarWithMinTraj(head,tail,start_yaw,start_direction,false,goal_yaw,false,
                            Eigen::Vector2d::Zero(),0,paths,yaws,seeds,directions);
  result.search_wall_s=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
  if(search_status==remani_planner::KinoAstar::START_COLLISION ||
     search_status==remani_planner::KinoAstar::GOAL_COLLISION) {
    const bool start=search_status==remani_planner::KinoAstar::START_COLLISION;
    result.source_validation=start?"start_geometry_rejected":"goal_geometry_rejected";
    return fail(start?"source start geometry rejected":"source goal geometry rejected");
  }
  if(seeds.empty() || seeds.size()!=directions.size())return fail("original search failed");
  if(capture_diagnostics_ && !reference) {
    for(size_t i=0;i<paths.size() && i<yaws.size();++i) {
      for(size_t j=0;j<paths[i].size() && j<yaws[i].size();++j) {
        if(paths[i][j].size()!=8)continue;
        std::array<double,9> point{};
        point[0]=paths[i][j][0];point[1]=paths[i][j][1];
        point[2]=std::remainder(yaws[i][j]-planning_yaw_in_base_,2.*M_PI);
        for(int k=2;k<8;++k)point[k+1]=paths[i][j][k];
        result.diagnostic_search_path.push_back(point);
      }
    }
    double seed_offset=0.;
    for(size_t i=0;i<seeds.size();++i) {
      const auto trajectory=seeds[i].getTraj(directions[i]);
      const double duration=trajectory.getTotalDuration();
      const int samples=std::max(1,static_cast<int>(std::ceil(duration/sample_s_)));
      for(int k=(i?1:0);k<=samples;++k) {
        const double local=duration*k/samples;
        const auto position=trajectory.getPos(local);
        Point point;point.time_s=seed_offset+local;
        for(int j=0;j<8;++j)point.position[j<2?j:j+1]=position[j];
        point.position[2]=std::remainder(trajectory.getCarAngle(local)-planning_yaw_in_base_,2.*M_PI);
        result.diagnostic_minco_seed.push_back(point);
      }
      seed_offset+=duration;
    }
  }
  std::vector<Eigen::MatrixXd> heads,tails,inners,control,waypoints;
  std::vector<Eigen::VectorXd> durations,opt_times,ee;
  optimizer_.clear_resize_Cps_container(seeds.size());
  for(size_t i=0;i<seeds.size();++i) {
    auto traj=seeds[i].getTraj(directions[i]);int n=traj.getPieceNum();
    Eigen::MatrixXd h(8,4),t(8,4);
    h<<traj.getJuncPos(0),traj.getJuncVel(0),traj.getJuncAcc(0),traj.getJuncJerk(0);
    t<<traj.getJuncPos(n),traj.getJuncVel(n),traj.getJuncAcc(n),traj.getJuncJerk(n);
    RCLCPP_INFO(rclcpp::get_logger("remani_adapter"),
      "task=%s segment=%zu gear=%d initial_velocity=(%.5f,%.5f) terminal_velocity=(%.5f,%.5f)",
      task.task_id.c_str(),i,directions[i],h(0,1),h(1,1),t(0,1),t(1,1));
    heads.push_back(h);tails.push_back(t);durations.push_back(traj.getDurations());
    inners.push_back(traj.getPositions().block(0,1,8,n-1));
    optimizer_.setControlPoints(i,seeds[i].getInitConstrainPoints(optimizer_.get_cps_num_prePiece_()));
  }
  const bool source_passed=optimizer_.OptimizeTrajectory_lbfgs(heads,tails,inners,durations,directions,control,waypoints,opt_times,ee);
  result.optimization_wall_s=optimizer_.solveWallSeconds();result.source_check_wall_s=optimizer_.checkWallSeconds();
  result.cost_evaluations=optimizer_.costEvaluations();
  result.optimization_budget_hit=optimizer_.solveBudgetHit();
  result.source_validation=source_passed?(result.optimization_budget_hit?"budget_candidate_checked":"passed"):"optimizer_rejected";
  if(source_passed && optimizer_.externalKinematicValidation())result.source_validation="geometry_checked; current robot kinematics require public validation";
  const auto &optimized=*optimizer_.getMinSnapOptContainerPtr();
  if(capture_diagnostics_) {
    double diagnostic_offset=0.;
    for(size_t i=0;i<optimized.size() && i<directions.size();++i) {
      const auto trajectory=optimized[i].getTraj(directions[i]);
      const double duration=trajectory.getTotalDuration();
      const int samples=std::max(1,static_cast<int>(std::ceil(duration/sample_s_)));
      for(int k=(i?1:0);k<=samples;++k) {
        const double local=duration*k/samples;
        const auto position=trajectory.getPos(local);
        Point point;point.time_s=diagnostic_offset+local;
        for(int j=0;j<8;++j)point.position[j<2?j:j+1]=position[j];
        point.position[2]=std::remainder(trajectory.getCarAngle(local)-planning_yaw_in_base_,2.*M_PI);
        result.diagnostic_optimized_candidate.push_back(point);
      }
      diagnostic_offset+=duration;
    }
  }
  if(!source_passed) {
    RCLCPP_WARN(rclcpp::get_logger("remani_adapter"),"task=%s rejected after %d evaluations; seed=%s",
      task.task_id.c_str(),result.cost_evaluations,([&]{std::string value;for(const auto &seed:seeds){auto p=seed.getTraj(1).getPositions();for(int i=0;i<p.cols();++i)value+="("+std::to_string(p(0,i))+","+std::to_string(p(1,i))+") ";}return value;})().c_str());
    return fail("original optimizer rejected trajectory");
  }
  RCLCPP_INFO(rclcpp::get_logger("remani_adapter"),"task=%s source_validation=%s",task.task_id.c_str(),result.source_validation.c_str());
  double offset=0;
  std::vector<poly_traj::Trajectory<7>> exact_segments;
  if(optimized.empty())return fail("original optimizer returned no trajectory");
  for(size_t i=0;i<optimized.size();++i) {
    auto trajectory=optimized[i].getTraj(directions[i]);
    exact_segments.push_back(trajectory);
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
  std::shared_ptr<const Trajectory> remani_trajectory=
      std::make_shared<RemaniTrajectory>(std::move(exact_segments),start_yaw,planning_yaw_in_base_);
  result.trajectory=use_startup_ramp?
      std::static_pointer_cast<const Trajectory>(std::make_shared<StartupRampTrajectory>(remani_trajectory,task.start,
          start_yaw,planning_yaw_in_base_,startup_ramp_duration_s,startup_ramp_distance_m)):
      remani_trajectory;
  if(use_startup_ramp) {
    result.points.clear();
    const double duration=result.trajectory->duration();
    const int samples=std::max(1,static_cast<int>(std::ceil(duration/sample_s_)));
    for(int k=0;k<=samples;++k) {
      const double time=duration*k/samples;Point point;point.time_s=time;
      point.position=result.trajectory->evaluate(time).position;result.points.push_back(point);
    }
  }
  for(auto &point:result.points)point.position=result.trajectory->evaluate(point.time_s).position;
  result.planning_wall_s=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
  return result; // Candidate only: common validator/executor integration is separate.
}
} // namespace risk_mm
