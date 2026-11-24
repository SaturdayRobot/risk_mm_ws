#include "risk_mm_runtime/world_scene.hpp"
#include "remani_adapter/planner.hpp"
#include "risk_mm_runtime/validation.hpp"
#include "risk_mm_runtime/time_scaling.hpp"
#include <optimizer/four_wheel_constraint.hpp>
#include <mm_model/wheel_kinematics.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>

int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp::Node>("coupled_probe",rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  YAML::Node report;
  report["scope"]="real original REMANI planning in the complete selected static world; no execution";
  report["status"]="failed";
  std::string output;
  node->get_parameter("report_file",output);
  try {
    for(const auto &sample:std::vector<std::array<double,4>>{{.02,.005,.03,.04},{-.1,.03,.02,-.05},{.2,0.,0.,.1}}) {
      Eigen::Vector2d v(sample[0],sample[1]),a(sample[2],sample[3]);
      const auto analytic=risk_mm::fourWheelCurvature(v,a,.3);
      for(int k=0;k<4;++k) {
        auto vp=v,vm=v,ap=a,am=a;const double epsilon=1e-7;
        if(k<2){vp[k]+=epsilon;vm[k]-=epsilon;}else{ap[k-2]+=epsilon;am[k-2]-=epsilon;}
        const double numeric=(risk_mm::fourWheelCurvature(vp,ap,.3).value-risk_mm::fourWheelCurvature(vm,am,.3).value)/(2*epsilon);
        const double expected=k<2?analytic.velocity_gradient[k]:analytic.acceleration_gradient[k-2];
        if(std::abs(numeric-expected)>1e-5*std::max(1.,std::abs(numeric)))throw std::runtime_error("four-wheel curvature gradient mismatch");
      }
    }
    report["four_wheel_curvature_gradient_passed"]=true;
    std::string world,metadata;
    node->get_parameter("world_file",world);node->get_parameter("world_metadata_file",metadata);
    risk_mm::WorldScene scene(world,metadata);
    auto map=std::make_shared<GridMap>();map->initMap(node);
    auto odom=node->create_publisher<nav_msgs::msg::Odometry>("grid_map/odom",10);
    nav_msgs::msg::Odometry state;state.header.frame_id="world";state.pose.pose.orientation.w=1.;
    auto start=std::chrono::steady_clock::now();
    while(!map->odomValid()) {
      if(std::chrono::steady_clock::now()-start>std::chrono::seconds(3))throw std::runtime_error("map odometry initialization timed out");
      state.header.stamp=node->now();odom->publish(state);rclcpp::spin_some(node);std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std_msgs::msg::Header header;header.frame_id="world";header.stamp=node->now();
    auto cloud=scene.cloud(map->getResolution(),header);
    if(!map->replaceCompleteSnapshot(cloud))throw std::runtime_error("complete world snapshot rejected");
    auto model=std::make_shared<remani_planner::MMConfig>();model->setParam(node,map);
    double heading_gradient_error=0.,rotation_gradient_error=0.;
    for(int direction:{-1,1})for(const auto &v:std::vector<Eigen::Vector2d>{
        {.005,.002},{-.01,.003},{.02,0.},{.1,.03},{1e-8,2e-8}}) {
      const double epsilon=v.norm()<1e-6?1e-10:1e-8;
      const Eigen::Vector2d point(.2,-.1),heading=model->caldYawdV(v);
      const auto rotation=model->caldRldv(v,point,direction);
      for(int k=0;k<2;++k) {
        auto plus=v,minus=v;plus[k]+=epsilon;minus[k]-=epsilon;
        const double numeric=std::remainder(model->calYaw(plus,direction)-model->calYaw(minus,direction),2*M_PI)/(2*epsilon);
        heading_gradient_error=std::max(heading_gradient_error,std::abs(numeric-heading[k])/(1.+std::abs(numeric)));
        const Eigen::Vector2d differential=(model->calR(plus,direction)*point-model->calR(minus,direction)*point)/(2*epsilon);
        rotation_gradient_error=std::max(rotation_gradient_error,(differential-rotation.col(k)).norm()/(1.+differential.norm()));
      }
    }
    report["low_speed_heading_gradient_relative_error"]=heading_gradient_error;
    report["low_speed_rotation_gradient_relative_error"]=rotation_gradient_error;
    if(heading_gradient_error>1e-5 || rotation_gradient_error>1e-5)
      throw std::runtime_error("low-speed heading/rotation gradient mismatch");
    report["low_speed_geometry_gradient_passed"]=true;
    risk_mm::RemaniPlanner planner(node,map,model);
    bool capture_diagnostics=false;
    node->get_parameter_or("probe.capture_diagnostics",capture_diagnostics,false);
    planner.setCaptureDiagnostics(capture_diagnostics);
    report["diagnostics_enabled"]=capture_diagnostics;
    bool forward_only=false,external_kinematics=false;double minimum_turning_radius=.55;
    node->get_parameter_or("probe.forward_only",forward_only,false);
    node->get_parameter_or("probe.minimum_turning_radius",minimum_turning_radius,.55);
    node->get_parameter_or("probe.external_kinematics",external_kinematics,false);
    planner.setForwardOnly(forward_only,minimum_turning_radius);
    planner.setExternalKinematicValidation(external_kinematics);
    report["forward_only"]=forward_only;report["minimum_turning_radius_m"]=minimum_turning_radius;
    report["external_four_wheel_validation"]=external_kinematics;
    risk_mm::Task task;task.task_id="coupled-world-probe";task.map_revision=scene.revision();
    std::vector<double> from,to;
    node->get_parameter("task_start",from);node->get_parameter("task_goal",to);
    node->get_parameter("model_signature",task.model_signature);
    if(from.size()!=9 || to.size()!=9)throw std::runtime_error("nine-dimensional start and goal required");
    std::copy(from.begin(),from.end(),task.start.begin());std::copy(to.begin(),to.end(),task.goal.begin());
    double through_speed=0.;node->get_parameter_or("probe.through_speed",through_speed,0.);
    bool stop_at_goal=through_speed<=0.;node->get_parameter_or("probe.stop_at_goal",stop_at_goal,stop_at_goal);
    task.stop_at_goal=stop_at_goal;
    if(through_speed>0.)task.goal_tangent_speed=through_speed;
    if(!task.stop_at_goal && through_speed<=0.)throw std::runtime_error("moving goal requires positive through speed");
    node->get_parameter_or("probe.moving_start",task.moving_start,false);
    if(task.moving_start) {
      for(const auto &entry:std::vector<std::pair<std::string,std::array<double,9>*>>{
          {"probe.start_velocity",&task.start_velocity},
          {"probe.start_acceleration",&task.start_acceleration},
          {"probe.start_jerk",&task.start_jerk}}) {
        std::vector<double> values;
        if(!node->get_parameter(entry.first,values) || values.size()!=9)
          throw std::runtime_error("moving start requires nine values: "+entry.first);
        for(double value:values)if(!std::isfinite(value))throw std::runtime_error("nonfinite moving boundary: "+entry.first);
        std::copy(values.begin(),values.end(),entry.second->begin());
      }
    }
    for(const auto &entry:std::vector<std::pair<std::string,std::array<double,4>*>>{
        {"task_start_steer",&task.start_steer},{"task_start_wheel_position",&task.start_wheel_position}}) {
      std::vector<double> values;
      if(node->get_parameter(entry.first,values)) {
        if(values.size()!=4)throw std::runtime_error("four wheel states required: "+entry.first);
        std::copy(values.begin(),values.end(),entry.second->begin());
      }
    }
    report["task_start"]=from;report["task_goal"]=to;report["moving_start"]=task.moving_start;
    report["stop_at_goal"]=task.stop_at_goal;report["through_speed_m_s"]=through_speed;
    int search_seed=0;
    report["search_random_seed"]=node->get_parameter("search.random_seed",search_seed)?YAML::Node(search_seed):YAML::Node(YAML::Null);
    report["start_velocity"]=std::vector<double>(task.start_velocity.begin(),task.start_velocity.end());
    report["start_acceleration"]=std::vector<double>(task.start_acceleration.begin(),task.start_acceleration.end());
    report["start_jerk"]=std::vector<double>(task.start_jerk.begin(),task.start_jerk.end());
    std::string urdf,srdf;
    node->get_parameter("robot_urdf_file",urdf);node->get_parameter("robot_srdf_file",srdf);
    risk_mm::ValidationLimits limits;
    auto limit=[&](const std::string &name,double &value){node->get_parameter_or(name,value,value);};
    limit("validation.arm_velocity",limits.arm_velocity);limit("validation.arm_acceleration",limits.arm_acceleration);
    limit("validation.base_velocity",limits.base_velocity);limit("validation.base_yaw_rate",limits.base_yaw_rate);
    limit("validation.wheel_velocity",limits.wheel_velocity);limit("validation.wheel_acceleration",limits.wheel_acceleration);
    limit("validation.steer_rate",limits.steer_rate);limit("validation.collision_padding",limits.collision_padding);
    std::vector<double> speeds;
    if(node->get_parameter("task_speed_limits",speeds)) {
      if(speeds.size()!=3)throw std::runtime_error("three task speed limits required");
      std::array<double*,3> values{&limits.base_velocity,&limits.base_yaw_rate,&limits.arm_velocity};
      for(int i=0;i<3;++i) {
        if(!std::isfinite(speeds[i]) || speeds[i]<=0. || speeds[i]>*values[i])throw std::runtime_error("task speed exceeds configured limit");
        *values[i]=speeds[i];task.speed_limits[i]=speeds[i];
      }
    }
    report["requested_speed_limits"]=task.speed_limits;
    std::vector<double> wheel_positions;node->get_parameter("controller.wheel_positions",wheel_positions);
    if(wheel_positions.size()!=8)throw std::runtime_error("Missing current wheel geometry");
    std::copy(wheel_positions.begin(),wheel_positions.end(),limits.wheel_positions.begin());
    auto mesh=std::make_shared<risk_mm::MeshScene>(scene,urdf,srdf,limits.collision_padding);
    std::string reason;
    if(!mesh->valid(task.start,task.start_steer,task.start_wheel_position,reason))throw std::runtime_error("start: "+reason);
    report["start_geometry_valid"]=true;
    if(!task.moving_start) {
      if(!mesh->valid(task.goal,task.start_steer,task.start_wheel_position,reason))throw std::runtime_error("goal: "+reason);
      report["goal_geometry_preflight"]=true;
    } else report["goal_geometry_preflight"]=YAML::Null; // The sampled trajectory checks the solved end-wheel state.
    // A planning-frame calibration must leave physical world geometry intact.
    // Compare the C++ model against the untouched URDF-derived CAD parameters.
    double planning_yaw=0.;node->get_parameter_or("mm.planning_frame_yaw_in_base_rad",planning_yaw,0.);
    std::array<double,8> planning_wheels{};
    for(int i=0;i<4;++i) {
      const double x=wheel_positions[2*i],y=wheel_positions[2*i+1];
      planning_wheels[2*i]=std::cos(planning_yaw)*x+std::sin(planning_yaw)*y;
      planning_wheels[2*i+1]=-std::sin(planning_yaw)*x+std::cos(planning_yaw)*y;
    }
    double wheel_gradient_error=0.,wheel_model_error=0.;
    for(int direction:{-1,1})for(const auto &sample:std::vector<std::array<double,6>>{
        {.02,.005,.03,.04,.01,-.02},{-.1,.03,.02,-.05,.04,.01},{.2,0.,0.,.1,.02,-.03}}) {
      const Eigen::Vector2d v(sample[0],sample[1]),a(sample[2],sample[3]),j(sample[4],sample[5]);
      const auto analytic=risk_mm::fourWheelDifferentials(v,a,j,direction,planning_wheels,limits.wheel_radius);
      for(int k=0;k<6;++k) {
        auto vp=v,vm=v,ap=a,am=a,jp=j,jm=j;const double epsilon=1e-7;
        if(k<2){vp[k]+=epsilon;vm[k]-=epsilon;}
        else if(k<4){ap[k-2]+=epsilon;am[k-2]-=epsilon;}
        else {jp[k-4]+=epsilon;jm[k-4]-=epsilon;}
        const Eigen::Matrix<double,12,1> numeric=(risk_mm::fourWheelDifferentials(vp,ap,jp,direction,planning_wheels,limits.wheel_radius).squared-
          risk_mm::fourWheelDifferentials(vm,am,jm,direction,planning_wheels,limits.wheel_radius).squared)/(2.*epsilon);
        for(int row=0;row<12;++row)wheel_gradient_error=std::max(wheel_gradient_error,
            std::abs(numeric[row]-analytic.jacobian(row,k))/(1.+std::abs(numeric[row])));
      }
      auto public_wheels=[&](double t) {
        const Eigen::Vector2d velocity=v+t*a+.5*t*t*j,acceleration=a+t*j;
        const double omega=(velocity.x()*acceleration.y()-velocity.y()*acceleration.x())/velocity.squaredNorm();
        return mm_model::inverse(std::cos(planning_yaw)*direction*velocity.norm(),0.,omega,
            limits.wheel_positions,limits.wheel_radius,std::numeric_limits<double>::infinity(),{});
      };
      const auto center=public_wheels(0.),plus=public_wheels(1e-6),minus=public_wheels(-1e-6);
      for(int wheel=0;wheel<4;++wheel) {
        const double acceleration=(plus.speed[wheel]-minus.speed[wheel])/2e-6;
        const double steer_rate=std::remainder(plus.steer[wheel]-minus.steer[wheel],M_PI)/2e-6;
        const std::array<double,3> expected{center.speed[wheel]*center.speed[wheel],acceleration*acceleration,steer_rate*steer_rate};
        for(int k=0;k<3;++k)wheel_model_error=std::max(wheel_model_error,
            std::abs(expected[k]-analytic.squared[3*wheel+k])/(1.+std::abs(expected[k])));
      }
    }
    report["four_wheel_differential_gradient_relative_error"]=wheel_gradient_error;
    report["four_wheel_public_model_relative_error"]=wheel_model_error;
    if(wheel_gradient_error>1e-5 || wheel_model_error>1e-5)
      throw std::runtime_error("four-wheel soft model/gradient disagrees with public inverse");
    report["four_wheel_differential_gradient_passed"]=true;
    std::vector<double> mount,body;
    node->get_parameter("mm.arm_mount_transform",mount);node->get_parameter("mm.base_collision_points",body);
    if(mount.size()!=16 || body.empty() || body.size()%3)throw std::runtime_error("missing frame regression geometry");
    Eigen::Matrix4d cad_mount;for(int r=0;r<4;++r)for(int c=0;c<4;++c)cad_mount(r,c)=mount[4*r+c];
    for(double yaw:std::vector<double>{0.,task.start[2],.37}) {
      Eigen::Matrix4d cad_world,planning_world;
      model->CarState2T(Eigen::Vector3d(task.start[0],task.start[1],yaw),cad_world);
      model->CarState2T(Eigen::Vector3d(task.start[0],task.start[1],yaw+planning_yaw),planning_world);
      if((cad_world*cad_mount-planning_world*model->getTq0()).norm()>1e-10)
        throw std::runtime_error("planning calibration moved the arm mount in world");
      std::vector<Eigen::Vector3d> points;
      model->getCarPts(Eigen::Vector3d(task.start[0],task.start[1],yaw+planning_yaw),points);
      if(points.size()!=body.size()/3)throw std::runtime_error("planning calibration lost collision samples");
      for(size_t k=0;k<points.size();++k) {
        const Eigen::Vector4d expected=cad_world*Eigen::Vector4d(body[3*k],body[3*k+1],body[3*k+2],1.);
        if((points[k]-expected.head<3>()).norm()>1e-10)
          throw std::runtime_error("planning calibration moved a body collision sample in world");
      }
    }
    report["planning_frame_geometry_invariant_passed"]=true;
    report["planning_frame_yaw_in_base_rad"]=planning_yaw;
    remani_planner::PolyTrajOptimizer cost_reference;cost_reference.setParam(node,map,model);
    double cost_error=0.;
    for(int sample=0;sample<12;++sample) {
      Eigen::VectorXd p(8),v=Eigen::VectorXd::Zero(8);
      for(int j=0;j<8;++j)p[j]=task.start[j<2?j:j+1];
      for(int j=2;j<8;++j)p[j]+=.35*std::sin((sample+1.)*(j+1.));
      v[0]=.15*std::cos(task.start[2]+planning_yaw);v[1]=.15*std::sin(task.start[2]+planning_yaw);
      cost_error=std::max(cost_error,cost_reference.checkCollisionCostIndex(p,v));
    }
    report["collision_cost_index_relative_error"]=cost_error;
    if(cost_error>1e-10)throw std::runtime_error("collision cost/gradient index parity failed");
    auto plan=planner.plan(task);
    report["source_validation"]=plan.source_validation;
    for(const auto &point:plan.diagnostic_search_path) {
      YAML::Node sample;for(double value:point)sample.push_back(value);
      report["search_path"].push_back(sample);
    }
    for(const auto &point:plan.diagnostic_minco_seed) {
      YAML::Node sample;sample["time_s"]=point.time_s;
      for(double value:point.position)sample["position"].push_back(value);
      report["minco_seed_points"].push_back(sample);
    }
    for(const auto &point:plan.diagnostic_optimized_candidate) {
      YAML::Node sample;sample["time_s"]=point.time_s;
      for(double value:point.position)sample["position"].push_back(value);
      report["optimized_candidate_points"].push_back(sample);
    }
    report["world_signature"]=scene.signature();report["model_signature"]=task.model_signature;
    report["world_boxes"]=scene.boxes().size();report["map_points"]=cloud->width*cloud->height;
    report["planning_wall_s"]=plan.planning_wall_s;
    report["search_wall_s"]=plan.search_wall_s;report["optimization_wall_s"]=plan.optimization_wall_s;
    report["source_check_wall_s"]=plan.source_check_wall_s;report["cost_evaluations"]=plan.cost_evaluations;
    report["startup_ramp_duration_s"]=plan.start_transition_duration_s;
    report["startup_ramp_distance_m"]=plan.start_transition_distance_m;
    report["optimization_budget_hit"]=plan.optimization_budget_hit;
    report["candidate_generated"]=plan.failure.empty() && !plan.points.empty();
    report["failure"]=plan.failure;report["candidate_samples"]=plan.points.size();
    report["validation_passed"]=YAML::Null;report["execution_started"]=false;report["execution_completed"]=false;
    if(!plan.failure.empty())throw std::runtime_error(plan.failure);
    if(!plan.trajectory || plan.points.size()<2)throw std::runtime_error("missing exact polynomial trajectory");
    int replans=0;node->get_parameter_or("probe.warm_replans",replans,0);
    auto previous=plan;
    for(int i=0;i<replans;++i) {
      const double advance=std::min(.1,previous.trajectory->duration()*.05);
      const auto anchor=previous.trajectory->evaluate(advance);
      auto moving=previous.task;moving.start=anchor.position;moving.start_velocity=anchor.velocity;
      moving.start_acceleration=anchor.acceleration;moving.moving_start=true;
      moving.start_jerk=anchor.jerk;
      const auto replanned=planner.replan(moving,previous.trajectory,advance);
      YAML::Node sample;sample["planning_ms"]=replanned.planning_wall_s*1000.;sample["search_ms"]=replanned.search_wall_s*1000.;
      sample["optimization_ms"]=replanned.optimization_wall_s*1000.;sample["source_check_ms"]=replanned.source_check_wall_s*1000.;
      sample["failure"]=replanned.failure;sample["budget_hit"]=replanned.optimization_budget_hit;
      if(!replanned.failure.empty()){report["warm_replans"].push_back(sample);throw std::runtime_error("warm replan: "+replanned.failure);}
      risk_mm::SampledValidator warm_validator(mesh,limits,moving.model_signature,moving.map_revision);std::string invalid;
      const bool checked=warm_validator.validate(replanned,invalid);sample["raw_public_validation"]=checked;sample["validation_reason"]=invalid;
      report["warm_replans"].push_back(sample);
      if(!checked)throw std::runtime_error("warm replan public validation: "+invalid);
      previous=replanned;
    }
    for(int k=0;k<=100;++k) {
      const auto state=plan.trajectory->evaluate(plan.trajectory->duration()*k/100.);
      const double longitudinal=std::cos(state.position[2])*state.velocity[0]+std::sin(state.position[2])*state.velocity[1];
      const double lateral=-std::sin(state.position[2])*state.velocity[0]+std::cos(state.position[2])*state.velocity[1];
      if(std::abs(longitudinal-state.forward_velocity)>1e-7 || std::abs(lateral)>1e-7)
        throw std::runtime_error("trajectory body velocity disagrees with public CAD heading");
    }
    report["public_frame_velocity_consistency_passed"]=true;
    report["duration_s"]=plan.trajectory->duration();report["trajectory_source"]="original optimized polynomials; no sample interpolation";
    for(const auto &point:plan.points) {
      YAML::Node sample;sample["time_s"]=point.time_s;
      const auto exact=plan.trajectory->evaluate(point.time_s);
      for(double value:exact.position)sample["position"].push_back(value);
      for(double value:exact.velocity)sample["velocity"].push_back(value);
      for(double value:exact.acceleration)sample["acceleration"].push_back(value);
      report["points"].push_back(sample);
    }
    report["optimized_points"]=YAML::Clone(report["points"]);
    report["raw_points"]=YAML::Clone(report["points"]);
    const auto terminal=plan.trajectory->evaluate(plan.trajectory->duration());
    for(double value:terminal.position)report["terminal_position"].push_back(value);
    for(double value:terminal.velocity)report["terminal_velocity"].push_back(value);
    for(double value:terminal.acceleration)report["terminal_acceleration"].push_back(value);
    risk_mm::SampledValidator validator(mesh,limits,task.model_signature,task.map_revision);
    bool valid=validator.validate(plan,reason);
    report["raw_validation_passed"]=valid;report["raw_validation_reason"]=reason;
    report["raw_duration_s"]=plan.trajectory->duration();report["time_scale"]=1.;
    const auto raw_plan=plan;
    bool adapt=false;double maximum=1.;node->get_parameter_or("coupled_time_scaling.enabled",adapt,false);
    node->get_parameter_or("coupled_time_scaling.max_scale",maximum,1.);
    std::string profile;node->get_parameter_or("coupled_time_scaling.profile",profile,std::string("uniform"));
    const auto timing_begin=std::chrono::steady_clock::now();
    if(adapt && (profile=="through" || (valid && profile=="local") || (!valid && risk_mm::timingViolation(reason)))) {
      risk_mm::TimeAdaptation timing;auto adjusted=risk_mm::adaptCandidateTime(plan,limits,maximum,profile,timing,reason);
      report["timing_profile"]=profile;report["time_scale"]=timing.duration_ratio;report["endpoint_time_scale"]=timing.endpoint_scale;
      report["maximum_local_time_scale"]=timing.maximum_scale;
      report["time_adaptation_wall_s"]=std::chrono::duration<double>(std::chrono::steady_clock::now()-timing_begin).count();
      report["first_violation_constraint"]=timing.first_violation_constraint;
      report["first_violation_time_s"]=timing.first_violation_time_s;
      report["first_violation_ratio"]=timing.first_violation_ratio;
      report["best_feasible_scale_candidate"]=timing.best_scale;
      report["best_scale_max_constraint"]=timing.best_constraint;
      report["best_scale_max_time_s"]=timing.best_time_s;
      report["best_scale_max_ratio"]=timing.best_ratio;
      report["source_time_knots"]=timing.source_knots;report["execution_time_knots"]=timing.time_knots;report["local_time_scale_knots"]=timing.scale_knots;
      if(adjusted){plan=*adjusted;valid=validator.validate(plan,reason);}else valid=false;
      if(adjusted && profile=="local") {
        double geometry_error=0.,velocity_error=0.,acceleration_error=0.;
        for(size_t i=0;i<timing.source_knots.size();++i) {
          const auto raw=raw_plan.trajectory->evaluate(timing.source_knots[i]);
          const auto mapped=plan.trajectory->evaluate(timing.time_knots[i]);
          for(int j=0;j<9;++j)geometry_error=std::max(geometry_error,std::abs(j==2?
            std::remainder(raw.position[j]-mapped.position[j],2*M_PI):raw.position[j]-mapped.position[j]));
        }
        for(int k=1;k<150;++k) {
          const double t=plan.trajectory->duration()*k/150.,epsilon=1e-5;
          const auto at=plan.trajectory->evaluate(t),lo=plan.trajectory->evaluate(t-epsilon),hi=plan.trajectory->evaluate(t+epsilon);
          for(int j=0;j<9;++j) {
            const double dp=j==2?std::remainder(hi.position[j]-lo.position[j],2*M_PI):hi.position[j]-lo.position[j];
            velocity_error=std::max(velocity_error,std::abs(dp/(2*epsilon)-at.velocity[j]));
            acceleration_error=std::max(acceleration_error,std::abs((hi.velocity[j]-lo.velocity[j])/(2*epsilon)-at.acceleration[j]));
          }
        }
        report["local_geometry_error"]=geometry_error;report["local_velocity_fd_error"]=velocity_error;report["local_acceleration_fd_error"]=acceleration_error;
        if(geometry_error>1e-7 || velocity_error>1e-5 || acceleration_error>1e-4)throw std::runtime_error("local time geometry/derivative regression failed");
        double uniform=1.;std::string benchmark_reason;
        auto benchmark=risk_mm::scaleCandidateTime(raw_plan,limits,maximum,uniform,benchmark_reason);
        if(benchmark)report["uniform_reference_duration_s"]=benchmark->trajectory->duration();
      }
    }
    if(report["time_adaptation_wall_s"].IsNull())
      report["time_adaptation_wall_s"]=std::chrono::duration<double>(std::chrono::steady_clock::now()-timing_begin).count();
    report["points"]=YAML::Node(YAML::NodeType::Sequence);
    report["duration_s"]=plan.trajectory->duration();
    for(const auto &point:plan.points) {
      YAML::Node sample;sample["time_s"]=point.time_s;const auto exact=plan.trajectory->evaluate(point.time_s);
      for(double value:exact.position)sample["position"].push_back(value);
      for(double value:exact.velocity)sample["velocity"].push_back(value);
      for(double value:exact.acceleration)sample["acceleration"].push_back(value);
      report["points"].push_back(sample);
    }
    report["validation_passed"]=valid;report["validation_reason"]=reason;
    const auto &checked=validator.report();
    report["validation_samples"]=checked.samples;report["validation_wall_s"]=checked.wall_s;
    report["max_arm_velocity"]=checked.max_arm_velocity;report["max_arm_acceleration"]=checked.max_arm_acceleration;
    report["max_wheel_velocity"]=checked.max_wheel_velocity;report["max_wheel_acceleration"]=checked.max_wheel_acceleration;
    report["max_steer_rate"]=checked.max_steer_rate;
    for(double value:checked.end_steer)report["end_steer"].push_back(value);
    for(double value:checked.end_wheel_position)report["end_wheel_position"].push_back(value);
    if(!valid)throw std::runtime_error(reason);
    bool run_regression=true;node->get_parameter_or("probe.run_time_scaling_regression",run_regression,true);
    if(run_regression) {
    // Exercise physical timing adaptation separately from the production
    // limits: tighten wheel acceleration in this regression fixture only.
    auto tightened=limits;tightened.wheel_acceleration=std::max(.001,checked.max_wheel_acceleration/4.);
    risk_mm::SampledValidator tight_validator(mesh,tightened,task.model_signature,task.map_revision);
    if(tight_validator.validate(plan,reason) || !risk_mm::timingViolation(reason))
      throw std::runtime_error("timing regression fixture must exceed its tighter acceleration limit");
    double factor=1.;auto slowed=risk_mm::scaleCandidateTime(plan,tightened,8.,factor,reason);
    if(!slowed)throw std::runtime_error("time adaptation regression: "+reason);
    bool moving_boundary=false;
    if(plan.task.moving_start || plan.task.require_start_boundary)for(int j=0;j<9;++j)
      moving_boundary=moving_boundary || std::abs(plan.task.start_velocity[j])>1e-6 ||
          std::abs(plan.task.start_acceleration[j])>1e-6;
    if(moving_boundary) {
      if(tight_validator.validate(*slowed,reason) || reason.rfind("moving boundary mismatch",0)!=0)
        throw std::runtime_error("uniform timing must reject the original moving-boundary task: "+reason);
      report["uniform_moving_boundary_rejected"]=true;
    }
    // This is an independent uniform-scaling unit fixture with its own input
    // derivatives. The production moving task above keeps its original p/v/a.
    auto fixture=*slowed;
    for(int j=0;j<9;++j) {
      fixture.task.start_velocity[j]/=factor;
      fixture.task.start_acceleration[j]/=factor*factor;
      fixture.task.start_jerk[j]/=factor*factor*factor;
    }
    if(!tight_validator.validate(fixture,reason))throw std::runtime_error("time adaptation regression: "+reason);
    for(int k=0;k<=20;++k) {
      const double t=plan.trajectory->duration()*k/20.;
      const auto raw=plan.trajectory->evaluate(t),adjusted=slowed->trajectory->evaluate(t*factor);
      for(int j=0;j<9;++j)
        if(std::abs(j==2?std::remainder(raw.position[j]-adjusted.position[j],2*M_PI):raw.position[j]-adjusted.position[j])>1e-9 ||
           std::abs(raw.velocity[j]/factor-adjusted.velocity[j])>1e-9 ||
           std::abs(raw.acceleration[j]/factor/factor-adjusted.acceleration[j])>1e-9)
          throw std::runtime_error("time adaptation changed polynomial geometry or derivatives");
    }
    report["time_scaling_regression_passed"]=true;report["regression_time_scale"]=factor;report["regression_wheel_acceleration_limit"]=tightened.wheel_acceleration;
    }else report["time_scaling_regression_passed"]=YAML::Null;
    report["status"]="passed";
  } catch(const std::exception &error) {report["reason"]=error.what();}
  if(!output.empty()) {std::ofstream file(output);file<<report<<'\n';}
  std::cout<<"coupled_probe status="<<report["status"].as<std::string>()<<" report="<<output<<std::endl;
  rclcpp::shutdown();return report["status"].as<std::string>()=="passed"?0:1;
}
