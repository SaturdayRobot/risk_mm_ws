#include "risk_mm_runtime/validation.hpp"
#include <mm_model/wheel_kinematics.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <geometric_shapes/shapes.h>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>
#include <tinyxml2.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <set>
#include <sstream>
#include <mutex>
#include <thread>
#include <atomic>

namespace risk_mm {
namespace {
std::string read(const std::string &file) {
  std::ifstream input(file);
  if(!input)throw std::runtime_error("Missing collision model: "+file);
  return std::string(std::istreambuf_iterator<char>(input),{});
}
double angle_error(double first,double second) {return std::remainder(first-second,2.*M_PI);}
}

class MeshScene::Implementation {
 public:
  moveit::core::RobotModelPtr model;
  std::shared_ptr<planning_scene::PlanningScene> scene;
  std::unique_ptr<moveit::core::RobotState> state;
  std::array<double,6> bounds;
  std::mutex mutex;
  bool valid(const std::array<double,9> &,const std::array<double,4> &,const std::array<double,4> &,
             std::string &,double z=0.,double roll=0.,double pitch=0.);
  Implementation(const WorldScene &world,const std::string &urdf_file,const std::string &srdf_file,double padding) {
    bounds=world.bounds();
    const auto urdf=urdf::parseURDF(read(urdf_file));
    if(!urdf || urdf->getName()!="rangermini_xarm6")throw std::runtime_error("Invalid current robot model");
    tinyxml2::XMLDocument semantic;
    const auto text=read(srdf_file);
    if(semantic.Parse(text.c_str())!=tinyxml2::XML_SUCCESS)throw std::runtime_error("Invalid collision semantics");
    // Floating validation root represents the measured world pose without
    // changing the physical URDF or publishing an alternate robot TF tree.
    auto *joint=semantic.NewElement("virtual_joint");joint->SetAttribute("name","validation_world_joint");
    joint->SetAttribute("type","floating");joint->SetAttribute("parent_frame","world");joint->SetAttribute("child_link","base_link");
    semantic.RootElement()->InsertEndChild(joint);
    tinyxml2::XMLPrinter printer;semantic.Print(&printer);
    auto srdf=std::make_shared<srdf::Model>();
    if(!srdf->initString(*urdf,printer.CStr()))throw std::runtime_error("Cannot load current SRDF");
    model=std::make_shared<moveit::core::RobotModel>(urdf,srdf);
    scene=std::make_shared<planning_scene::PlanningScene>(model);
    for(const auto &box:world.boxes()) {
      Eigen::Isometry3d pose=Eigen::Isometry3d::Identity();pose.translation()=box.center;
      scene->getWorldNonConst()->addToObject(box.name,std::make_shared<shapes::Box>(box.size.x(),box.size.y(),box.size.z()),pose);
    }
    Eigen::Isometry3d floor=Eigen::Isometry3d::Identity();floor.translation().z()=world.floorHeight();
    scene->getWorldNonConst()->addToObject("support_plane",std::make_shared<shapes::Plane>(0.,0.,1.,0.),floor);
    for(auto *wheel:{"wheel_fl","wheel_fr","wheel_rl","wheel_rr"})
      scene->getAllowedCollisionMatrixNonConst().setEntry("support_plane",wheel,true);
    scene->getCollisionEnvNonConst()->setPadding(padding);
    state=std::make_unique<moveit::core::RobotState>(model);state->setToDefaultValues();
  }
};

MeshScene::MeshScene(const WorldScene &world,const std::string &urdf,const std::string &srdf,double padding,size_t workers) {
  if(workers<1 || workers>4)throw std::invalid_argument("mesh batch workers must be in 1..4");
  for(size_t i=0;i<workers;++i)workers_.push_back(std::make_unique<Implementation>(world,urdf,srdf,padding));
}
MeshScene::~MeshScene()=default;

bool MeshScene::valid(const std::array<double,9> &position,const std::array<double,4> &steer,
                      const std::array<double,4> &wheels,std::string &reason,double z,double roll,double pitch) {
  return workers_.front()->valid(position,steer,wheels,reason,z,roll,pitch);
}
std::vector<std::string> MeshScene::checkBatch(const std::vector<MeshState> &states) {
  std::vector<std::string> results(states.size());std::atomic<size_t> next{0};
  auto run=[&](size_t worker) {
    for(size_t index=next.fetch_add(1);index<states.size();index=next.fetch_add(1)) {
      const auto &state=states[index];
      try {workers_[worker]->valid(state.position,state.steer,state.wheels,results[index]);}
      catch(const std::exception &error){results[index]="mesh exception: "+std::string(error.what());}
      catch(...){results[index]="unknown mesh exception";}
    }
  };
  std::vector<std::thread> threads;
  try {for(size_t worker=1;worker<workers_.size();++worker)threads.emplace_back(run,worker);}
  catch(...) {for(auto &thread:threads)thread.join();throw;}
  run(0);for(auto &thread:threads)thread.join();
  return results;
}
bool MeshScene::Implementation::valid(const std::array<double,9> &position,const std::array<double,4> &steer,
                      const std::array<double,4> &wheels,std::string &reason,double z,double roll,double pitch) {
  std::lock_guard<std::mutex> lock(mutex);
  for(double value:position)if(!std::isfinite(value)){reason="nonfinite collision state";return false;}
  Eigen::Isometry3d root=Eigen::Isometry3d::Identity();root.translation()=Eigen::Vector3d(position[0],position[1],z);
  root.linear()=(Eigen::AngleAxisd(position[2],Eigen::Vector3d::UnitZ())*
                 Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY())*
                 Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX())).toRotationMatrix();
  state->setJointPositions("validation_world_joint",root);
  for(int i=0;i<6;++i)state->setVariablePosition("xarm6_joint"+std::to_string(i+1),position[i+3]);
  const std::array<std::string,4> names{"fl","fr","rl","rr"};
  for(size_t i=0;i<4;++i) {
    if(!std::isfinite(steer[i]) || !std::isfinite(wheels[i])){reason="nonfinite wheel state";return false;}
    state->setVariablePosition("joint_steer_"+names[i],steer[i]);
    state->setVariablePosition("joint_wheel_"+names[i],wheels[i]);
  }
  state->update();
  std::vector<double> extent;
  state->computeAABB(extent);
  if(extent.size()!=6 || extent[0]<bounds[0] || extent[1]>bounds[1] ||
     extent[2]<bounds[2] || extent[3]>bounds[3] || extent[5]>bounds[5]) {
    reason="robot geometry outside complete world bounds";return false;
  }
  if(!state->satisfiesBounds()){reason="URDF joint bound violation";return false;}
  collision_detection::CollisionRequest request;request.contacts=true;request.max_contacts=1;
  // Ground clearance is checked by the actual support plane below. A rotated
  // tire's local mesh bounding box extends below its circular tread and is not
  // a valid lower-height bound. Only the four support tires may touch the floor.
  collision_detection::CollisionResult result;
  scene->checkCollision(request,result,*state);
  if(result.collision) {
    reason="full-mesh collision";
    if(!result.contacts.empty())reason+=" between "+result.contacts.begin()->first.first+" and "+result.contacts.begin()->first.second;
    return false;
  }
  reason.clear();return true;
}

SampledValidator::SampledValidator(std::shared_ptr<MeshScene> scene,ValidationLimits limits,std::string signature,unsigned long revision)
  :scene_(std::move(scene)),limits_(limits),model_signature_(std::move(signature)),map_revision_(revision) {}

SteeringAlignment initialSteeringAlignment(const Plan &plan,const ValidationLimits &limits) {
  SteeringAlignment alignment;alignment.target=plan.task.start_steer;
  if(plan.task.moving_start)return alignment;
  const double duration=plan.trajectory->duration();
  const int count=std::max(1,static_cast<int>(std::ceil(duration/limits.sample_s)));
  for(int k=0;k<=count;++k) {
    const auto value=plan.trajectory->evaluate(duration*k/count);
    if(std::abs(value.forward_velocity)+std::abs(value.velocity[2])<=1e-6)continue;
    alignment.target=mm_model::inverse(value.forward_velocity,0.,value.velocity[2],limits.wheel_positions,
        limits.wheel_radius,std::numeric_limits<double>::infinity(),plan.task.start_steer).steer;
    break;
  }
  for(size_t j=0;j<4;++j)
    alignment.duration_s=std::max(alignment.duration_s,std::abs(alignment.target[j]-plan.task.start_steer[j])/limits.steer_rate);
  return alignment;
}

bool SampledValidator::validate(const Plan &plan,std::string &reason) const {
  const auto started=std::chrono::steady_clock::now();report_={};
  auto finish=[&](bool passed,const std::string &why) {
    reason=why;report_.wall_s=std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();return passed;
  };
  if(model_signature_.empty() || plan.task.model_signature!=model_signature_ || plan.task.map_revision!=map_revision_)
    return finish(false,"model/map version mismatch");
  if(!plan.failure.empty() || !plan.trajectory || plan.points.size()<2)return finish(false,"missing polynomial candidate");
  const double duration=plan.trajectory->duration();
  if(!std::isfinite(duration)||duration<=0 || duration>120.)return finish(false,"invalid candidate duration");
  const auto start=plan.trajectory->evaluate(0.),end=plan.trajectory->evaluate(duration);
  for(int i=0;i<9;++i) {
    const double first=i==2?angle_error(start.position[i],plan.task.start[i]):start.position[i]-plan.task.start[i];
    const double last=i==2?angle_error(end.position[i],plan.task.goal[i]):end.position[i]-plan.task.goal[i];
    if(std::abs(first)>1e-4 || std::abs(last)>(i==2?.03:1e-4))return finish(false,"candidate endpoint mismatch");
    if((plan.task.moving_start || plan.task.require_start_boundary) &&
        (std::abs(start.velocity[i]-plan.task.start_velocity[i])>1e-4 ||
        std::abs(start.acceleration[i]-plan.task.start_acceleration[i])>.002))
      return finish(false,"moving boundary mismatch at state "+std::to_string(i)+" velocity="+std::to_string(start.velocity[i])+
        " expected="+std::to_string(plan.task.start_velocity[i])+" acceleration="+std::to_string(start.acceleration[i])+
        " expected="+std::to_string(plan.task.start_acceleration[i]));
  }
  // REMANI uses a terminal tangent to encode yaw. Validate the small
  // straight braking extension separately; the actuator enforces wheel ramp
  // limits and the executor requires a measured stop before completion.
  const double terminal_speed=std::hypot(end.velocity[0],end.velocity[1]);
  if((plan.task.stop_at_goal && terminal_speed>.020001) || std::abs(end.velocity[2])>1e-5)
    return finish(false,"unsupported terminal braking state");
  for(double steer:plan.task.start_steer)
    if(!std::isfinite(steer) || std::abs(steer)>M_PI_2+1e-6)return finish(false,"invalid initial steering state");
  const auto alignment=initialSteeringAlignment(plan,limits_);
  report_.initial_alignment_s=alignment.duration_s;report_.initial_steer=alignment.target;
  if(alignment.duration_s>0.) {
    const int steps=std::max(1,static_cast<int>(std::ceil(alignment.duration_s/limits_.sample_s)));
    for(int k=0;k<=steps;++k) {
      auto steer=plan.task.start_steer;
      for(size_t j=0;j<4;++j) {
        const double delta=alignment.target[j]-steer[j];
        steer[j]+=delta*double(k)/steps;
        report_.max_steer_rate=std::max(report_.max_steer_rate,std::abs(delta)/alignment.duration_s);
      }
      std::string collision;
      if(!scene_->valid(plan.task.start,steer,plan.task.start_wheel_position,collision))
        return finish(false,"stationary steering alignment: "+collision);
      ++report_.samples;
    }
  }
  std::set<double> times;
  const int count=static_cast<int>(std::ceil(duration/limits_.sample_s));
  for(int k=0;k<=count;++k)times.insert(duration*k/count);
  for(double boundary:plan.trajectory->boundaries()) {
    times.insert(boundary);
    if(boundary>1e-6 && boundary<duration-1e-6) {
      const auto left=plan.trajectory->evaluate(boundary-1e-6),right=plan.trajectory->evaluate(boundary+1e-6);
      for(int j=0;j<9;++j) {
        const double position_error=j==2?angle_error(left.position[j],right.position[j]):left.position[j]-right.position[j];
        if(std::abs(position_error)>.002)return finish(false,"phase position/heading discontinuity");
        if(std::abs(left.velocity[j]-right.velocity[j])>.01 || std::abs(left.acceleration[j]-right.acceleration[j])>.02)
          return finish(false,"phase velocity/acceleration discontinuity");
      }
    }
  }
  auto previous_steer=alignment.target;
  std::array<double,4> previous_speed{},wheels=plan.task.start_wheel_position;
  double previous_time=0.;bool first=true;
  std::vector<MeshState> mesh_states;std::vector<double> mesh_times;
  for(double time:times) {
    const auto value=plan.trajectory->evaluate(time);
    for(double x:value.position)if(!std::isfinite(x))return finish(false,"nonfinite position");
    for(double x:value.velocity)if(!std::isfinite(x))return finish(false,"nonfinite velocity");
    for(double x:value.acceleration)if(!std::isfinite(x))return finish(false,"nonfinite acceleration");
    if(std::hypot(value.velocity[0],value.velocity[1])>limits_.base_velocity+1e-6 || std::abs(value.velocity[2])>limits_.base_yaw_rate+1e-6)
      return finish(false,"base velocity bound violation");
    const double lateral=-std::sin(value.position[2])*value.velocity[0]+std::cos(value.position[2])*value.velocity[1];
    if(std::abs(lateral)>1e-5)return finish(false,"candidate requires unsupported lateral motion");
    for(int j=3;j<9;++j) {
      report_.max_arm_velocity=std::max(report_.max_arm_velocity,std::abs(value.velocity[j]));
      report_.max_arm_acceleration=std::max(report_.max_arm_acceleration,std::abs(value.acceleration[j]));
      if(std::abs(value.velocity[j])>limits_.arm_velocity+1e-6 || std::abs(value.acceleration[j])>limits_.arm_acceleration+1e-6)
        return finish(false,"arm velocity/acceleration bound violation");
    }
    const auto solution=mm_model::inverse(value.forward_velocity,0.,value.velocity[2],limits_.wheel_positions,
                                          limits_.wheel_radius,std::numeric_limits<double>::infinity(),previous_steer);
    const double dt=time-previous_time;
    for(size_t j=0;j<4;++j) {
      report_.max_wheel_velocity=std::max(report_.max_wheel_velocity,std::abs(solution.speed[j]));
      if(std::abs(solution.steer[j])>M_PI_2+1e-6 || std::abs(solution.speed[j])>limits_.wheel_velocity+1e-6)
        return finish(false,"four-wheel steering/drive bound violation");
      if(!first && dt>1e-10) {
        const double acceleration=std::abs(solution.speed[j]-previous_speed[j])/dt;
        const double rate=std::abs(solution.steer[j]-previous_steer[j])/dt;
        report_.max_wheel_acceleration=std::max(report_.max_wheel_acceleration,acceleration);
        report_.max_steer_rate=std::max(report_.max_steer_rate,rate);
        if(acceleration>limits_.wheel_acceleration+1e-5 || rate>limits_.steer_rate+1e-5)
          return finish(false,"four-wheel acceleration/steering-rate bound violation at t="+std::to_string(time)+
            " acceleration="+std::to_string(acceleration)+" steer_rate="+std::to_string(rate));
        wheels[j]+=.5*(solution.speed[j]+previous_speed[j])*dt;
      }
    }
    mesh_states.push_back({value.position,solution.steer,wheels});mesh_times.push_back(time);
    previous_steer=solution.steer;previous_speed=solution.speed;previous_time=time;first=false;
  }
  const auto collisions=scene_->checkBatch(mesh_states);report_.samples+=mesh_states.size();
  for(size_t i=0;i<collisions.size();++i)if(!collisions[i].empty())
    return finish(false,collisions[i]+" at t="+std::to_string(mesh_times[i]));
  if(plan.task.stop_at_goal && terminal_speed>1e-8) {
    const double deceleration=limits_.wheel_acceleration*limits_.wheel_radius;
    const double braking_time=terminal_speed/deceleration;
    for(int k=1;k<=3;++k) {
      const double t=braking_time*k/3.;auto pose=end.position;
      const double displacement=end.forward_velocity*t-std::copysign(.5*deceleration*t*t,end.forward_velocity);
      pose[0]+=displacement*std::cos(end.position[2]);pose[1]+=displacement*std::sin(end.position[2]);
      std::string collision;
      if(!scene_->valid(pose,previous_steer,wheels,collision))return finish(false,"terminal braking: "+collision);
      ++report_.samples;
    }
  }
  report_.end_steer=previous_steer;report_.end_wheel_position=wheels;
  return finish(true,"sampled full-mesh and four-wheel validation passed; no continuous-safety claim");
}
}
