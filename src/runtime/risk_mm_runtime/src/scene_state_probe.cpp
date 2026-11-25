#include "risk_mm_runtime/validation.hpp"
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <mm_model/wheel_kinematics.hpp>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>

namespace {
template<size_t N> std::array<double,N> finiteArray(const YAML::Node &node) {
  if(!node.IsSequence() || node.size()!=N)throw std::runtime_error("invalid state vector dimension");
  std::array<double,N> out;
  for(size_t i=0;i<N;++i) {
    out[i]=node[i].as<double>();
    if(!std::isfinite(out[i]))throw std::runtime_error("nonfinite state vector");
  }
  return out;
}
}
// Batch static geometry inspection for validation fixtures, not a planner.
int main(int argc,char **argv) {
  if(argc!=3){std::cerr<<"usage: scene_state_probe input.yaml report.yaml\n";return 2;}
  rclcpp::init(argc,argv);
  YAML::Node report;report["status"]="failed";report["scope"]="independent full URDF mesh state checks; no path planning, execution or continuous safety claim";
  int result=1;
  try {
    const auto input=YAML::LoadFile(argv[1]);
    const auto world_file=input["world_file"].as<std::string>();
    const auto metadata=input["world_metadata_file"].as<std::string>();
    const auto urdf=input["robot_urdf_file"].as<std::string>(),srdf=input["robot_srdf_file"].as<std::string>();
    const double padding=input["collision_padding"].as<double>();
    if(!std::isfinite(padding) || padding<0.)throw std::runtime_error("invalid collision padding");
    risk_mm::WorldScene world(world_file,metadata);
    const int workers=input["batch_workers"]?input["batch_workers"].as<int>():1;
    risk_mm::MeshScene scene(world,urdf,srdf,padding,workers);report["batch_workers"]=workers;
    report["world_sha256"]=world.signature();report["robot_urdf_sha256"]=risk_mm::content_sha256(urdf);
    report["robot_srdf_sha256"]=risk_mm::content_sha256(srdf);report["collision_padding"]=padding;
    const auto states=input["states"];
    if(!states.IsSequence() || states.size()==0)throw std::runtime_error("nonempty states fixture required");
    size_t accepted=0;const auto begin=std::chrono::steady_clock::now();
    std::array<double,4> previous_steer=input["initial_steer"]?finiteArray<4>(input["initial_steer"]):std::array<double,4>{};
    std::array<double,4> wheel_position=input["initial_wheel_position"]?finiteArray<4>(input["initial_wheel_position"]):std::array<double,4>{};
    std::array<double,4> previous_speed{};
    std::vector<risk_mm::MeshState> samples;
    double previous_time=0.;bool first=true;
    for(const auto &entry:states) {
      const auto position=finiteArray<9>(entry["position"]);
      auto steer=entry["steer"]?finiteArray<4>(entry["steer"]):std::array<double,4>{};
      auto wheels=entry["wheel_position"]?finiteArray<4>(entry["wheel_position"]):std::array<double,4>{};
      if(entry["velocity"]) {
        const auto velocity=finiteArray<9>(entry["velocity"]);
        const auto contacts=finiteArray<8>(input["wheel_positions"]);
        const double radius=input["wheel_radius"].as<double>(),time=entry["time_s"].as<double>();
        if(!std::isfinite(time) || time<previous_time || !std::isfinite(radius) || radius<=0.)throw std::runtime_error("invalid counterfactual trajectory time/wheel radius");
        const double vx=std::cos(position[2])*velocity[0]+std::sin(position[2])*velocity[1];
        const auto solution=mm_model::inverse(vx,0.,velocity[2],contacts,radius,std::numeric_limits<double>::infinity(),previous_steer);
        const double dt=time-previous_time;
        if(!first)for(size_t j=0;j<4;++j)wheel_position[j]+=.5*(previous_speed[j]+solution.speed[j])*dt;
        steer=solution.steer;wheels=wheel_position;previous_steer=steer;previous_speed=solution.speed;previous_time=time;first=false;
      }
      samples.push_back({position,steer,wheels});
    }
    const auto reasons=scene.checkBatch(samples);
    for(size_t i=0;i<samples.size();++i) {
      const bool valid=reasons[i].empty();
      YAML::Node item;item["id"]=states[i]["id"].as<std::string>();item["valid"]=valid;item["reason"]=reasons[i];
      item["position"]=samples[i].position;report["states"].push_back(item);accepted+=valid;
    }
    report["geometry_wall_s"]=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
    report["accepted_states"]=accepted;report["rejected_states"]=states.size()-accepted;
    // Rejections are data, not tool errors. The fixture consumer asserts its
    // expected positives and negatives explicitly.
    report["status"]="completed";result=0;
  }catch(const std::exception &error){report["reason"]=error.what();}
  std::ofstream out(argv[2]);out<<report<<'\n';
  if(!out){std::cerr<<"cannot write geometry report\n";result=1;}
  std::cout<<"scene_state_probe status="<<report["status"].as<std::string>()<<" report="<<argv[2]<<'\n';
  rclcpp::shutdown();return result;
}
