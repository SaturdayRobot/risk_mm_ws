#include "mm_bringup/sim_model.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tinyxml2.h>
#include <yaml-cpp/yaml.h>
#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace risk_mm {
namespace {
using namespace tinyxml2;
using Vec = std::array<double, 3>;
std::string text(const Vec &v) {
  std::ostringstream out; out.precision(12); out << v[0] << ' ' << v[1] << ' ' << v[2]; return out.str();
}
Vec vector(const YAML::Node &node) {
  if (!node.IsSequence() || node.size()!=3) throw std::runtime_error("Expected three collision coordinates");
  Vec out{}; for (size_t i=0;i<3;++i) {
    out[i]=node[i].as<double>(); if(!std::isfinite(out[i])) throw std::runtime_error("Nonfinite collision parameter");
  } return out;
}
XMLElement *add(XMLDocument &doc, XMLElement *parent, const char *tag) {
  auto *element=doc.NewElement(tag); parent->InsertEndChild(element); return element;
}
void box(XMLDocument &doc, XMLElement *link, const Vec &size, const Vec &center) {
  for(double v:size) if(v<=0) throw std::runtime_error("Nonpositive collision box");
  auto *collision=add(doc,link,"collision");
  add(doc,collision,"origin")->SetAttribute("xyz",text(center).c_str());
  add(doc,add(doc,collision,"geometry"),"box")->SetAttribute("size",text(size).c_str());
}
std::pair<Vec,Vec> bounds(const std::string &file) {
  std::ifstream stream(file,std::ios::binary);
  stream.seekg(80); uint32_t count=0; stream.read(reinterpret_cast<char*>(&count),4);
  if(!stream || count==0 || std::filesystem::file_size(file)!=84ULL+50ULL*count)
    throw std::runtime_error("Invalid binary STL: "+file);
  Vec low,high;low.fill(std::numeric_limits<double>::infinity());high.fill(-std::numeric_limits<double>::infinity());
  for(uint32_t i=0;i<count;++i) {
    std::array<float,12> row{};uint16_t attr;
    stream.read(reinterpret_cast<char*>(row.data()),48);stream.read(reinterpret_cast<char*>(&attr),2);
    for(int vertex=1;vertex<4;++vertex)for(int axis=0;axis<3;++axis) {
      double value=row[3*vertex+axis];if(!std::isfinite(value))throw std::runtime_error("Invalid STL vertex");
      low[axis]=std::min(low[axis],value);high[axis]=std::max(high[axis],value);
    }
  }
  if(!stream)throw std::runtime_error("Truncated STL: "+file);
  return {low,high};
}
}

std::string prepare_simulation_model() {
  const auto share=[](const std::string &name){return ament_index_cpp::get_package_share_directory(name);};
  const auto bringup=share("mm_bringup");
  const auto config=YAML::LoadFile(bringup+"/config/simulation_collision.yaml");
  const auto robot=YAML::LoadFile(bringup+"/config/robot.yaml");
  if(config["mesh_proxy"].as<std::string>()!="local_aabb")throw std::runtime_error("Unsupported contact proxy");
  XMLDocument doc;
  if(doc.LoadFile((share("rangermini_description")+"/urdf/rangermini_xarm6.urdf").c_str())!=XML_SUCCESS)
    throw std::runtime_error("Cannot load local robot URDF");
  auto *root=doc.RootElement();
  auto resolve=[&](XMLElement *mesh) {
    std::string uri=mesh->Attribute("filename");const std::string prefix="package://";
    if(uri.rfind(prefix,0)!=0)throw std::runtime_error("Expected local package mesh URI");
    auto slash=uri.find('/',prefix.size());
    auto file=share(uri.substr(prefix.size(),slash-prefix.size()))+uri.substr(slash);
    if(!std::filesystem::is_regular_file(file))throw std::runtime_error("Missing mesh: "+file);
    mesh->SetAttribute("filename",("file://"+file).c_str());
  };
  for(auto *link=root->FirstChildElement("link");link;link=link->NextSiblingElement("link"))
    for(auto tag:{"visual","collision"})for(auto *e=link->FirstChildElement(tag);e;e=e->NextSiblingElement(tag)) {
      auto *g=e->FirstChildElement("geometry");if(g && g->FirstChildElement("mesh"))resolve(g->FirstChildElement("mesh"));
    }
  for(auto *joint=root->FirstChildElement("ros2_control")->FirstChildElement("joint");joint;joint=joint->NextSiblingElement("joint")) {
    double value=0.;std::string name=joint->Attribute("name");
    for(size_t i=0;i<robot["arm_joint_names"].size();++i)
      if(robot["arm_joint_names"][i].as<std::string>()==name)value=robot["display_arm"][i].as<double>();
    for(auto *s=joint->FirstChildElement("state_interface");s;s=s->NextSiblingElement("state_interface"))
      if(std::string(s->Attribute("name"))=="position") {
        auto *p=add(doc,s,"param");p->SetAttribute("name","initial_value");p->SetText(value);
      }
  }
  for(auto *g=root->FirstChildElement("gazebo");g;g=g->NextSiblingElement("gazebo")) {
    if(auto *self=g->FirstChildElement("selfCollide"))self->SetText(config["self_collision"].as<bool>());
    for(auto *p=g->FirstChildElement("plugin");p;p=p->NextSiblingElement("plugin"))
      if(std::string(p->Attribute("name"))=="gazebo_ros2_control")
        p->FirstChildElement("parameters")->SetText((bringup+"/config/ros2_control.yaml").c_str());
    for(auto *s=g->FirstChildElement("sensor");s;s=s->NextSiblingElement("sensor"))add(doc,s,"always_on")->SetText(true);
  }
  for(auto *link=root->FirstChildElement("link");link;link=link->NextSiblingElement("link")) {
    const std::string name=link->Attribute("name");
    if(name=="base_link") {
      while(auto *c=link->FirstChildElement("collision"))link->DeleteChild(c);
      for(const auto &b:config["chassis_boxes"])box(doc,link,vector(b["size"]),vector(b["center"]));
      for(const auto &wheel:{"fl","fr","rl","rr"}) {
        const std::string joint_name=std::string("joint_steer_")+wheel;
        for(auto *j=root->FirstChildElement("joint");j;j=j->NextSiblingElement("joint"))if(joint_name==j->Attribute("name")) {
          Vec center{};std::istringstream(j->FirstChildElement("origin")->Attribute("xyz"))>>center[0]>>center[1]>>center[2];
          center[2]=config["steering_motor_center_z"].as<double>();box(doc,link,vector(config["steering_motor_box_size"]),center);
        }
      }continue;
    }
    for(auto *c=link->FirstChildElement("collision");c;c=c->NextSiblingElement("collision")) {
      auto *g=c->FirstChildElement("geometry");auto *mesh=g->FirstChildElement("mesh");if(!mesh)continue;
      auto [low,high]=bounds(std::string(mesh->Attribute("filename")).substr(7));
      Vec size{},center{};for(int i=0;i<3;++i){size[i]=high[i]-low[i];center[i]=(high[i]+low[i])/2;}
      auto *origin=c->FirstChildElement("origin");if(!origin)origin=add(doc,c,"origin");
      for(const auto *key:{"xyz","rpy"})if(origin->Attribute(key) && std::string(origin->Attribute(key))!="0 0 0")
        throw std::runtime_error("Transformed collision mesh requires an explicit proxy: "+name);
      g->DeleteChild(mesh);
      if(name.rfind("wheel_",0)==0) {
        auto tire=config["tire"];auto *cylinder=add(doc,g,"cylinder");
        cylinder->SetAttribute("radius",tire["radius"].as<double>());cylinder->SetAttribute("length",tire["width"].as<double>());
        origin->SetAttribute("xyz",text({0.,tire["center_y"][name.substr(6)].as<double>(),0.}).c_str());
        origin->SetAttribute("rpy","1.5707963267948966 0 0");
        auto *properties=add(doc,root,"gazebo");properties->SetAttribute("reference",name.c_str());
        for(auto *key:{"mu1","mu2","kp","kd","minDepth","maxVel"})add(doc,properties,key)->SetText(tire[key].as<double>());
      } else {
        origin->SetAttribute("xyz",text(center).c_str());add(doc,g,"box")->SetAttribute("size",text(size).c_str());
      }
    }
  }
  XMLPrinter printer;doc.Print(&printer);return printer.CStr();
}
}
