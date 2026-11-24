#include "risk_mm_runtime/world_scene.hpp"
#include <openssl/evp.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tinyxml2.h>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <set>
#include <sstream>
#include <stdexcept>

namespace risk_mm {
namespace {
Eigen::Vector3d vector(const char *text) {
  if(!text)throw std::runtime_error("Missing scene vector");
  Eigen::Vector3d out;
  std::istringstream input(text);
  if(!(input>>out.x()>>out.y()>>out.z()) || !out.allFinite())throw std::runtime_error("Invalid scene vector");
  return out;
}
Eigen::Isometry3d pose(const tinyxml2::XMLElement *element) {
  Eigen::Isometry3d out=Eigen::Isometry3d::Identity();
  if(!element)return out;
  if(element->Attribute("relative_to"))throw std::runtime_error("Relative SDF poses are not supported in this static world adapter");
  double x,y,z,r,p,w;
  std::istringstream input(element->GetText()?element->GetText():"");
  if(!(input>>x>>y>>z>>r>>p>>w))throw std::runtime_error("Invalid world pose");
  out.translation()=Eigen::Vector3d(x,y,z);
  out.linear()=(Eigen::AngleAxisd(w,Eigen::Vector3d::UnitZ())*
                Eigen::AngleAxisd(p,Eigen::Vector3d::UnitY())*
                Eigen::AngleAxisd(r,Eigen::Vector3d::UnitX())).toRotationMatrix();
  if(!out.matrix().allFinite())throw std::runtime_error("Nonfinite world pose");
  return out;
}
}

std::string content_sha256(const std::string &file) {
  std::ifstream input(file,std::ios::binary);
  if(!input)throw std::runtime_error("Missing scene resource: "+file);
  const std::string data((std::istreambuf_iterator<char>(input)),{});
  unsigned char digest[EVP_MAX_MD_SIZE];unsigned int size=0;
  if(EVP_Digest(data.data(),data.size(),digest,&size,EVP_sha256(),nullptr)!=1)
    throw std::runtime_error("Cannot hash scene resource");
  std::ostringstream out;
  for(unsigned int i=0;i<size;++i)out<<std::hex<<std::setw(2)<<std::setfill('0')<<static_cast<unsigned>(digest[i]);
  return out.str();
}

WorldScene::WorldScene(const std::string &world_file,const std::string &metadata_file) {
  signature_=content_sha256(world_file);
  const auto metadata=YAML::LoadFile(metadata_file);
  if(metadata["world_sha256"].as<std::string>()!=signature_)throw std::runtime_error("World/metadata version mismatch");
  const auto bounds=metadata["map_bounds"];
  if(bounds.size()!=6)throw std::runtime_error("Missing bounded free-space semantics");
  for(size_t i=0;i<6;++i)bounds_[i]=bounds[i].as<double>();
  if(!(bounds_[0]<bounds_[1] && bounds_[2]<bounds_[3] && bounds_[4]<bounds_[5]))
    throw std::runtime_error("Invalid world bounds");
  tinyxml2::XMLDocument xml;
  if(xml.LoadFile(world_file.c_str())!=tinyxml2::XML_SUCCESS)throw std::runtime_error("Cannot parse selected world");
  auto *sdf=xml.FirstChildElement("sdf");
  auto *world=sdf?sdf->FirstChildElement("world"):nullptr;
  if(!world)throw std::runtime_error("Missing SDF world");
  bool floor=false;
  for(auto *model=world->FirstChildElement("model");model;model=model->NextSiblingElement("model")) {
    const std::string name=model->Attribute("name");
    if(!model->FirstChildElement("static") || !model->FirstChildElement("static")->BoolText())
      throw std::runtime_error("Unmodelled dynamic world object: "+name);
    const auto model_pose=pose(model->FirstChildElement("pose"));
    models_.emplace(name,model_pose);
    for(auto *link=model->FirstChildElement("link");link;link=link->NextSiblingElement("link")) {
      const auto link_pose=model_pose*pose(link->FirstChildElement("pose"));
      for(auto *collision=link->FirstChildElement("collision");collision;collision=collision->NextSiblingElement("collision")) {
        const auto transform=link_pose*pose(collision->FirstChildElement("pose"));
        const auto *geometry=collision->FirstChildElement("geometry");
        if(!geometry)throw std::runtime_error("Missing world collision geometry");
        if(auto *shape=geometry->FirstChildElement("box")) {
          const auto size=vector(shape->FirstChildElement("size")->GetText());
          if((size.array()<=0).any())throw std::runtime_error("Invalid world collision box");
          boxes_.push_back({name+"/"+collision->Attribute("name"),transform.translation(),transform.linear().cwiseAbs()*size});
        } else if(auto *plane=geometry->FirstChildElement("plane")) {
          if(floor || (transform.linear()*vector(plane->FirstChildElement("normal")->GetText())-Eigen::Vector3d::UnitZ()).norm()>1e-8)
            throw std::runtime_error("Unsupported support-plane geometry");
          floor=true;floor_z_=transform.translation().z();
        } else throw std::runtime_error("Unsupported collision shape in complete world snapshot");
      }
    }
  }
  if(!floor || boxes_.empty() || metadata["collision_boxes"].size()!=boxes_.size())
    throw std::runtime_error("Incomplete scene or missing support plane");
}

unsigned long WorldScene::revision() const {return std::stoul(signature_.substr(0,12),nullptr,16);}

bool WorldScene::matches(const gazebo_msgs::msg::ModelStates &states,std::string &reason) const {
  if(states.name.size()!=states.pose.size()){reason="incomplete Gazebo model state";return false;}
  std::set<std::string> seen;
  for(size_t i=0;i<states.name.size();++i) {
    const auto &name=states.name[i];
    if(name=="rangermini")continue;
    const auto expected=models_.find(name);
    if(expected==models_.end()){reason="unknown Gazebo object: "+name;return false;}
    const auto &p=states.pose[i].position;const auto &q=states.pose[i].orientation;
    Eigen::Quaterniond rotation(q.w,q.x,q.y,q.z);
    if(!Eigen::Vector3d(p.x,p.y,p.z).allFinite() || !rotation.coeffs().allFinite() || std::abs(rotation.norm()-1.)>1e-4 ||
       (Eigen::Vector3d(p.x,p.y,p.z)-expected->second.translation()).norm()>1e-4 ||
       rotation.angularDistance(Eigen::Quaterniond(expected->second.linear()))>1e-4) {
      reason="world object pose changed: "+name;return false;
    }
    if(!seen.insert(name).second){reason="duplicate scene object";return false;}
  }
  if(seen.size()!=models_.size()){reason="missing world objects";return false;}
  reason.clear();return true;
}

sensor_msgs::msg::PointCloud2::SharedPtr WorldScene::cloud(double resolution,const std_msgs::msg::Header &header) const {
  if(!std::isfinite(resolution)||resolution<=0)throw std::runtime_error("Invalid scene grid resolution");
  std::vector<Eigen::Vector3f> points;
  const Eigen::Vector3d origin(bounds_[0],bounds_[2],floor_z_-.05);
  const Eigen::Vector3d maximum(bounds_[1],bounds_[3],bounds_[5]-.05);
  for(const auto &box:boxes_) {
    const Eigen::Vector3d low=(box.center-box.size/2).cwiseMax(origin);
    const Eigen::Vector3d high=(box.center+box.size/2).cwiseMin(maximum-Eigen::Vector3d::Constant(1e-6));
    if((low.array()>high.array()).any())continue;
    const Eigen::Vector3i first=((low-origin)/resolution).array().floor().cast<int>();
    const Eigen::Vector3i last=((high-origin)/resolution).array().floor().cast<int>();
    for(int x=first.x();x<=last.x();++x)for(int y=first.y();y<=last.y();++y)for(int z=first.z();z<=last.z();++z)
      points.push_back((origin+resolution*(Eigen::Vector3d(x,y,z)+Eigen::Vector3d::Constant(.5))).cast<float>());
  }
  auto result=std::make_shared<sensor_msgs::msg::PointCloud2>();result->header=header;
  sensor_msgs::PointCloud2Modifier modifier(*result);modifier.setPointCloud2FieldsByString(1,"xyz");modifier.resize(points.size());
  sensor_msgs::PointCloud2Iterator<float> x(*result,"x"),y(*result,"y"),z(*result,"z");
  for(const auto &point:points){*x=point.x();*y=point.y();*z=point.z();++x;++y;++z;}
  return result;
}
}
