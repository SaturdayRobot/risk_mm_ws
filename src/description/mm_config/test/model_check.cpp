#include "mm_config/mm_config.hpp"
#include <iostream>
#include <algorithm>
int main(int argc,char**argv) {
 rclcpp::init(argc,argv);rclcpp::NodeOptions options;options.automatically_declare_parameters_from_overrides(true);
 auto n=std::make_shared<rclcpp::Node>("model_check",options);rclcpp::Node::SharedPtr nh(n);
 try {
  remani_planner::MMConfig model;model.setParam(nh);
  std::vector<double> qv,expected;nh->get_parameter("test.q", qv);nh->get_parameter("test.expected_ee", expected);
  Eigen::VectorXd q=Eigen::Map<Eigen::VectorXd>(qv.data(),qv.size());
  if(q.size()!=6||expected.size()!=16)throw std::runtime_error("Missing test fixture");
  Eigen::Matrix4d result=model.getTq0();double max_gradient_error=0;
  for(int i=0;i<6;++i) {
   Eigen::Matrix4d t,g,plus,minus,unused;model.getAJointTran(i,q[i],t,g);result*=t;
   model.getAJointTran(i,q[i]+1e-6,plus,unused);model.getAJointTran(i,q[i]-1e-6,minus,unused);
   max_gradient_error=std::max(max_gradient_error,(g-(plus-minus)/2e-6).cwiseAbs().maxCoeff());
  }
  double fk_error=0;for(int r=0;r<4;++r)for(int c=0;c<4;++c)fk_error=std::max(fk_error,std::abs(result(r,c)-expected[4*r+c]));
  const auto clouds=model.getLinkPoint();
  for(int link=0;link<6;++link)for(int sample=0;sample<12;++sample) {
   Eigen::Vector3d point=clouds[link].col((sample*17)%clouds[link].cols()).head<3>();
   point+=Eigen::Vector3d(.003*sample,-.002*sample,.001*sample);
   const double radius=.07;
   for(bool sorted : {true,false}) {
    const auto candidates=model.armNeighbors(link,point,radius,sorted);
    for(int k=0;k<clouds[link].cols();++k)
     if((clouds[link].col(k).head<3>()-point).norm()<radius && std::find(candidates.begin(),candidates.end(),k)==candidates.end())
      throw std::runtime_error("Spatial broad phase missed a collision candidate");
   }
   // Explicitly test equality and either side of the original narrow threshold.
   const double boundary=(clouds[link].col(0).head<3>()-point).norm();
   for(double threshold : {boundary-1e-12,boundary,boundary+1e-12}) {
    if(threshold<=0.)continue;
    const auto candidates=model.armNeighbors(link,point,threshold,false);
    for(int k=0;k<clouds[link].cols();++k)
     if((clouds[link].col(k).head<3>()-point).norm()<threshold && std::find(candidates.begin(),candidates.end(),k)==candidates.end())
      throw std::runtime_error("Double radius index missed a threshold-boundary candidate");
   }
  }
  std::cout<<"collision_candidate_recall=pass " ;
  double distance=0;const bool body=model.checkCarManiCollision(q,true,distance);const bool self=model.checkManiManiCollision(q,true,distance);
  std::cout<<"fk_error="<<fk_error<<" gradient_error="<<max_gradient_error<<" body_collision="<<body<<" self_collision="<<self<<std::endl;
  if(fk_error>1e-8 || max_gradient_error>1e-6 || body || self)throw std::runtime_error("Model validation failed");
 } catch(const std::exception&e) {std::cerr<<e.what()<<std::endl;rclcpp::shutdown();return 1;}
 rclcpp::shutdown();return 0;
}
