#pragma once
#include <Eigen/Core>
#include <array>
#include <cmath>

namespace risk_mm {
struct CurvatureResidual {
  double value;
  Eigen::Vector2d velocity_gradient,acceleration_gradient;
};
// Model-specific extension: keep turn radius beyond the largest lateral wheel
// offset, avoiding +/-90 degree steering branch crossings while rolling.
inline CurvatureResidual fourWheelCurvature(const Eigen::Vector2d &v,const Eigen::Vector2d &a,double radius) {
  const double squared=v.squaredNorm()+1e-12;
  const double cross=v.x()*a.y()-v.y()*a.x();
  const double inverse=1./(squared*squared*squared);
  return {cross*cross*inverse-1./(radius*radius),
    2.*cross*inverse*Eigen::Vector2d(a.y(),-a.x())-6.*cross*cross*inverse/squared*v,
    2.*cross*inverse*Eigen::Vector2d(-v.y(),v.x())};
}

struct FourWheelDifferentials {
  // Three rows per wheel: drive speed², drive acceleration², steering rate².
  Eigen::Matrix<double,12,1> squared;
  Eigen::Matrix<double,12,6> jacobian; // vx, vy, ax, ay, jx, jy in world
};
inline FourWheelDifferentials fourWheelDifferentials(const Eigen::Vector2d &v,
    const Eigen::Vector2d &a,const Eigen::Vector2d &j,int direction,
    const std::array<double,8> &positions,double radius) {
  using Gradient=Eigen::Matrix<double,6,1>;
  const double vv=v.squaredNorm()+1e-12,speed=std::sqrt(vv),dot=v.dot(a);
  const double cross=v.x()*a.y()-v.y()*a.x(),cross_jerk=v.x()*j.y()-v.y()*j.x();
  Gradient gvv,gdot,gcross,gcross_jerk;
  gvv<<2*v.x(),2*v.y(),0,0,0,0;
  gdot<<a.x(),a.y(),v.x(),v.y(),0,0;
  gcross<<a.y(),-a.x(),-v.y(),v.x(),0,0;
  gcross_jerk<<j.y(),-j.x(),0,0,-v.y(),v.x();
  const Gradient gspeed=gvv/(2.*speed);
  const double omega=cross/vv,alpha=cross_jerk/vv-2.*cross*dot/(vv*vv);
  const Gradient gomega=gcross/vv-cross*gvv/(vv*vv);
  const Gradient galpha=gcross_jerk/vv-cross_jerk*gvv/(vv*vv)
      -2.*(dot*gcross+cross*gdot)/(vv*vv)+4.*cross*dot*gvv/(vv*vv*vv);
  const double longitudinal=direction*speed,acceleration=direction*dot/speed;
  const Gradient glongitudinal=direction*gspeed;
  const Gradient gacceleration=direction*(gdot/speed-dot*gspeed/vv);
  FourWheelDifferentials result;
  for(int wheel=0;wheel<4;++wheel) {
    const double rx=positions[2*wheel],ry=positions[2*wheel+1];
    const double x=longitudinal-omega*ry,y=omega*rx;
    const double ax=acceleration-alpha*ry,ay=alpha*rx;
    const Gradient gx=glongitudinal-ry*gomega,gy=rx*gomega;
    const Gradient gax=gacceleration-ry*galpha,gay=rx*galpha;
    const double squared=x*x+y*y+1e-12,numerator=x*ax+y*ay,turn=x*ay-y*ax;
    const Gradient gsquared=2.*(x*gx+y*gy);
    const Gradient gnumerator=ax*gx+x*gax+ay*gy+y*gay;
    const Gradient gturn=ay*gx+x*gay-ax*gy-y*gax;
    const int row=3*wheel;
    result.squared[row]=squared/(radius*radius);
    result.jacobian.row(row)=(gsquared/(radius*radius)).transpose();
    result.squared[row+1]=numerator*numerator/(squared*radius*radius);
    result.jacobian.row(row+1)=(2.*numerator*gnumerator/(squared*radius*radius)
        -numerator*numerator*gsquared/(squared*squared*radius*radius)).transpose();
    result.squared[row+2]=turn*turn/(squared*squared);
    result.jacobian.row(row+2)=(2.*turn*gturn/(squared*squared)
        -2.*turn*turn*gsquared/(squared*squared*squared)).transpose();
  }
  return result;
}
}
