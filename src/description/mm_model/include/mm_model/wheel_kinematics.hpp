#pragma once
#include <array>
#include <algorithm>
#include <cmath>

namespace mm_model {
struct WheelSolution { std::array<double,4> steer{}, speed{}; };
inline WheelSolution inverse(double vx, double vy, double wz,
    const std::array<double,8>& positions, double radius,
    double max_rate, const std::array<double,4>& previous) {
  WheelSolution out;
  double scale=1.0;
  for (int i=0;i<4;++i) {
    const double x=vx-wz*positions[2*i+1], y=vy+wz*positions[2*i];
    double velocity=std::hypot(x,y), angle=std::atan2(y,x);
    if (velocity<1e-6) angle=previous[i];
    else if (angle>M_PI_2) {angle-=M_PI; velocity=-velocity;}
    else if (angle<-M_PI_2) {angle+=M_PI; velocity=-velocity;}
    out.steer[i]=angle;out.speed[i]=velocity/radius;
    scale=std::max(scale,std::abs(out.speed[i])/max_rate);
  }
  for(auto &v:out.speed)v/=scale;
  return out;
}
}
