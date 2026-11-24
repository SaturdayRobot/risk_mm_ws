#include "risk_mm_runtime/time_scaling.hpp"
#include <mm_model/wheel_kinematics.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <cstddef>
#include <array>
#include <string>
#include <stdexcept>

namespace risk_mm {
namespace {
class ScaledTrajectory final : public Trajectory {
  std::shared_ptr<const Trajectory> source_;
  double scale_;
 public:
  ScaledTrajectory(std::shared_ptr<const Trajectory> source,double scale):source_(std::move(source)),scale_(scale) {}
  double duration() const override {return source_->duration()*scale_;}
  std::vector<double> boundaries() const override {
    auto values=source_->boundaries();for(auto &value:values)value*=scale_;return values;
  }
  TrajectoryState evaluate(double time) const override {
    auto state=source_->evaluate(time/scale_);
    for(auto &value:state.velocity)value/=scale_;
    for(auto &value:state.acceleration)value/=scale_*scale_;
    for(auto &value:state.jerk)value/=scale_*scale_*scale_;
    state.forward_velocity/=scale_;return state;
  }
};
}
bool timingViolation(const std::string &reason) {
  return reason.rfind("four-wheel acceleration/steering-rate",0)==0 ||
    reason=="four-wheel steering/drive bound violation" ||
    reason=="base velocity bound violation" || reason=="arm velocity/acceleration bound violation";
}

std::shared_ptr<const Plan> scaleCandidateTime(const Plan &source,const ValidationLimits &limits,
                                             double maximum,double &scale,std::string &reason) {
  scale=1.;
  if(!source.trajectory || maximum<1. || !std::isfinite(maximum)) {reason="invalid time adaptation input";return {};}
  const double duration=source.trajectory->duration();
  if(!std::isfinite(duration) || duration<=0. || duration>120.){reason="invalid time adaptation duration";return {};}
  for(double value:{limits.base_velocity,limits.base_yaw_rate,limits.arm_velocity,limits.arm_acceleration,
                    limits.wheel_velocity,limits.wheel_acceleration,limits.wheel_radius,limits.steer_rate})
    if(!std::isfinite(value) || value<=0.){reason="invalid time adaptation limits";return {};}
  std::set<double> times;
  const int count=static_cast<int>(std::ceil(duration/.002));
  for(int k=0;k<=count;++k)times.insert(duration*k/count);
  for(double boundary:source.trajectory->boundaries())times.insert(boundary);
  auto previous_steer=source.task.start_steer;
  std::array<double,4> previous_speed{};
  double previous_time=0.,required=1.;bool first=true;
  for(double time:times) {
    const auto state=source.trajectory->evaluate(time);
    for(const auto &group:{state.position,state.velocity,state.acceleration})for(double value:group)
      if(!std::isfinite(value)){reason="nonfinite polynomial during time adaptation";return {};}
    const double dt=time-previous_time;
    required=std::max(required,std::hypot(state.velocity[0],state.velocity[1])/limits.base_velocity);
    required=std::max(required,std::abs(state.velocity[2])/limits.base_yaw_rate);
    for(int j=3;j<9;++j) {
      required=std::max(required,std::abs(state.velocity[j])/limits.arm_velocity);
      required=std::max(required,std::sqrt(std::abs(state.acceleration[j])/limits.arm_acceleration));
    }
    const auto solution=mm_model::inverse(state.forward_velocity,0.,state.velocity[2],limits.wheel_positions,
                                          limits.wheel_radius,std::numeric_limits<double>::infinity(),previous_steer);
    const double c=std::cos(state.position[2]),s=std::sin(state.position[2]);
    const double acceleration=c*state.acceleration[0]+s*state.acceleration[1];
    for(int j=0;j<4;++j) {
      const double x=state.forward_velocity-state.velocity[2]*limits.wheel_positions[2*j+1];
      const double y=state.velocity[2]*limits.wheel_positions[2*j];
      const double ax=acceleration-state.acceleration[2]*limits.wheel_positions[2*j+1];
      const double ay=state.acceleration[2]*limits.wheel_positions[2*j];
      const double squared=x*x+y*y;
      if(squared>1e-12) {
        required=std::max(required,std::abs(x*ay-y*ax)/squared/limits.steer_rate);
        required=std::max(required,std::sqrt(std::abs(x*ax+y*ay)/std::sqrt(squared)/limits.wheel_radius/limits.wheel_acceleration));
      }
      required=std::max(required,std::abs(solution.speed[j])/limits.wheel_velocity);
      if(!first && dt>1e-10) {
        required=std::max(required,std::abs(solution.steer[j]-previous_steer[j])/dt/limits.steer_rate);
        required=std::max(required,std::sqrt(std::abs(solution.speed[j]-previous_speed[j])/dt/limits.wheel_acceleration));
      }
    }
    if(!std::isfinite(required)){reason="nonfinite time adaptation";return {};}
    previous_steer=solution.steer;previous_speed=solution.speed;previous_time=time;first=false;
  }
  scale=std::ceil(required*1.05*100.)/100.;
  if(scale>maximum){reason="required time scale exceeds configured maximum: "+std::to_string(scale);return {};}
  auto result=std::make_shared<Plan>(source);
  result->trajectory=std::make_shared<ScaledTrajectory>(source.trajectory,scale);
  for(auto &point:result->points)point.time_s*=scale;
  reason="explicit four-wheel time scale="+std::to_string(scale);return result;
}
namespace {
class LocalTimeTrajectory final : public Trajectory {
  std::shared_ptr<const Trajectory> source_;
 public:
  std::vector<double> u,scale,time;
  LocalTimeTrajectory(std::shared_ptr<const Trajectory> source,std::vector<double> knots,std::vector<double> values)
      :source_(std::move(source)),u(std::move(knots)),scale(std::move(values)),time(u.size(),0.) {
    for(size_t i=1;i<u.size();++i)time[i]=time[i-1]+(u[i]-u[i-1])*(scale[i]+scale[i-1])*.5;
  }
  double duration() const override {return time.back();}
  size_t interval(double value,const std::vector<double> &knots) const {
    return std::min(knots.size()-2,static_cast<size_t>(std::max<ptrdiff_t>(0,std::upper_bound(knots.begin(),knots.end(),value)-knots.begin()-1)));
  }
  double physicalTime(double value) const {
    const size_t i=interval(value,u);const double h=u[i+1]-u[i],r=std::clamp((value-u[i])/h,0.,1.);
    return time[i]+h*(scale[i]*r+(scale[i+1]-scale[i])*(r*r*r-.5*r*r*r*r));
  }
  TrajectoryState mapState(double value,TrajectoryState state) const {
    const size_t i=interval(value,u);const double h=u[i+1]-u[i],r=std::clamp((value-u[i])/h,0.,1.);
    const double s=scale[i]+(scale[i+1]-scale[i])*(3*r*r-2*r*r*r);
    const double derivative=(scale[i+1]-scale[i])*(6*r-6*r*r)/h;
    const double second=(scale[i+1]-scale[i])*(6-12*r)/(h*h);
    for(size_t j=0;j<9;++j) {
      state.jerk[j]=state.jerk[j]/(s*s*s)-3*state.acceleration[j]*derivative/(s*s*s*s)-
          state.velocity[j]*second/(s*s*s*s)+3*state.velocity[j]*derivative*derivative/(s*s*s*s*s);
      state.acceleration[j]=state.acceleration[j]/(s*s)-state.velocity[j]*derivative/(s*s*s);
      state.velocity[j]/=s;
    }
    state.forward_velocity/=s;return state;
  }
  TrajectoryState atSource(double value) const {return mapState(value,source_->evaluate(value));}
  TrajectoryState evaluate(double value) const override {
    if(!std::isfinite(value)||value<0.||value>duration()+1e-8)throw std::out_of_range("local time trajectory");
    value=std::clamp(value,0.,duration());const size_t i=interval(value,time);
    double lo=u[i],hi=u[i+1],guess=lo+(hi-lo)*(value-time[i])/(time[i+1]-time[i]);
    for(int iteration=0;iteration<30;++iteration) {
      const double residual=physicalTime(guess)-value;
      if(std::abs(residual)<1e-12)break;
      if(residual>0.)hi=guess;else lo=guess;
      const double r=(guess-u[i])/(u[i+1]-u[i]);
      const double s=scale[i]+(scale[i+1]-scale[i])*(3*r*r-2*r*r*r);
      const double next=guess-residual/s;
      guess=next>lo && next<hi?next:(lo+hi)*.5;
    }
    return atSource(std::clamp(guess,u.front(),u.back()));
  }
  std::vector<double> boundaries() const override {
    auto result=time;for(double value:source_->boundaries())result.push_back(physicalTime(value));
    std::sort(result.begin(),result.end());result.erase(std::unique(result.begin(),result.end()),result.end());return result;
  }
};
struct MotionRatio {
  double value{0.};
  std::string constraint;
};
MotionRatio motionRatio(const TrajectoryState &state,const ValidationLimits &limits) {
  MotionRatio result;
  auto include=[&](double value,const std::string &constraint) {
    if(!std::isfinite(value)){result.value=std::numeric_limits<double>::infinity();result.constraint=constraint;return;}
    if(value>result.value){result.value=value;result.constraint=constraint;}
  };
  include(std::hypot(state.velocity[0],state.velocity[1])/limits.base_velocity,"base_velocity");
  include(std::abs(state.velocity[2])/limits.base_yaw_rate,"base_yaw_rate");
  for(int j=3;j<9;++j) {
    include(std::abs(state.velocity[j])/limits.arm_velocity,"arm_velocity_joint_"+std::to_string(j-2));
    include(std::sqrt(std::abs(state.acceleration[j])/limits.arm_acceleration),
            "arm_acceleration_joint_"+std::to_string(j-2));
  }
  const double acceleration=std::cos(state.position[2])*state.acceleration[0]+std::sin(state.position[2])*state.acceleration[1];
  static const std::array<const char*,4> wheel_names{"fl","fr","rl","rr"};
  for(int j=0;j<4;++j) {
    const double x=state.forward_velocity-state.velocity[2]*limits.wheel_positions[2*j+1],y=state.velocity[2]*limits.wheel_positions[2*j];
    const double ax=acceleration-state.acceleration[2]*limits.wheel_positions[2*j+1],ay=state.acceleration[2]*limits.wheel_positions[2*j];
    const double squared=x*x+y*y;
    const std::string wheel=wheel_names[j];
    include(std::sqrt(squared)/limits.wheel_radius/limits.wheel_velocity,"wheel_velocity_"+wheel);
    if(squared>1e-12) {
      include(std::abs(x*ay-y*ax)/squared/limits.steer_rate,"steer_rate_"+wheel);
      include(std::sqrt(std::abs(x*ax+y*ay)/std::sqrt(squared)/limits.wheel_radius/limits.wheel_acceleration),
              "wheel_acceleration_"+wheel);
    }
  }
  return result;
}
}
std::shared_ptr<const Plan> adaptCandidateTime(const Plan &source,const ValidationLimits &limits,
    double maximum,const std::string &profile,TimeAdaptation &report,std::string &reason) {
  report={};report.profile=profile;
  if(profile=="uniform") {
    auto result=scaleCandidateTime(source,limits,maximum,report.duration_ratio,reason);
    report.maximum_scale=report.endpoint_scale=report.duration_ratio;return result;
  }
  if(profile=="through") {
    // Fixed endpoint time derivatives preserve the moving p/v/a handoff.
    // A single interior scale avoids a second iterative trajectory optimizer.
    if(!source.trajectory || !std::isfinite(maximum) || maximum<1.){reason="invalid through timing input";return {};}
    for(double value:{limits.base_velocity,limits.base_yaw_rate,limits.arm_velocity,limits.arm_acceleration,
                      limits.wheel_velocity,limits.wheel_acceleration,limits.wheel_radius,limits.steer_rate})
      if(!std::isfinite(value)||value<=0.){reason="invalid through timing limits";return {};}
    const double duration=source.trajectory->duration();
    if(!std::isfinite(duration) || duration<=0. || duration>120.){reason="invalid through timing duration";return {};}
    const int count=std::max(1,static_cast<int>(std::ceil(duration/.002)));
    std::vector<TrajectoryState> samples; samples.reserve(count+1);
    for(int k=0;k<=count;++k)samples.push_back(source.trajectory->evaluate(duration*k/count));
    const auto initial=source.trajectory->evaluate(0.),terminal=source.trajectory->evaluate(duration);
    const double boundary_speed=std::max(std::hypot(initial.velocity[0],initial.velocity[1]),
        std::hypot(terminal.velocity[0],terminal.velocity[1]));
    // Tie the C2 boundary transition to physical acceleration, rather than a
    // large fraction of the whole mission segment. Endpoint p/v/a stays exact;
    // the same dense four-wheel checks below decide whether this map is valid.
    const double transition=std::min(duration*.25,std::max(.2,
        2.*boundary_speed/(limits.wheel_acceleration*limits.wheel_radius)));
    const std::vector<double> knots{0.,transition,duration-transition,duration};
    double best_ratio=std::numeric_limits<double>::infinity(),best_scale=1.,best_time=0.;
    double first_time=-1.,first_ratio=0.;std::string best_constraint,first_constraint;
    for(double scale=1.;scale<=maximum+1e-9;scale+=.05) {
      auto trajectory=std::make_shared<LocalTimeTrajectory>(source.trajectory,knots,std::vector<double>{1.,scale,scale,1.});
      double profile_ratio=0.,profile_time=0.;std::string profile_constraint;
      for(int k=0;k<=count;++k) {
        const double source_time=duration*k/count;
        const auto metric=motionRatio(trajectory->mapState(source_time,samples[k]),limits);
        const double ratio=metric.value;
        if(!std::isfinite(ratio)){reason="nonfinite through timing constraint at t="+std::to_string(source_time);return {};}
        if(ratio>profile_ratio){profile_ratio=ratio;profile_time=source_time;profile_constraint=metric.constraint;}
        if(scale==1. && first_time<0. && ratio>.97) {
          first_time=source_time;first_ratio=ratio;first_constraint=metric.constraint;
        }
      }
      if(profile_ratio<best_ratio) {
        best_ratio=profile_ratio;best_scale=scale;best_time=profile_time;best_constraint=profile_constraint;
      }
      if(profile_ratio>.97)continue;
      auto result=std::make_shared<Plan>(source);result->trajectory=trajectory;
      for(auto &point:result->points)point.time_s=trajectory->physicalTime(point.time_s);
      report.duration_ratio=trajectory->duration()/duration;report.maximum_scale=scale;report.endpoint_scale=1.;
      report.first_violation_time_s=first_time;report.first_violation_ratio=first_ratio;
      report.first_violation_constraint=first_constraint;
      report.best_scale=scale;report.best_time_s=profile_time;report.best_ratio=profile_ratio;report.best_constraint=profile_constraint;
      report.source_knots=knots;report.time_knots=trajectory->time;report.scale_knots=trajectory->scale;
      reason="explicit through time profile; moving endpoint derivatives preserved";return result;
    }
    report.first_violation_time_s=first_time;report.first_violation_ratio=first_ratio;
    report.first_violation_constraint=first_constraint;report.best_scale=best_scale;
    report.best_time_s=best_time;report.best_ratio=best_ratio;report.best_constraint=best_constraint;
    reason="no feasible moving-boundary time profile within configured scale; first="+
      first_constraint+" at t="+std::to_string(first_time)+" ratio="+std::to_string(first_ratio)+
      "; best scale="+std::to_string(best_scale)+" max="+best_constraint+" at t="+
      std::to_string(best_time)+" ratio="+std::to_string(best_ratio);
    return {};
  }
  if(profile!="local" || !source.trajectory || !std::isfinite(maximum) || maximum<1.) {reason="invalid local time adaptation input";return {};}
  for(double value:{limits.base_velocity,limits.base_yaw_rate,limits.arm_velocity,limits.arm_acceleration,
                    limits.wheel_velocity,limits.wheel_acceleration,limits.wheel_radius,limits.steer_rate})
    if(!std::isfinite(value)||value<=0.){reason="invalid local time adaptation limits";return {};}
  const double duration=source.trajectory->duration();
  if(!std::isfinite(duration)||duration<=0.||duration>120.){reason="invalid local time adaptation duration";return {};}
  const int count=std::max(1,static_cast<int>(std::ceil(duration/.04)));
  const double step=duration/count;std::vector<double> knots(count+1),scales;
  for(int i=0;i<=count;++i)knots[i]=duration*i/count;
  const int dense=std::max(1,static_cast<int>(std::ceil(duration/.002)));
  double global=1.;
  for(int k=0;k<=dense;++k) {
    const double u=duration*k/dense;const auto state=source.trajectory->evaluate(u);
    const double ratio=motionRatio(state,limits).value*1.06;
    if(!std::isfinite(ratio)){reason="nonfinite local time adaptation";return {};}
    global=std::max(global,ratio);
  }
  global=std::ceil(global*100.)/100.;
  if(global>maximum){reason="required local time scale exceeds configured maximum";return {};}
  // Start from a feasible uniform map, then shorten each local interval while
  // rechecking its exact velocity/acceleration chain rule. Cached polynomial
  // samples avoid repeating geometry and polynomial evaluation in this step.
  struct TimingSample {TrajectoryState state;double r;};
  std::vector<std::vector<TimingSample>> samples(count);
  for(int i=0;i<count;++i) {
    const int n=std::max(2,static_cast<int>(std::ceil(step/.002)));
    for(int k=0;k<=n;++k)samples[i].push_back({source.trajectory->evaluate(knots[i]+step*k/n),double(k)/n});
  }
  scales.assign(count+1,global);
  auto interval_valid=[&](int i) {
    for(const auto &sample:samples[i]) {
      const double r=sample.r,s=scales[i]+(scales[i+1]-scales[i])*(3*r*r-2*r*r*r);
      const double derivative=(scales[i+1]-scales[i])*(6*r-6*r*r)/step;
      auto state=sample.state;
      for(int j=0;j<9;++j) {
        state.acceleration[j]=state.acceleration[j]/(s*s)-state.velocity[j]*derivative/(s*s*s);
        state.velocity[j]/=s;
      }
      state.forward_velocity/=s;
      const double ratio=motionRatio(state,limits).value;
      if(!std::isfinite(ratio)||ratio>.97)return false;
    }
    return true;
  };
  for(int pass=0;pass<100;++pass) {
    bool changed=false;
    for(int k=0;k<=count;++k) {
      const int i=pass%2?count-k:k;const double saved=scales[i];
      const double minimum=(i==0 || i==count)?1.:.5;
      const double trial=std::max(minimum,saved*.95);
      if(trial>=saved-1e-10)continue;
      scales[i]=trial;
      if((i>0 && !interval_valid(i-1)) || (i<count && !interval_valid(i)))scales[i]=saved;
      else changed=true;
    }
    if(!changed)break;
  }
  auto trajectory=std::make_shared<LocalTimeTrajectory>(source.trajectory,knots,scales);
  auto previous_steer=source.task.start_steer;std::array<double,4> previous_speed{};double previous_time=0.;
  for(int k=0;k<=dense;++k) {
    const double u=duration*k/dense,t=trajectory->physicalTime(u);const auto state=trajectory->atSource(u);
    double ratio=motionRatio(state,limits).value;
    const auto wheels=mm_model::inverse(state.forward_velocity,0.,state.velocity[2],limits.wheel_positions,
        limits.wheel_radius,std::numeric_limits<double>::infinity(),previous_steer);
    if(k>0)for(int j=0;j<4;++j) {
      ratio=std::max(ratio,std::abs(wheels.steer[j]-previous_steer[j])/(t-previous_time)/limits.steer_rate);
      ratio=std::max(ratio,std::sqrt(std::abs(wheels.speed[j]-previous_speed[j])/(t-previous_time)/limits.wheel_acceleration));
    }
    if(!std::isfinite(ratio) || ratio>1.+1e-5) {reason="local time profile failed dense four-wheel/arm check";return {};}
    previous_steer=wheels.steer;previous_speed=wheels.speed;previous_time=t;
  }
  if(trajectory->duration()>120.){reason="local timing duration exceeds public cap";return {};}
  auto result=std::make_shared<Plan>(source);result->trajectory=trajectory;
  for(auto &point:result->points)point.time_s=trajectory->physicalTime(point.time_s);
  report.duration_ratio=trajectory->duration()/duration;report.maximum_scale=*std::max_element(scales.begin(),scales.end());
  report.endpoint_scale=scales.back();report.source_knots=knots;report.time_knots=trajectory->time;report.scale_knots=scales;
  reason="explicit local time profile; duration ratio="+std::to_string(report.duration_ratio)+" max local scale="+std::to_string(report.maximum_scale);
  return result;
}
}
