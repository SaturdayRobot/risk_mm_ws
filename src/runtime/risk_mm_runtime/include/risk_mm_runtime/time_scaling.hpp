#pragma once
#include "risk_mm_runtime/validation.hpp"

namespace risk_mm {
// Model adaptation after original planning. Geometry and phase order are fixed;
// p(t/s), v(t/s)/s and a(t/s)/s² preserve the exact polynomial representation.
bool timingViolation(const std::string &reason);
std::shared_ptr<const Plan> scaleCandidateTime(const Plan &,const ValidationLimits &,
                                             double max_scale,double &scale,std::string &reason);
struct TimeAdaptation {
  std::string profile;
  double duration_ratio{1.},maximum_scale{1.};
  double endpoint_scale{1.};
  double first_violation_time_s{-1.},first_violation_ratio{0.};
  std::string first_violation_constraint;
  double best_scale{1.},best_time_s{0.},best_ratio{0.};
  std::string best_constraint;
  std::vector<double> source_knots,time_knots,scale_knots;
};
// Explicit local timing policy. The source polynomial is evaluated exactly;
// only its monotone C2 time map changes. No geometry or constraint is relaxed.
std::shared_ptr<const Plan> adaptCandidateTime(const Plan &,const ValidationLimits &,
    double max_scale,const std::string &profile,TimeAdaptation &,std::string &reason);
}
