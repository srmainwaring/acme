//
// Created by frongere on 04/08/2021.
//

#include "PropellerBaseModel.h"

#include "MathUtils/Angles.h"

namespace acme {

  PropellerBaseModel::PropellerBaseModel(const PropellerParams &params,
                                         const std::string &perf_data_json_string,
                                         PropellerModelType type) :
      m_params(params),
      m_temp_perf_data_json_string(perf_data_json_string),
      m_is_initialized(false),
      m_type(type),
      m_ku(1.),
      c_sidewash_angle_rad(0.),
      c_thrust_N(0.),
      c_torque_Nm(0.),
      c_efficiency(0.),
      c_power_W(0.) {
  }

  void PropellerBaseModel::Initialize() {
    ParsePropellerPerformanceCurveJsonString();
    m_temp_perf_data_json_string.clear();

    if (m_params.m_use_advance_velocity_correction_factor) ComputeAdvanceVelocityCorrectionFactor();

    m_is_initialized = true;
  }

  void PropellerBaseModel::DefineLogMessages(hermes::Message *msg) {

    msg->AddField<double>("uPA", "m/s", "apparent longitudinal velocity, at propeller",
                          [this](){return c_uPA;});
    msg->AddField<double>("thrust", "N", "thrust of the propeller",
                          [this](){return c_thrust_N;});
    msg->AddField<double>("torque", "Nm", "torque of the propeller",
                          [this](){return c_torque_Nm;});
    msg->AddField<double>("power", "W", "power of the propeller",
                          [this](){return c_power_W;});
    msg->AddField<double>("efficiency", "", "efficiency of the propeller",
                          [this](){return c_efficiency;});
    msg->AddField<double>("sidewash_angle", "deg", "sidewash angle at the propeller, in degrees",
                          [this](){return c_sidewash_angle_rad * RAD2DEG;});

  }

  PropellerModelType PropellerBaseModel::GetThrusterModelType() const {
    return m_type;
  }

  const PropellerParams &PropellerBaseModel::GetParameters() const {
    return m_params;
  }

  double PropellerBaseModel::GetAdvanceVelocity() const {
    return c_uPA;
  }

  double PropellerBaseModel::GetThrust() const {
    return c_thrust_N;
  }

  double PropellerBaseModel::GetTorque() const {
    return c_torque_Nm;
  }

  double PropellerBaseModel::GetPropellerEfficiency() const {
    return c_efficiency;
  }

  double PropellerBaseModel::GetPower() const {
    return c_power_W;
  }

  double PropellerBaseModel::GetPropellerAdvanceVelocity(const double &u_NWU,
                                                         const double &v_NWU) const {
    // sidewash angle
    c_sidewash_angle_rad = mathutils::Normalize__PI_PI(std::atan2(v_NWU, u_NWU));

    // estimated wake_fraction taken into account the sidewash angle (0 when the absolute value of the sidewash
    // angle exceeds 90°)
    double wp = (std::abs(c_sidewash_angle_rad) > MU_PI_2) ? 0. :
                m_params.m_hull_wake_fraction_0 * std::exp(-4. * c_sidewash_angle_rad * c_sidewash_angle_rad);

    // Propeller advance velocity
    c_uPA = m_ku * u_NWU * (1 - wp);
    return c_uPA;
  }

  void PropellerBaseModel::ComputeAdvanceVelocityCorrectionFactor() {
    /*
     * Jopt is the maximum efficiency advance ratio for the used propeller model
     */
    double Jopt = 1.; // TODO par recherche d'extremum de eta0 = J * kt / (2pi *kq)

    // Propeller design rps
    double nd = m_params.m_propeller_design_rpm / 60.;

    // Design advance ratio
    double Jd = (m_params.m_vessel_design_speed_ms * (1 - m_params.m_hull_wake_fraction_0)) /
                (nd * m_params.m_diameter_m);

    m_ku = Jopt / Jd;

  }

}  // end namespace acme