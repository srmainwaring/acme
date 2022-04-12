//
// Created by lletourn on 04/11/2021.
//

#include "RudderBaseModel.h"

#include <iostream>
#include <vector>
#include <MathUtils/Angles.h>
#include "MathUtils/Unit.h"
#include <fstream>

namespace acme {

  double compute_ITTC57_frictional_resistance_coefficient(double rudder_chord_m,
                                                          double mean_velocity_ms,
                                                          double nu_water) {
    assert(mean_velocity_ms > 1E-1);
    // Reynolds number
    auto Re = mean_velocity_ms * rudder_chord_m / nu_water;
    // Frictional coefficient from ITTC57
    return 0.075 / pow(log10(Re - 2), 2);
  }

  RudderBaseModel::RudderBaseModel(const RudderParams &params) :
      m_params(params),
      m_type(RudderModelType::E_SIMPLE_RUDDER),
      m_is_initialized(false),
      m_is_logged(false){

  }

  void RudderBaseModel::Compute(const double &water_density,
                                  const double &u_NWU,
                                  const double &v_NWU,
                                  const double &rudder_angle_deg,
                                  const double &u_ship_NWU,
                                  const double &v_ship_NWU,
                                  const double &r_ship_NWU,
                                  const double &x_r) const {

    if (!m_is_initialized) {
      std::cerr << "Rudder model MUST be initialized before being used." << std::endl;
      exit(EXIT_FAILURE);
    }

    c_u_NWU = u_NWU;
    c_v_NWU = v_NWU;

    c_uRA = u_NWU;
    c_vRA = v_NWU;

    if (m_params.m_has_hull_influence) {

      double sidewash_angle_0 = std::atan2(v_NWU, u_NWU); // ou drift_angle_0 pour calculer le wake fraction

      // Estimated wake fraction
      double wr = m_params.m_hull_wake_fraction_0 * std::exp(-4. * sidewash_angle_0 * sidewash_angle_0);

      c_uRA *= (1. - wr);

      if (m_params.m_has_hull_influence_transverse_velocity) {
        double beta_R = atan2(v_ship_NWU + 2*x_r*r_ship_NWU, u_ship_NWU);
        double kappa = HullStraighteningFunction(beta_R);
        c_vRA *= kappa;
      };

    }

    // Drift angle
    c_beta_R_rad = std::atan2(c_vRA, c_uRA);

    // Attack angle
    c_rudder_angle_rad = rudder_angle_deg * MU_PI_180;
    c_alpha_R_rad = c_rudder_angle_rad - c_beta_R_rad;
    c_alpha_R_rad = mathutils::Normalize__PI_PI(c_alpha_R_rad);

    // Get coefficients
    ComputeLoads(water_density, c_uRA, c_vRA, c_alpha_R_rad);

    // Hull/rudder interactions
    if (m_params.m_has_hull_influence) {
      c_torque_Nm += m_params.m_aH * (m_params.m_xH - m_params.m_xR) * c_fy_N;
      c_fx_N *= (1. - m_params.m_tR);
      c_fy_N *= (1. + m_params.m_aH);
    }

  }

  void RudderBaseModel::ComputeLoads(const double &water_density,
                                     const double &uR_ms,
                                     const double &vR_ms,
                                     const double &alpha_R_rad) const {

    // Get coefficients
    double cl, cd, cn;
    GetClCdCn(alpha_R_rad, 0., cl, cd, cn);

    // Forces in flow frame
    double q = 0.5 * water_density * (uR_ms * uR_ms + vR_ms * vR_ms); // stagnation pressure at rudder position
    c_drag_N = q * cd * m_params.m_lateral_area_m2;
    c_lift_N = q * cl * m_params.m_lateral_area_m2;
    c_torque_Nm = q * cn * m_params.m_lateral_area_m2 * m_params.m_chord_m;

    // Forces in body frame
    auto beta_R_rad = std::atan2(vR_ms, uR_ms);
    double Cbeta = std::cos(beta_R_rad);
    double Sbeta = std::sin(beta_R_rad);

    c_fx_N = Cbeta * c_drag_N - Sbeta * c_lift_N;
    c_fy_N = Sbeta * c_drag_N + Cbeta * c_lift_N;

  }

  RudderModelType RudderBaseModel::GetRudderModelType() const {
    return m_type;
  }

  const RudderParams &RudderBaseModel::GetParameters() const {
    return m_params;
  }

  void RudderBaseModel::Log(bool is_logged) {
    m_is_logged = is_logged;
  }

  void RudderBaseModel::DefineLogMessages(hermes::Message* msg) {

    msg->AddField<double>("u_NWU", "m/s", "vessel longitudinal velocity",
                          [this](){return c_u_NWU;});
    msg->AddField<double>("v_NWU", "m/s", "vessel lateral velocity",
                          [this](){return c_v_NWU;});
    msg->AddField<double>("uRA", "m/s", "apparent longitudinal velocity of the rudder",
                          [this](){return c_uRA;});
    msg->AddField<double>("vRA", "m/s", "apparent lateral velocity of the rudder",
                          [this](){return c_vRA;});
    msg->AddField<double>("rudder_angle", "deg", "rudder deflection angle, in degrees",
                          [this](){return c_rudder_angle_rad * RAD2DEG;});
    msg->AddField<double>("attack_angle", "deg", "attack angle at the rudder, in degrees",
                          [this](){return c_alpha_R_rad * RAD2DEG;});
    msg->AddField<double>("drift_angle", "deg", "rudder drift angle, in degrees",
                          [this](){return c_alpha_R_rad * RAD2DEG;});
    msg->AddField<double>("drag", "N", "drag force induced by the rudder",
                          [this](){return c_drag_N;});
    msg->AddField<double>("lift", "N", "lift force induced by the rudder",
                          [this](){return c_lift_N;});
    msg->AddField<double>("fx", "N", "longitudinal force induced by the rudder",
                          [this](){return c_fx_N;});
    msg->AddField<double>("fy", "N", "lateral force induced by the rudder",
                          [this](){return c_fy_N;});
    msg->AddField<double>("torque", "Nm", "torque induced by the rudder, at the rudder position",
                          [this](){return c_torque_Nm;});

  }

  void RudderBaseModel::Finalize(double time) {
    if (m_is_logged) {
//      auto log_file = "rudder_log.csv";
//      std::ofstream myfile;
//      myfile.open(log_file, std::ios::app);
//      if (!myfile.tellp()) {
//        myfile << "time;u_NWU;v_NWU;uRA;vRA;rudder_angle;alpha_R_rad;beta_R_rad;drag_N;lift_N;torque_Nm;fx_N;fy_N;" << std::endl;
//        myfile << "s;m/s;m/s;m/s;m/s;rad;rad;rad;N;N;N.m;N;N;" << std::endl;
//      }
//      myfile << time << ';' << c_u_NWU << ';' << c_v_NWU << ';' << c_uRA << ';' << c_vRA << ';'
//             << c_rudder_angle_rad << ';' << c_alpha_R_rad << ';' << c_beta_R_rad << ';'
//             << c_drag_N << ';' << c_lift_N << ';' << c_torque_Nm << ';' << c_fx_N << ';'
//             << c_fy_N << ';' << std::endl;
    }
  }

  double RudderBaseModel::HullStraighteningFunction(const double & beta) {
    double K2 = 0.5, K3 = 0.45, beta_1 = 1.3, beta_2 = MU_PI_2;
    double bv = (1-K2)/(beta_2 - beta_1);
    double av = K2 - bv * beta_1;

    double kappa;
    if (abs(beta) < beta_1)
      kappa = std::min(K2, K3 * abs(beta));
    else if (abs(beta)>beta_2)
      kappa = 1.;
    else
      kappa = av + bv * abs(beta);
    return kappa;
  }
}  // end namespace acme
