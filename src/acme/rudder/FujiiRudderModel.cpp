//
// Created by lletourn on 04/11/2021.
//

#include "FujiiRudderModel.h"

namespace acme {

  FujiiRudderModel::FujiiRudderModel(const acme::RudderParams params) : RudderBaseModel(params), m_f_alpha(0.) {

  }

  void FujiiRudderModel::Initialize() {

    auto aspect_ratio = m_params.m_lateral_area_m2 / (m_params.m_chord_m * m_params.m_chord_m);
    m_f_alpha = 6.13 * aspect_ratio / (aspect_ratio + 2.25);
    m_is_initialized = true;

  }

  void FujiiRudderModel::ComputeLoads(const double &water_density) const {

    // Normal force in rudder frame
    double q = 0.5 * water_density * (c_uRA * c_uRA + c_vRA * c_vRA); // stagnation pressure at rudder position
    auto c_normal_N = q * m_params.m_lateral_area_m2 * m_f_alpha * sin(c_alpha_R_rad);

    // Projection in vessel frame
    c_fx_N = -c_normal_N * sin(c_rudder_angle_rad);
    c_fy_N = c_normal_N * cos(c_rudder_angle_rad);

  }

} // end namespace acme