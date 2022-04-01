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

  void
  FujiiRudderModel::GetClCdCn(const double &attack_angle_rad, const double &rudder_angle_rad, double &cl, double &cd,
                              double &cn) const {

    double salpha = std::sin(attack_angle_rad);
    double calpha = std::cos(attack_angle_rad);
    cd = -m_f_alpha * salpha * salpha;
    cl = m_f_alpha * salpha * calpha;
    cn = 0.;

  }

} // end namespace acme