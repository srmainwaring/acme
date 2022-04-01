//
// Created by lletourn on 05/11/2021.
//

#include "BrixRudderModel.h"

namespace acme {

  BrixRudderModel::BrixRudderModel(const RudderParams &params,
                    const std::string &rudder_perf_data_json_string) :
      RudderBaseModel(params) {
  }

  void BrixRudderModel::Initialize() {
    m_is_initialized = true;
  }

  void
  BrixRudderModel::GetClCdCn(const double &attack_angle_rad,
                             const double &rudder_angle_rad,
                             double &cl,
                             double &cd,
                             double &cn) const {

    auto aspect_ratio = m_params.m_lateral_area_m2 / (m_params.m_chord_m * m_params.m_chord_m);
    
    auto ca = cos(attack_angle_rad);
    auto sa = sin(attack_angle_rad);
    
    // Lift coefficient
    auto Cl1 = 2 * MU_PI * aspect_ratio * (aspect_ratio + 1) / pow(aspect_ratio+2, 2) * sa;
    auto Cl2 = m_params.m_Cq * sa * abs(sa) * ca;
    cl = Cl1 + Cl2;

    // Drag coefficient
    auto Cd1 = 1.1 * cl * cl / (MU_PI * aspect_ratio);
    auto Cd2 = m_params.m_Cq * pow(abs(sa), 3) + 2.5 * m_params.m_Cf;
    cd = - (Cd1 + Cd2);

    // Torque coefficient at rudder nose
    auto Cqn = -(Cl1*ca + Cd1*sa) * (0.47 - (aspect_ratio+2)/(4*(aspect_ratio+1)))
        - 0.75 *(Cl2*ca + Cd2*sa);

    // Torque coefficient at rudder stock
    cn = Cqn + m_params.m_distance_nose_stock_m / m_params.m_chord_m * (cl * ca + cd * sa);

  }
} // end namespace acme