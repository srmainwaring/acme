//
// Created by frongere on 09/08/2021.
//

#include <cfloat>
#include "MMGPropellerRudder.h"

namespace acme {

  template<class Rudder>
  MMGPropellerRudder<Rudder>::MMGPropellerRudder(const PropellerParams &thruster_params,
                                                 const RudderParams &rudder_params) :
      PropellerRudder<FPP1Q, Rudder>(thruster_params, rudder_params) {
    double epsilon = (1. - rudder_params.m_hull_wake_fraction_0) / (1. - thruster_params.m_hull_wake_fraction_0);
    m_kappa = 0.55 / epsilon;

    auto Dp = thruster_params.m_diameter_m;
    auto HR = rudder_params.m_height_m;

    // Here if HR (rudder span length) is lower than Dp (propeller diameter) we can consider that the whole rudder
    // is always into the slipstream and thus eta = 1
    m_eta = (Dp < HR) ? Dp / HR : 1.;

    m_gamma_R = rudder_params.m_flow_straightening;

  }

  template<class Rudder>
  void MMGPropellerRudder<Rudder>::Compute(const double &water_density,
                                           const double &u_NWU_propeller_ms,
                                           const double &v_NWU_propeller_ms,
                                           const double &u_NWU_ship_ms,
                                           const double &v_NWU_ship_ms,
                                           const double &r_rads,
                                           const double &x_pr_m,
                                           const double &x_gr_m,
                                           const double &rpm,
                                           const double &pitch_ratio,
                                           const double &rudder_angle_deg) const {

    /**
     * Solving for propeller action directly using propeller classes implementation
     * ref : Manoeuvring Technical Manual, Brix, Soder, 1992, p84
     * https://drive.google.com/file/d/195jz2YHRuhX3tSrqEPNJkYZg_7ZVTcHn/view?usp=sharing
     */
    this->m_propeller->Compute(water_density,
                               u_NWU_propeller_ms,
                               v_NWU_propeller_ms,
                               rpm,
                               pitch_ratio);

    PropellerParams propeller_params = this->m_propeller->GetParameters();
    RudderParams rudder_params = this->m_rudder->GetParameters();

    auto STW_ms = u_NWU_ship_ms * u_NWU_ship_ms + v_NWU_ship_ms * v_NWU_ship_ms;
    STW_ms = sqrt(STW_ms);

    auto leeway_rad = atan2(v_NWU_ship_ms, u_NWU_ship_ms);

    //TODO : prendre en compte lr'.r' (pas fait non plus dans esperado)
//    auto r_prime = r_rads * m_Lpp / STW_ms;
//    auto lr_prime = x_gr_m / m_Lpp;
//    auto vR_ms = -STW_ms * m_gamma_R * (leeway_rad - lr_prime * r_prime);

    auto vR_ms = -STW_ms * m_gamma_R * leeway_rad;

    auto wr = this->m_rudder->m_params.m_hull_wake_fraction_0 * std::exp(-4. * leeway_rad * leeway_rad);

    auto uR_ms = (1 - wr) * u_NWU_ship_ms;

    // Applying correction due to propeller slipstream
    auto J = this->m_propeller->J();
    auto kt = this->m_propeller->kt(J);
    if (J > DBL_EPSILON) {
      double tmp = 1. + m_kappa * (std::sqrt(1. + 8. * kt / (MU_PI * J * J)) - 1.);
      // TODO: calculer dynamiquement eta avec une formule donnant un rayon de slipstream au niveau du safran
      uR_ms *= std::sqrt(m_eta * tmp * tmp + (1. - m_eta));
    }

    // Attack angle
    auto alpha_R_rad = rudder_angle_deg * MU_PI_180 - std::atan2(vR_ms, uR_ms);
    alpha_R_rad = mathutils::Normalize__PI_PI(alpha_R_rad);

    this->m_rudder->ComputeLoads(water_density, uR_ms, vR_ms, alpha_R_rad);

    // Transport of the rudder torque to the propeller location
    this->c_rudder_torque_Nm = this->m_rudder->GetMz() - x_pr_m * this->m_rudder->GetFy();

  }

  template<class Rudder>
  void MMGPropellerRudder<Rudder>::DefineLogMessages(hermes::Message *propeller_message,
                                                     hermes::Message *rudder_message) {

    PropellerRudder<FPP1Q, Rudder>::DefineLogMessages(propeller_message, rudder_message);

//    // Propeller
//    propeller_message->AddField<double>("uPA", "m/s",
//                                        "Longitudinal velocity at the propeller position, in propeller reference frame",
//                                        [this]() { return c_uPA; });
//
//    propeller_message->AddField<double>("vPA", "m/s",
//                                        "Transverse velocity at the propeller position, in propeller reference frame",
//                                        [this]() { return c_vPA; });
//
//    // Rudder
//    //        outside slipstream
//
//    rudder_message->AddField<double>("area_RA", "m2", "Rudder area outside the slipstream",
//                                     [this]() { return c_A_RA_m2; });
//
//    rudder_message->AddField<double>("DriftAngle_RA", "rad", "Drift angle outside the slipstream",
//                                     [this]() { return c_beta_RA_rad; });
//
//    rudder_message->AddField<double>("AttackAngle_RA", "rad", "Attack angle outside the slipstream",
//                                     [this]() { return c_alpha_RA_rad; });
//
//    rudder_message->AddField<double>("u_RA", "m/s", "Longitudinal velocity",
//                                     [this]() { return c_uRA; });
//
//    rudder_message->AddField<double>("v_RA", "m/s", "transversal velocity",
//                                     [this]() { return c_vRA; });
//
//    rudder_message->AddField<double>("Drag_RA", "N", "Drag delivered by the part of the rudder outside the slipstream",
//                                     [this]() { return c_drag_RA_N; });
//
//    rudder_message->AddField<double>("Lift_RA", "N", "Lift delivered by the part of the rudder outside the slipstream",
//                                     [this]() { return c_lift_RA_N; });
//
//    rudder_message->AddField<double>("Torque_RA", "Nm",
//                                     "Torque delivered by the part of the rudder outside the slipstream",
//                                     [this]() { return c_torque_RA_Nm; });
//
//
//    //        inside slipstream
//
//    rudder_message->AddField<double>("area_RP", "m2", "Rudder area in the slipstream",
//                                     [this]() { return c_A_RP_m2; });
//
//    rudder_message->AddField<double>("DriftAngle_RP", "rad", "Drift angle inside the slipstream",
//                                     [this]() { return c_beta_RP_rad; });
//
//    rudder_message->AddField<double>("AttackAngle_RP", "rad", "Attack angle inside the slipstream",
//                                     [this]() { return c_alpha_RP_rad; });
//
//    rudder_message->AddField<double>("u_RP", "m/s", "Longitudinal velocity",
//                                     [this]() { return c_uRP; });
//
//    rudder_message->AddField<double>("v_RP", "m/s", "transversal velocity",
//                                     [this]() { return c_vRP; });
//
//    rudder_message->AddField<double>("Drag_RP", "N", "Drag delivered by the part of the rudder in the slipstream",
//                                     [this]() { return c_drag_RP_N; });
//
//    rudder_message->AddField<double>("Lift_RP", "N", "Lift delivered by the part of the rudder in the slipstream",
//                                     [this]() { return c_lift_RP_N; });
//
//    rudder_message->AddField<double>("Torque_RP", "Nm", "Torque delivered by the part of the rudder in the slipstream",
//                                     [this]() { return c_torque_RP_Nm; });

  }


}  // end namespace acme