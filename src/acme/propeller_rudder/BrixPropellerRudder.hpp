//
// Created by frongere on 09/08/2021.
//

#include "BrixPropellerRudder.h"

namespace acme {

  template<class Propeller, class Rudder>
  BrixPropellerRudder<Propeller, Rudder>::BrixPropellerRudder(const PropellerParams &thruster_params,
                                                          const RudderParams &rudder_params) :
      PropellerRudder<Propeller, Rudder>(thruster_params, rudder_params){

      }

  template<class Propeller, class Rudder>
  double BrixPropellerRudder<Propeller, Rudder>::GetRudderFx() const {
    return c_fx_R_N;
  }

  template<class Propeller, class Rudder>
  double BrixPropellerRudder<Propeller, Rudder>::GetRudderFy() const {
    return c_fy_R_N;
  }

  template<class Propeller, class Rudder>
  double BrixPropellerRudder<Propeller, Rudder>::GetRudderMz() const {
    return c_torque_RA_Nm + c_torque_RP_Nm;
  }

  template<class Propeller, class Rudder>
  void BrixPropellerRudder<Propeller, Rudder>::Compute(const double &water_density,
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

    /*
     * Computing velocities seen by the rudder outside the slipstream but taking into account the wake fraction
     */

    double wr0 = rudder_params.m_hull_wake_fraction_0; // rudder wake fraction in straight line

    double u_R0 = u_NWU_propeller_ms;
    double v_R0 = v_NWU_propeller_ms + r_rads * x_pr_m;  // Transport of the propeller velocity to the rudder position

    c_uRA = u_R0;
    c_vRA = v_R0;

    if (rudder_params.m_has_hull_influence and u_R0 > std::numeric_limits<double>::epsilon()) {
      double rudder_sidewash_angle_0 = std::atan2(v_R0, u_R0);

      // Estimated wake fraction for the rudder
      double wR = wr0 * std::exp(-4. * rudder_sidewash_angle_0 * rudder_sidewash_angle_0);

      c_uRA *= (1 - wR);

      if (rudder_params.m_has_hull_influence_transverse_velocity) {
        double beta_R = atan2(v_NWU_ship_ms + 2 * x_gr_m * r_rads, u_NWU_ship_ms);
        double kappa = RudderBaseModel::HullStraighteningFunction(beta_R);
        c_vRA *= kappa;
      }
    }

    this->c_rudder_angle_rad = rudder_angle_deg * MU_PI_180;

    // Mean axial speed of inflow to the propeller (with wake fraction correction included)
    c_uPA = this->m_propeller->GetAdvanceVelocity();
    c_vPA = v_NWU_propeller_ms;

    // Propeller data
    double r0 = 0.5 * propeller_params.m_diameter_m;  // Propeller radius
    double t = propeller_params.m_thrust_deduction_factor_0;
    double Ap = MU_PI * r0 * r0;  // Propeller disk area

    // Rudder data
    double A_R = rudder_params.m_lateral_area_m2;// Rudder total area
    double c = rudder_params.m_chord_m;// Rudder chord length at its half height
    double h_R = rudder_params.m_height_m;// Rudder height
    double a_H = rudder_params.m_aH;
    double x_R = rudder_params.m_xR;
    double x_H = rudder_params.m_xH;

    /**
     * Dealing with rudder forces INSIDE the slipstream of the propeller (RP)
     */

    // Stagnation pressure at propeller position
    double q_PA = 0.5 * water_density * (c_uPA * c_uPA + c_vPA * c_vPA);

    if (q_PA == 0.) {
      c_uRP = 0.;
      c_vRP = c_vRA;
      c_A_RP_m2 = 0.;
      c_beta_RP_rad = 0.;
      c_alpha_RP_rad = this->c_rudder_angle_rad;
      c_drag_RP_N = 0.;
      c_lift_RP_N = 0.;
      c_torque_RP_Nm = 0.;
      c_fx_RP_N = 0.;
      c_fy_RP_N = 0.;
    } else {
      // Thrust loading coefficient
      double Cth = std::abs(this->m_propeller->GetThrust() / (q_PA * Ap));

      // Mean axial speed of the slipstream far behind the propeller
      double u_inf = c_uPA * std::sqrt(1. + Cth);

      // Slipstream radius far behind the propeller (potential)
      double r_inf = r0 * std::sqrt(0.5 * (1. + c_uPA / u_inf));

      // Slipstream radius at rudder position (potential)
      double rinf_r0 = r_inf / r0;
      double rinf_r0_3 = std::pow(rinf_r0, 3);
      double x_r0_1_5 = std::pow(abs(x_pr_m) / r0, 1.5);

      double rx = r0 * (0.14 * rinf_r0_3 + rinf_r0 * x_r0_1_5) / (0.14 * rinf_r0_3 + x_r0_1_5);

      // Axial velocity at rudder position (potential)
      double rinf_rx = r_inf / rx;
      double ux = u_inf * rinf_rx * rinf_rx;

      // Turbulent mixing correction on radius
      double drx = 0.15 * x_pr_m * (ux - c_uPA) / (ux + c_uPA);

      // Corrected radius and axial velocities
      double r_RP = rx + drx; // corrected radius
      double r_rdr = rx / (r_RP);
      auto u_corr = (ux - c_uPA) * r_rdr * r_rdr + c_uPA; // corrected axial velocity

      // Correction for the influence of lateral variation of flow speed
      double d = sqrt(MU_PI_2) * r_RP;
      double f = 2. * std::pow(2. / (2. + d / c), 8);
      // FIXME : pow not defined for negative c_uPA / c_uRP
      double lambda = std::pow(c_uPA / u_corr, f);

      // Influence of the hull in front of the rudder
      c_uRP = (u_corr * u_corr + t * u_NWU_propeller_ms * u_NWU_propeller_ms) / u_corr;
      r_RP *= sqrt(u_corr/c_uRP);

//      c_uRP = u_corr;

      // Rudder area seen by the slipstream
      c_A_RP_m2 = 2. * r_RP < h_R ? (2. * r_RP / h_R) * A_R : A_R;

      // TODO: ici, on calcule les efforts de portance et de trainee

      c_vRP = c_vRA; // Radial velocity at the rudder position


      /// Debut du code replique
      // Drift angle in the slipstream
      c_beta_RP_rad = std::atan2(c_vRP, c_uRP);

      // Attack angle in the slipstream
      c_alpha_RP_rad = mathutils::Normalize__PI_PI(this->c_rudder_angle_rad - c_beta_RP_rad);

      // Get Coefficients
      double cl_RP, cd_RP, cn_RP;
      this->m_rudder->GetClCdCn(c_alpha_RP_rad, this->c_rudder_angle_rad, cl_RP, cd_RP, cn_RP);
      cl_RP *= lambda; // Influence of lateral variation of flow speed

      // Stagnation pressure ar rudder level
      double q_RP = 0.5 * water_density * (c_uRP * c_uRP + c_vRP * c_vRP);

      // Computing loads at rudder in the slipstream
      c_drag_RP_N = q_RP * cd_RP * c_A_RP_m2;
      c_lift_RP_N = q_RP * cl_RP * c_A_RP_m2;
      c_torque_RP_Nm = q_RP * cn_RP * c_A_RP_m2 * c;

      // Projection to the ship frame
      double Cbeta_RP = std::cos(c_beta_RP_rad);
      double Sbeta_RP = std::sin(c_beta_RP_rad);

      // Hull/rudder interactions
      c_torque_RP_Nm += a_H * (x_H - x_R) * c_lift_RP_N * Cbeta_RP;
      c_lift_RP_N *= (1. + a_H);

      c_fx_RP_N = Cbeta_RP * c_drag_RP_N - Sbeta_RP * c_lift_RP_N;
      c_fy_RP_N = Sbeta_RP * c_drag_RP_N + Cbeta_RP * c_lift_RP_N;
    }

    /// Fin du code replique


    /**
     * Dealing with rudder forces OUTSIDE the slipstream of the propeller (RA) (no influence of the propeller)
     */

    // Rudder area outside of the slipstream
    c_A_RA_m2 = A_R - c_A_RP_m2;

    // test if rudder has area outside the slipstream
    if (c_A_RA_m2 > 0) {

      /// Debut du code replique
      // Drift angle outside the slipstream
      c_beta_RA_rad = std::atan2(c_vRA, c_uRA);

      // Attack angle outside the slipstream
      c_alpha_RA_rad = mathutils::Normalize__PI_PI(this->c_rudder_angle_rad - c_beta_RA_rad);

      // Get Coefficients
      double cl_RA, cd_RA, cn_RA;
      this->m_rudder->GetClCdCn(c_alpha_RA_rad, this->c_rudder_angle_rad, cl_RA, cd_RA, cn_RA);

      // Stagnation pressure ar rudder level
      double q_RA = 0.5 * water_density * (c_uRA * c_uRA + c_vRA * c_vRA);

      // Computing loads at rudder outside the slipstream
      c_drag_RA_N = q_RA * cd_RA * c_A_RA_m2;
      c_lift_RA_N = q_RA * cl_RA * c_A_RA_m2;
      c_torque_RA_Nm = q_RA * cn_RA * c_A_RA_m2 * c;

      // Projection to the rudder frame
      double Cbeta_RA = std::cos(c_beta_RA_rad);
      double Sbeta_RA = std::sin(c_beta_RA_rad);

      // Hull/rudder interactions
      c_torque_RA_Nm += a_H * (x_H - x_R) * c_lift_RA_N * Cbeta_RA;
      c_lift_RA_N *= (1. + a_H);

      c_fx_RA_N = Cbeta_RA * c_drag_RA_N - Sbeta_RA * c_lift_RA_N;
      c_fy_RA_N = Sbeta_RA * c_drag_RA_N + Cbeta_RA * c_lift_RA_N;
      /// Fin du code replique
    } else {
      c_beta_RA_rad = 0.;
      c_alpha_RA_rad = this->c_rudder_angle_rad;
      c_lift_RA_N = 0.;
      c_drag_RA_N = 0.;
      c_torque_RA_Nm = 0.;
      c_fx_RA_N = 0.;
      c_fy_RA_N = 0.;
    }

    /**
     * Summing up rudder forces from outside and inside the propeller slipstream
     */

    c_fx_R_N = c_fx_RA_N + c_fx_RP_N;
    c_fy_R_N = c_fy_RA_N + c_fy_RP_N;
    // Transport of the rudder torque to the propeller location
    this->c_rudder_torque_Nm = (c_torque_RA_Nm + c_torque_RP_Nm) - x_pr_m * c_fy_R_N;

  }

  template<class Propeller, class Rudder>
  void BrixPropellerRudder<Propeller, Rudder>::DefineLogMessages(hermes::Message *propeller_message,
                                                             hermes::Message *rudder_message) {

    PropellerRudder<Propeller, Rudder>::DefineLogMessages(propeller_message, rudder_message);

    // Propeller
    propeller_message->AddField<double>("uPA", "m/s",
                                        "Longitudinal velocity at the propeller position, in propeller reference frame",
                                        [this]() { return c_uPA; });

    propeller_message->AddField<double>("vPA", "m/s",
                                        "Transverse velocity at the propeller position, in propeller reference frame",
                                        [this]() { return c_vPA; });

    // Rudder
    //        outside slipstream

    rudder_message->AddField<double>("area_RA", "m2", "Rudder area outside the slipstream",
                                     [this]() { return c_A_RA_m2 ; });

    rudder_message->AddField<double>("DriftAngle_RA", "rad", "Drift angle outside the slipstream",
                                     [this]() { return c_beta_RA_rad; });

    rudder_message->AddField<double>("AttackAngle_RA", "rad", "Attack angle outside the slipstream",
                                     [this]() { return c_alpha_RA_rad; });

    rudder_message->AddField<double>("u_RA", "m/s", "Longitudinal velocity",
                                     [this]() { return c_uRA; });

    rudder_message->AddField<double>("v_RA", "m/s", "transversal velocity",
                                     [this]() { return c_vRA; });

    rudder_message->AddField<double>("Drag_RA", "N", "Drag delivered by the part of the rudder outside the slipstream",
                                     [this]() { return c_drag_RA_N; });

    rudder_message->AddField<double>("Lift_RA", "N", "Lift delivered by the part of the rudder outside the slipstream",
                                     [this]() { return c_lift_RA_N; });

    rudder_message->AddField<double>("Torque_RA", "Nm", "Torque delivered by the part of the rudder outside the slipstream",
                                     [this]() { return c_torque_RA_Nm; });


    //        inside slipstream

    rudder_message->AddField<double>("area_RP", "m2", "Rudder area in the slipstream",
                                     [this]() { return c_A_RP_m2; });

    rudder_message->AddField<double>("DriftAngle_RP", "rad", "Drift angle inside the slipstream",
                                     [this]() { return c_beta_RP_rad; });

    rudder_message->AddField<double>("AttackAngle_RP", "rad", "Attack angle inside the slipstream",
                                     [this]() { return c_alpha_RP_rad; });

    rudder_message->AddField<double>("u_RP", "m/s", "Longitudinal velocity",
                                     [this]() { return c_uRP; });

    rudder_message->AddField<double>("v_RP", "m/s", "transversal velocity",
                                     [this]() { return c_vRP; });

    rudder_message->AddField<double>("Drag_RP", "N", "Drag delivered by the part of the rudder in the slipstream",
                                     [this]() { return c_drag_RP_N; });

    rudder_message->AddField<double>("Lift_RP", "N", "Lift delivered by the part of the rudder in the slipstream",
                                     [this]() { return c_lift_RP_N; });

    rudder_message->AddField<double>("Torque_RP", "Nm", "Torque delivered by the part of the rudder in the slipstream",
                                     [this]() { return c_torque_RP_Nm; });

  }


}  // end namespace acme