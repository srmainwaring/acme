//
// Created by frongere on 09/08/2021.
//


#include "PropellerRudder.h"

namespace acme {

  template<class Propeller, class Rudder>
  PropellerRudder<Propeller, Rudder>::PropellerRudder(const PropellerParams &thruster_params,
                                                      const std::string &thruster_perf_data_json_string,
                                                      const RudderParams &rudder_params,
                                                      const std::string &rudder_perf_data_json_string) :
      m_propeller(std::make_unique<Propeller>(thruster_params, thruster_perf_data_json_string)),
      m_rudder(std::make_unique<Rudder>(rudder_params, rudder_perf_data_json_string)) {}

  template<class Propeller, class Rudder>
  void PropellerRudder<Propeller, Rudder>::Initialize() {
    m_propeller->Initialize();
    m_rudder->Initialize();
  }

  template<class Propeller, class Rudder>
  void PropellerRudder<Propeller, Rudder>::Compute(const double &water_density,
                                                   const double &u_NWU_propeller, // u_P0
                                                   const double &v_NWU_propeller, // v_P0
                                                   const double &r, // Vessel
                                                   const double &xr,
                                                   const double &rpm,
                                                   const double &pitch_ratio,
                                                   const double &rudder_angle_deg) const {

    /**
     * Solving for propeller action directly using propeller classes implementation
     * ref : Manoeuvring Technical Manual, Brix, Soder, 1992, p84
     * https://drive.google.com/file/d/195jz2YHRuhX3tSrqEPNJkYZg_7ZVTcHn/view?usp=sharing
     */
    m_propeller->Compute(water_density,
                         u_NWU_propeller,
                         v_NWU_propeller,
                         rpm,
                         pitch_ratio);

    PropellerParams propeller_params = m_propeller->GetParameters();
    RudderParams rudder_params = m_rudder->GetParameters();

    /*
     * Computing velocities seen by the rudder outside the slipstream but taking into account the wake fraction
     */
    double wr0 = rudder_params.m_hull_wake_fraction_0; // rudder wake fraction in straight line

    double u_R0 = u_NWU_propeller;
    double v_R0 = v_NWU_propeller + r * xr;  // Transport of the propeller velocity to the rudder position

    double rudder_sidewash_angle_0 = std::atan2(v_R0, u_R0);

    // Estimated wake fraction for the rudder
    double wR = wr0 * std::exp(-4. * rudder_sidewash_angle_0 * rudder_sidewash_angle_0);

    c_uRA = u_R0 * (1 - wR);

    // TODO: ici, pour calculer vRA, on peut appliquer la correction kappa
    c_vRA = v_R0;
    //FIXME
//    if (rudder_params.m_use_transverse_velocity_correction) {
//
//    }

    c_rudder_angle_rad = rudder_angle_deg * MU_PI_180;

    // Mean axial speed of inflow to the propeller (with wake fraction correction included)
    c_uPA = m_propeller->GetAdvanceVelocity();
    c_vPA = v_NWU_propeller;

    // Propeller data
    double r0 = propeller_params.m_diameter_m;  // Propeller radius
    double Ap = MU_PI * r0 * r0;  // Propeller disk area

    // Rudder data
    double A_R = rudder_params.m_lateral_area_m2;// Rudder total area
    double c = rudder_params.m_chord_m;// Rudder chord length at its half height
    double h_R = rudder_params.m_height_m;// Rudder height

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
      c_alpha_RP_rad = c_rudder_angle_rad;
      c_drag_RP_N = 0.;
      c_lift_RP_N = 0.;
      c_torque_RP_Nm = 0.;
      c_fx_RP_N = 0.;
      c_fy_RP_N = 0.;
    } else {
      // Thrust loading coefficient
      double Cth = std::abs(m_propeller->GetThrust() / (q_PA * Ap));

      // Mean axial speed of the slipstream far behind the propeller
      double u_inf = c_uPA * std::sqrt(1. + Cth);

      // Slipstream radius far behind the propeller (potential)
      double r_inf = r0 * std::sqrt(0.5 * (1. + c_uPA / u_inf));

      // Slipstream radius at rudder position (potential)
      double rinf_r0 = r_inf / r0;
      double rinf_r0_3 = std::pow(rinf_r0, 3);
      double x_r0_1_5 = std::pow(xr / r0, 1.5);

      double rx = r0 * (0.14 * rinf_r0_3 + rinf_r0 * x_r0_1_5) / (0.14 * rinf_r0_3 + x_r0_1_5);

      // Axial velocity at rudder position (potential)
      double rinf_rx = r_inf / rx;
      double ux = u_inf * rinf_rx * rinf_rx;

      // Turbulent mixing correction on radius
      double drx = 0.15 * xr * (ux - c_uPA) / (ux + c_uPA);

      // Corrected radius and axial velocities
      double r_RP = rx + drx; // corrected radius
      double r_rdr = rx / (r_RP);
      c_uRP = (ux - c_uPA) * r_rdr * r_rdr + c_uPA; // corrected axial velocity

      // Rudder area seen by the slipstream
      c_A_RP_m2 = 2. * r_RP < h_R ? (2. * r_RP / h_R) * A_R : A_R;

      // TODO: ici, on calcule les efforts de portance et de trainee

      c_vRP = c_vRA; // Radial velocity at the rudder position


      /// Debut du code replique
      // Drift angle in the slipstream
      c_beta_RP_rad = std::atan2(c_vRP, c_uRP);

      // Attack angle in the slipstream
      c_alpha_RP_rad = mathutils::Normalize_0_2PI(c_rudder_angle_rad - c_beta_RP_rad);

      // Get Coefficients
      double cl_RP, cd_RP, cn_RP;
      m_rudder->GetClCdCn(c_alpha_RP_rad, c_rudder_angle_rad, cl_RP, cd_RP, cn_RP);

      // Correction for the influence of lateral variation of flow speed
      double d = 0.886 * r_RP;
      double f = 2. * std::pow(2. / (2. + d / c), 8);
      double lambda = std::pow(c_uPA / c_uRP, f);
      cl_RP *= lambda;

      // Stagnation pressure ar rudder level
      double q_RP = 0.5 * water_density * (c_uRP * c_uRP + c_vRP * c_vRP);

      // Computing loads at rudder in the slipstream
      c_lift_RP_N = q_RP * cl_RP * c_A_RP_m2; // FIXME: prise en compte du signe de alpha_RP_rad ??
      c_drag_RP_N = q_RP * cd_RP * c_A_RP_m2;
      c_torque_RP_Nm = q_RP * cn_RP * c_A_RP_m2 * c;

      // Projection
      double Cbeta_RP = std::cos(c_beta_RP_rad);
      double Sbeta_RP = std::sin(c_beta_RP_rad);

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
      c_alpha_RA_rad = mathutils::Normalize_0_2PI(c_rudder_angle_rad - c_beta_RA_rad);

      // Get Coefficients
      double cl_RA, cd_RA, cn_RA;
      m_rudder->GetClCdCn(c_alpha_RA_rad, c_rudder_angle_rad, cl_RA, cd_RA, cn_RA);

      // Stagnation pressure ar rudder level
      double q_RA = 0.5 * water_density * (c_uRA * c_uRA + c_vRA * c_vRA);

      // Computing loads at rudder outside the slipstream
      c_lift_RA_N = q_RA * cl_RA * c_A_RA_m2; // FIXME: prise en compte du signe de alpha_RP_rad ??
      c_drag_RA_N = q_RA * cd_RA * c_A_RA_m2;
      c_torque_RA_Nm = q_RA * cn_RA * c_A_RA_m2 * c;

      // Projection
      double Cbeta_RA = std::cos(c_beta_RA_rad);
      double Sbeta_RA = std::sin(c_beta_RA_rad);

      c_fx_RA_N = Cbeta_RA * c_drag_RA_N - Sbeta_RA * c_lift_RA_N;
      c_fy_RA_N = Sbeta_RA * c_drag_RA_N + Cbeta_RA * c_lift_RA_N;
      /// Fin du code replique
    } else {
      c_beta_RA_rad = 0.;
      c_alpha_RA_rad = c_rudder_angle_rad;
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
    c_torque_R_Nm =
        (c_torque_RA_Nm + c_torque_RP_Nm) - xr * c_fy_R_N;  // Transport of the rudder torque to the propeller location

  }

  template<class Propeller, class Rudder>
  double PropellerRudder<Propeller, Rudder>::GetPropellerThrust() const {
    return m_propeller->GetThrust();
  }

  template<class Propeller, class Rudder>
  double PropellerRudder<Propeller, Rudder>::GetPropellerTorque() const {
    return m_propeller->GetTorque();
  }

  template<class Propeller, class Rudder>
  double PropellerRudder<Propeller, Rudder>::GetPropellerEfficiency() const {
    return m_propeller->GetPropellerEfficiency();
  }

  template<class Propeller, class Rudder>
  double PropellerRudder<Propeller, Rudder>::GetPropellerPower() const {
    return m_propeller->GetPower();
  }

  template<class Propeller, class Rudder>
  double PropellerRudder<Propeller, Rudder>::GetPropellerRudderFx() const {
    return c_fx_R_N + GetPropellerThrust();
  }

  template<class Propeller, class Rudder>
  double PropellerRudder<Propeller, Rudder>::GetPropellerRudderFy() const {
    return c_fy_R_N;
  }

  template<class Propeller, class Rudder>
  double PropellerRudder<Propeller, Rudder>::GetPropellerRudderMz() const {
    return c_torque_R_Nm;
  }

  template<class Propeller, class Rudder>
  void PropellerRudder<Propeller, Rudder>::DefineLogMessages(hermes::Message *propeller_message,
                                                             hermes::Message *rudder_message) {

    // Propeller
    propeller_message->AddField<double>("Thrust", "N", "Thrust delivered by the propeller",
                                        [this]() { return GetPropellerThrust(); });

    propeller_message->AddField<double>("Torque", "Nm", "Torque delivered by the propeller",
                                        [this]() { return GetPropellerTorque(); });

    propeller_message->AddField<double>("Power", "W", "Power delivered by the propeller",
                                        [this]() { return GetPropellerPower(); });

    propeller_message->AddField<double>("uPA", "m/s",
                                        "Longitudinal velocity at the propeller position, in propeller reference frame",
                                        [this]() { return c_uPA; });

    propeller_message->AddField<double>("vPA", "m/s",
                                        "Transverse velocity at the propeller position, in propeller reference frame",
                                        [this]() { return c_vPA; });

    // Rudder

    rudder_message->AddField<double>("fx", "N", "Total longitudinal force delivered by the rudder",
                                     [this]() { return c_fx_R_N; });

    rudder_message->AddField<double>("fy", "N", "Total transversal force delivered by the rudder",
                                     [this]() { return c_fy_R_N; });

    rudder_message->AddField<double>("mz_R", "Nm", "Total torque delivered by the rudder, at the rudder position",
                                     [this]() { return c_torque_RA_Nm + c_torque_RP_Nm; });

    rudder_message->AddField<double>("mz_P", "Nm", "Total torque delivered by the rudder, at the propeller position",
                                     [this]() { return c_torque_R_Nm; });


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