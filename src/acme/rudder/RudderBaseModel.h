//
// Created by lletourn on 04/11/2021.
//

#ifndef ACME_RUDDERBASEMODEL_H
#define ACME_RUDDERBASEMODEL_H

#include <string>

#include "MathUtils/LookupTable1D.h"
#include "MathUtils/Angles.h"

#include "RudderModelType.h"

#include "hermes/hermes.h"

namespace acme {

  struct RudderParams {
    double m_lateral_area_m2; // Rudder lateral projected area (m**2)
    double m_chord_m; // Rudder chord length at its half height
    double m_height_m;

    // Hull/propeller/rudder interaction coefficients
    bool m_has_hull_influence = true;
    bool m_has_hull_influence_transverse_velocity = false;
    double m_tR = 0;  // steering resistance deduction factor
    double m_aH = 0.; // correction factor for the rudder lift force (in the vessel frame)
    double m_xR = 0.; // longitudinal position of the rudder, relatively to the ship COG (approx -0.5 Lpp)
    double m_xH = 0.; // longitudinal hydrodynamic application point of the additional lift force, accounting for the
                      // hull influence. (approx -0.45 Lpp)

    double m_hull_wake_fraction_0 = 0.; // Straight-run hull wake fraction

    // Optional

    // For Flap rudder only
    double m_flap_slope = 0.; // only used for a flap rudder type

    // For Brix model only
    double m_distance_nose_stock_m; // Longitudinal distance from the rudder nose to its stock (0.25-0.5 x chord)
    double m_Cf=0.; // ITTC57 frictional resistance coefficient, can be computed with function compute_ITTC57_frictional_resistance_coefficient()
    double m_Cq=1.; // Rudder resistance coefficient, approximated to 1 by Brix for rudder with sharp upper and lower edges. Smaller values for rounded edges (see above 1.2.11)

  };

  double compute_ITTC57_frictional_resistance_coefficient(double rudder_chord_m,
                                                          double mean_velocity_ms,
                                                          double nu_water = 1.15E-6);


  class RudderBaseModel {

   public:
    explicit RudderBaseModel(const RudderParams &params);

    virtual void Initialize() = 0;

    void InitializeLog(hermes::Message* msg);

    void Finalize(double time);

    void Log(bool is_logged);

    virtual void
    Compute(const double &water_density,
            const double &u_NWU,
            const double &v_NWU,
            const double &rudder_angle_deg,
            const double &u_ship_NWU,
            const double &v_ship_NWU,
            const double &r_ship_NWU,
            const double &x_r) const;

    static double HullStraighteningFunction(const double & beta);

    RudderModelType GetRudderModelType() const;

    const RudderParams &GetParameters() const;

    double GetFx() const { return c_fx_N; }

    double GetFy() const { return c_fy_N; }

    double GetMz() const { return c_torque_Nm; }

    double GetDrag() const { return c_drag_N; }

    double GetLift() const { return c_lift_N; }

    double GetTorque() const { return c_torque_Nm; }

    double GetDriftAngle(mathutils::ANGLE_UNIT unit) const {
      return unit == mathutils::DEG ? c_beta_R_rad * RAD2DEG : c_beta_R_rad;
    }

    double GetAttackAngle(mathutils::ANGLE_UNIT unit) const {
      return unit == mathutils::DEG ? c_alpha_R_rad * RAD2DEG : c_alpha_R_rad;
    }

   protected:

    virtual void ComputeLoads(const double &water_density) const = 0;

    bool m_is_initialized;

    RudderParams m_params;

    RudderModelType m_type;

    double m_max_alpha_R_rad{};
    double m_min_alpha_R_rad{};

    bool m_is_logged;

    mutable double c_rudder_angle_rad{};
    mutable double c_beta_R_rad{};
    mutable double c_alpha_R_rad{};
    mutable double c_lift_N{};
    mutable double c_drag_N{};
    mutable double c_torque_Nm{};
    mutable double c_fx_N{};
    mutable double c_fy_N{};
    mutable double c_u_NWU{};
    mutable double c_v_NWU{};
    mutable double c_uRA{};
    mutable double c_vRA{};

  };


}  // end namespace acme

#endif //ACME_RUDDERBASEMODEL_H
