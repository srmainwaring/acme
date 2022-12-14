//
// Created by frongere on 09/08/2021.
//

#ifndef ACME_PROPELLERRUDDERBASE_H
#define ACME_PROPELLERRUDDERBASE_H

#include <memory>

#include "acme/propeller/propeller.h"
#include "acme/rudder/rudder.h"

namespace acme {

  enum PropellerRudderModelType {
    E_BRIX,
    E_MMG
  };


  class PropellerRudderBase {

   public:

    virtual void Initialize() = 0;

    virtual void DefineLogMessages(hermes::Message *propeller_message, hermes::Message *rudder_message) = 0;

    /// Perform the model calculations
    /// \param water_density in kg/m**3
    /// \param u_NWU_propeller_ms axial velocity with respect to water at the propeller location as obtained from rigid
    ///        body motion of the vessel eventually (including current and waves orbital velocities) in m/s
    /// \param v_NWU_propeller_ms radial velocity with respect to water at the propeller location as obtained from rigid
    ///        body motion of the vessel eventually (including current and waves orbital velocities) in m/s
    /// \param u_NWU_ship_ms axial velocity with respect to water at the ship COG location as obtained from rigid
    ///        body motion of the vessel eventually (including current and waves orbital velocities) in m/s
    /// \param v_NWU_ship_ms radial velocity with respect to water at the ship COG location as obtained from rigid
    ///        body motion of the vessel eventually (including current and waves orbital velocities) in m/s
    /// \param r_rads vessel rotation velocity around its local z-axis (rad/s)
    /// \param x_pr_m distance between the propeller and the rudder in m. Accounted positive when the rudder is behind the
    ///           propeller
    /// \param x_gr_m longitudinal distance between the ship COG and the rudder in m. Accounted positive when the rudder
    ///         is behind the COG
    /// \param rpm Propeller screw rate of rotation (round per minute)
    /// \param pitch_ratio Propeller pitch ratio. Only used if the chosen propeller model is a CPP.
    /// \param rudder_angle_deg rudder angle (in deg) between the vessel x axis and the rudder chord.
    ///
    /// \note In this implementation, it is considered that the rudder stock is collinear to the vessel z axis and
    ///       directed upwards. As such, u_NWU_propeller_ms and v_NWU_propeller_ms are lying in the Oxy plane of the vessel.
    ///       Propeller rotation axis is also collinear to the vessel x axis.
    virtual void
    Compute(const double &water_density,
            const double &u_NWU_propeller_ms,
            const double &v_NWU_propeller_ms,
            const double &u_NWU_ship_ms,
            const double &v_NWU_ship_ms,
            const double &r_rads,
            const double &x_rp_m,
            const double &x_gr_m,
            const double &rpm,
            const double &pitch_ratio,
            const double &rudder_angle_deg) const = 0;

    virtual double GetPropellerThrust() const = 0;

    virtual double GetPropellerTorque() const = 0;

    virtual double GetPropellerEfficiency() const = 0;

    virtual double GetPropellerPower() const = 0;

    virtual double GetRudderFx() const = 0;

    virtual double GetRudderFy() const = 0;

    virtual double GetRudderMz() const = 0;

    virtual double GetPropellerRudderFx() const = 0;

    virtual double GetPropellerRudderFy() const = 0;

    virtual double GetPropellerRudderMz() const = 0;

  };


  template<class Propeller, class Rudder>
  class PropellerRudder : public PropellerRudderBase {

   public:
    PropellerRudder(const PropellerParams &thruster_params, const RudderParams &rudder_params);

    void Initialize() override;

    double GetPropellerThrust() const override;

    double GetPropellerTorque() const override;

    double GetPropellerEfficiency() const override;

    double GetPropellerPower() const override;

    double GetRudderFx() const override;

    double GetRudderFy() const override;

    double GetRudderMz() const override;

    double GetPropellerRudderFx() const override;

    double GetPropellerRudderFy() const override;

    double GetPropellerRudderMz() const override;

    virtual void DefineLogMessages(hermes::Message *propeller_message, hermes::Message *rudder_message);

   protected:
    std::unique_ptr<Propeller> m_propeller;
    std::unique_ptr<Rudder> m_rudder;

    mutable double c_rudder_angle_rad;
//    mutable double c_drift_angle_rad;
//    mutable double c_attack_angle_rad;
//    mutable double c_rudder_drift_N;
//    mutable double c_rudder_drag_N;
    mutable double c_rudder_torque_Nm;

  };


}  // end namespace acme

#include "PropellerRudderBase.hpp"

#endif //ACME_PROPELLERRUDDERBASE_H
