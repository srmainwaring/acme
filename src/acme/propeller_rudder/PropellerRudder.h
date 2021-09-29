//
// Created by frongere on 09/08/2021.
//

#ifndef ACME_PROPELLERRUDDER_H
#define ACME_PROPELLERRUDDER_H

#include <memory>

#include "acme/propeller/PropellerBaseModel.h"
#include "acme/rudder/SimpleRudderModel.h"
#include "hermes/hermes.h"

namespace acme {

  class PropellerRudderBase {

   public:
    virtual void Initialize() = 0;

    /// Perform the model calculations
    /// \param water_density in kg/m**3
    /// \param u_NWU_propeller_ms axial velocity with respect to water at the propeller location as obtained from rigid
    ///        body motion of the vessel eventually (including current and waves orbital velocities) in m/s
    /// \param v_NWU_propeller_ms radial velocity with respect to water at the propeller location as obtained from rigid
    ///        body motion of the vessel eventually (including current and waves orbital velocities) in m/s
    /// \param r_rads vessel rotation velocity around its local z-axis (rad/s)
    /// \param xr_m distance between the propeller and the rudder in m. Accounted positive when the rudder is behind the
    ///           propeller
    /// \param rpm Propeller screw rate of rotation (round per minute)
    /// \param pitch_ratio Propeller pitch ratio. Only used if the chosen propeller model is a CPP.
    /// \param rudder_angle_deg rudder angle (in deg) between the vessel x axis and the rudder chord.
    ///
    /// \note In this implementation, it is considered that the rudder stock is collinear to the vessel z axis and
    ///       directed upwards. As such, u_NWU_propeller_ms and v_NWU_propeller_ms are lying in the Oxy plane of the vessel.
    ///       Propeller rotation axis is also collinear to the vessel x axis.
    virtual void Compute(const double &water_density,
                         const double &u_NWU_propeller_ms,
                         const double &v_NWU_propeller_ms,
                         const double &r_rads,
                         const double &xr_m,
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

    virtual void DefineLogMessages(hermes::Message* propeller_message, hermes::Message* rudder_message) = 0;

   protected:

    mutable double c_rudder_angle_rad;
    mutable double c_drift_angle_rad;
    mutable double c_attack_angle_rad;
    mutable double c_rudder_drift_N;
    mutable double c_rudder_drag_N;
    mutable double c_rudder_torque_Nm;

  };

  template<class Propeller, class Rudder>
  class PropellerRudder : public PropellerRudderBase {

   public:
    PropellerRudder(const PropellerParams &thruster_params,
                    const std::string &thruster_perf_data_json_string,
                    const RudderParams &rudder_params,
                    const std::string &rudder_perf_data_json_string);

    void Initialize() override;


    void Compute(const double &water_density,
                 const double &u_NWU_propeller,
                 const double &v_NWU_propeller,
                 const double &r,
                 const double &xr,
                 const double &rpm,
                 const double &pitch_ratio,
                 const double &rudder_angle_deg) const override;

    double GetPropellerThrust() const override;

    double GetPropellerTorque() const override;

    double GetPropellerEfficiency() const override;

    double GetPropellerPower() const override;

    double GetRudderFx() const override { return c_fx_R_N; };

    double GetRudderFy() const override { return c_fy_R_N; };

    double GetRudderMz() const override { return c_torque_R_Nm; };

    double GetPropellerRudderFx() const override;

    double GetPropellerRudderFy() const override;

    double GetPropellerRudderMz() const override;

    void DefineLogMessages(hermes::Message* propeller_message, hermes::Message* rudder_message) override;

   private:
    std::unique_ptr<Propeller> m_propeller;
    std::unique_ptr<Rudder> m_rudder;

    // Propeller
    mutable double c_uPA; // Propeller advance velocity
    mutable double c_vPA;

    // Rudder
    mutable double c_rudder_angle_rad;
    mutable double c_fx_R_N;
    mutable double c_fy_R_N;
    mutable double c_torque_R_Nm;

    //    Outside slipstream
    mutable double c_A_RA_m2;
    mutable double c_uRA;
    mutable double c_vRA;
    mutable double c_alpha_RA_rad; //drift angle
    mutable double c_beta_RA_rad; //drift angle
    mutable double c_drag_RA_N;
    mutable double c_lift_RA_N;
    mutable double c_torque_RA_Nm;
    mutable double c_fx_RA_N;
    mutable double c_fy_RA_N;

    //    Inside slipstream
    mutable double c_A_RP_m2;
    mutable double c_uRP;
    mutable double c_vRP;
    mutable double c_alpha_RP_rad; //drift angle
    mutable double c_beta_RP_rad; //drift angle
    mutable double c_drag_RP_N;
    mutable double c_lift_RP_N;
    mutable double c_torque_RP_Nm;
    mutable double c_fx_RP_N;
    mutable double c_fy_RP_N;


  };

  /// Build a propeller rudder model using type keys to get a custom combination of propeller and rudder model among the
  /// available models in acme.
  std::shared_ptr<PropellerRudderBase>
  build_pr(PropellerModelType prop_type,
           PropellerParams prop_params,
           const std::string &prop_perf_data_string,
           RudderModelType rudder_type,
           RudderParams rudder_params,
           const std::string &rudder_perf_data_string) {

    std::shared_ptr<PropellerRudderBase> pr;

    switch (prop_type) {
      case E_FPP1Q:
        switch (rudder_type) {
          case E_SIMPLE_RUDDER:
            pr = std::make_shared<PropellerRudder<FPP1Q, SimpleRudderModel>>(prop_params,
                                                                             prop_perf_data_string,
                                                                             rudder_params,
                                                                             rudder_perf_data_string);

            break;
          case E_FLAP_RUDDER:
            pr = std::make_shared<PropellerRudder<FPP1Q, FlapRudderModel>>(prop_params,
                                                                           prop_perf_data_string,
                                                                           rudder_params,
                                                                           rudder_perf_data_string);
            break;
        }
        break;
      case E_FPP4Q:
        switch (rudder_type) {
          case E_SIMPLE_RUDDER:
            pr = std::make_shared<PropellerRudder<FPP4Q, SimpleRudderModel>>(prop_params,
                                                                             prop_perf_data_string,
                                                                             rudder_params,
                                                                             rudder_perf_data_string);
            break;
          case E_FLAP_RUDDER:
            pr = std::make_shared<PropellerRudder<FPP4Q, FlapRudderModel>>(prop_params,
                                                                           prop_perf_data_string,
                                                                           rudder_params,
                                                                           rudder_perf_data_string);
            break;
        }
        break;
      case E_CPP:
        switch (rudder_type) {
          case E_SIMPLE_RUDDER:
            pr = std::make_shared<PropellerRudder<CPP, SimpleRudderModel>>(prop_params,
                                                                           prop_perf_data_string,
                                                                           rudder_params,
                                                                           rudder_perf_data_string);
            break;
          case E_FLAP_RUDDER:
            pr = std::make_shared<PropellerRudder<CPP, FlapRudderModel>>(prop_params,
                                                                         prop_perf_data_string,
                                                                         rudder_params,
                                                                         rudder_perf_data_string);
            break;
        }
        break;
    }

    return pr;
  }


}  // end namespace acme

#include "PropellerRudder.hpp"

#endif //ACME_PROPELLERRUDDER_H
