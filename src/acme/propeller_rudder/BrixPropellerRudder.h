//
// Created by lletourn on 11/04/2022.
//

#ifndef ACME_BRIXPROPELLERRUDDER_H
#define ACME_BRIXPROPELLERRUDDER_H

#include "PropellerRudderBase.h"

namespace acme {

  template<class Propeller, class Rudder>
  class BrixPropellerRudder : public PropellerRudder<Propeller, Rudder> {

   public:
    BrixPropellerRudder(const PropellerParams &thruster_params, const RudderParams &rudder_params);

    void
    Compute(const double &water_density, const double &u_NWU_propeller_ms, const double &v_NWU_propeller_ms,
            const double &u_NWU_ship_ms, const double &v_NWU_ship_ms, const double &r_rads, const double &x_pr_m,
            const double &x_gr_m, const double &rpm, const double &pitch_ratio,
            const double &rudder_angle_deg) const override;

    double GetRudderFx() const override;

    double GetRudderFy() const override;

    double GetRudderMz() const override;

    void DefineLogMessages(hermes::Message *propeller_message, hermes::Message *rudder_message) override;

   private:

    // Propeller
    mutable double c_uPA; // Propeller advance velocity
    mutable double c_vPA;

    // Rudder
    mutable double c_fx_R_N;
    mutable double c_fy_R_N;

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
  build_Brix_pr(PropellerModelType prop_type,
                PropellerParams prop_params,
                RudderModelType rudder_type,
                RudderParams rudder_params) {

    std::shared_ptr<PropellerRudderBase> pr;

    switch (prop_type) {
      case E_FPP1Q:
        switch (rudder_type) {
          case E_SIMPLE_RUDDER:
            pr = std::make_shared<BrixPropellerRudder<FPP1Q, SimpleRudderModel>>(prop_params, rudder_params);
            break;
          case E_FLAP_RUDDER:
            pr = std::make_shared<BrixPropellerRudder<FPP1Q, FlapRudderModel>>(prop_params, rudder_params);
            break;
          case E_FUJII_RUDDER:
            pr = std::make_shared<BrixPropellerRudder<FPP1Q, FujiiRudderModel>>(prop_params, rudder_params);
            break;
          case E_BRIX_RUDDER:
            pr = std::make_shared<BrixPropellerRudder<FPP1Q, BrixRudderModel>>(prop_params, rudder_params);
            break;
        }
        break;
      case E_FPP4Q:
        switch (rudder_type) {
          case E_SIMPLE_RUDDER:
            pr = std::make_shared<BrixPropellerRudder<FPP4Q, SimpleRudderModel>>(prop_params, rudder_params);
            break;
          case E_FLAP_RUDDER:
            pr = std::make_shared<BrixPropellerRudder<FPP4Q, FlapRudderModel>>(prop_params, rudder_params);
            break;
          case E_FUJII_RUDDER:
            pr = std::make_shared<BrixPropellerRudder<FPP4Q, FujiiRudderModel>>(prop_params, rudder_params);
            break;
          case E_BRIX_RUDDER:
            pr = std::make_shared<BrixPropellerRudder<FPP4Q, BrixRudderModel>>(prop_params, rudder_params);
            break;
        }
        break;
      case E_CPP:
        switch (rudder_type) {
          case E_SIMPLE_RUDDER:
            pr = std::make_shared<BrixPropellerRudder<CPP, SimpleRudderModel>>(prop_params, rudder_params);
            break;
          case E_FLAP_RUDDER:
            pr = std::make_shared<BrixPropellerRudder<CPP, FlapRudderModel>>(prop_params, rudder_params);
            break;
          case E_FUJII_RUDDER:
            pr = std::make_shared<BrixPropellerRudder<CPP, FujiiRudderModel>>(prop_params, rudder_params);
            break;
          case E_BRIX_RUDDER:
            pr = std::make_shared<BrixPropellerRudder<CPP, BrixRudderModel>>(prop_params, rudder_params);
            break;
        }
        break;
    }

    return pr;
  }


} // end namespace acme

#include "BrixPropellerRudder.hpp"


#endif //ACME_BRIXPROPELLERRUDDER_H
