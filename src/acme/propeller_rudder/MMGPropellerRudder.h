//
// Created by lletourn on 01/04/2022.
//

#ifndef ACME_MMGPROPELLERRUDDER_H
#define ACME_MMGPROPELLERRUDDER_H

#include "PropellerRudderBase.h"

namespace acme {

  template<class Rudder>
  class MMGPropellerRudder : public PropellerRudder<FPP1Q, Rudder> {

   public:
    MMGPropellerRudder(const PropellerParams &thruster_params, const RudderParams &rudder_params);

    void
    Compute(const double &water_density, const double &u_NWU_propeller_ms, const double &v_NWU_propeller_ms,
            const double &u_NWU_ship_ms, const double &v_NWU_ship_ms, const double &r_rads, const double &x_pr_m,
            const double &x_gr_m, const double &rpm, const double &pitch_ratio,
            const double &rudder_angle_deg) const override;

    void DefineLogMessages(hermes::Message *propeller_message, hermes::Message *rudder_message) override;

   private:

    double m_gamma_R; // flow straigthening factor
    double m_kappa;   // correction factor
    double m_eta;     // ratio of propeller diameter to rudder span
  };


  /// Build a propeller rudder model using type keys to get a custom combination of propeller and rudder model among the
  /// available models in acme.
  std::shared_ptr<PropellerRudderBase>
  build_MMG_pr(PropellerModelType prop_type,
                PropellerParams prop_params,
                RudderModelType rudder_type,
                RudderParams rudder_params) {

    std::shared_ptr<PropellerRudderBase> pr;

    switch (rudder_type) {
      case E_SIMPLE_RUDDER:
        pr = std::make_shared<MMGPropellerRudder<SimpleRudderModel>>(prop_params, rudder_params);
        break;
      case E_FLAP_RUDDER:
        pr = std::make_shared<MMGPropellerRudder<FlapRudderModel>>(prop_params, rudder_params);
        break;
      case E_FUJII_RUDDER:
        pr = std::make_shared<MMGPropellerRudder<FujiiRudderModel>>(prop_params, rudder_params);
        break;
      case E_BRIX_RUDDER:
        pr = std::make_shared<MMGPropellerRudder<BrixRudderModel>>(prop_params, rudder_params);
        break;
    }

    return pr;

  }


} // end namespace acme

#include "MMGPropellerRudder.hpp"

#endif //ACME_MMGPROPELLERRUDDER_H
