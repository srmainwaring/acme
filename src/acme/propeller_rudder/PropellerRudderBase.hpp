//
// Created by frongere on 09/08/2021.
//


#include "PropellerRudderBase.h"

namespace acme {
  PropellerRudderBase::~PropellerRudderBase() {
  }

  template<class Propeller, class Rudder>
  PropellerRudder<Propeller, Rudder>::PropellerRudder(const PropellerParams &thruster_params,
                                                      const RudderParams &rudder_params) :
      m_propeller(std::make_unique<Propeller>(thruster_params)),
      m_rudder(std::make_unique<Rudder>(rudder_params)) {}

  template<class Propeller, class Rudder>
  void PropellerRudder<Propeller, Rudder>::Initialize() {
    m_propeller->Initialize();
    m_rudder->Initialize();
  }


  template<class Propeller, class Rudder>
  void PropellerRudder<Propeller, Rudder>::DefineLogMessages(hermes::Message *propeller_message,
                                                             hermes::Message *rudder_message) {

    rudder_message->AddField<double>("fx_prop_rudder", "N", "Total longitudinal force delivered by the prop-rudder",
                                     [this]() { return GetPropellerRudderFx(); });

    rudder_message->AddField<double>("fy_prop_rudder", "N", "Total transversal force delivered by the prop-rudder",
                                     [this]() { return GetPropellerRudderFy(); });

    rudder_message->AddField<double>("mz_prop_rudder", "Nm", "Total torque delivered by the prop-rudder, at the propeller position",
                                     [this]() { return GetPropellerRudderMz(); });

    // Propeller
    propeller_message->AddField<double>("Thrust", "N", "Thrust delivered by the propeller",
                                        [this]() { return GetPropellerThrust(); });

    propeller_message->AddField<double>("Torque", "Nm", "Torque delivered by the propeller",
                                        [this]() { return GetPropellerTorque(); });

    propeller_message->AddField<double>("Power", "W", "Power delivered by the propeller",
                                        [this]() { return GetPropellerPower(); });

    // Rudder
    rudder_message->AddField<double>("fx_rudder", "N", "Total longitudinal force delivered by the rudder",
                                     [this]() { return GetRudderFx(); });

    rudder_message->AddField<double>("fy_rudder", "N", "Total transversal force delivered by the rudder",
                                     [this]() { return GetRudderFy(); });

    rudder_message->AddField<double>("mz_rudder", "Nm", "Total torque delivered by the rudder, at the rudder position",
                                     [this]() { return GetRudderMz(); });

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
  double PropellerRudder<Propeller, Rudder>::GetRudderFx() const {
    return m_rudder->GetFx();
  }

  template<class Propeller, class Rudder>
  double PropellerRudder<Propeller, Rudder>::GetRudderFy() const {
    return m_rudder->GetFy();
  }

  template<class Propeller, class Rudder>
  double PropellerRudder<Propeller, Rudder>::GetRudderMz() const {
    return m_rudder->GetMz();
  }

  template<class Propeller, class Rudder>
  double PropellerRudder<Propeller, Rudder>::GetPropellerRudderFx() const {
    return GetPropellerThrust() + GetRudderFx();
  }

  template<class Propeller, class Rudder>
  double PropellerRudder<Propeller, Rudder>::GetPropellerRudderFy() const {
    return GetRudderFy();
  }

  template<class Propeller, class Rudder>
  double PropellerRudder<Propeller, Rudder>::GetPropellerRudderMz() const {
    return c_rudder_torque_Nm;
  }


}  // end namespace acme