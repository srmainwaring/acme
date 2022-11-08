//
// Created by frongere on 04/08/2021.
//

#ifndef ACME_PROPELLERBASEMODEL_H
#define ACME_PROPELLERBASEMODEL_H

#include "MathUtils/Vector3d.h"
#include "PropellerModelType.h"
#include "hermes/hermes.h"

namespace acme {

  enum SCREW_DIRECTION {
    RIGHT_HANDED,
    LEFT_HANDED
  };

  struct PropellerParams {
    double m_diameter_m;
    SCREW_DIRECTION m_screw_direction;

    double m_hull_wake_fraction_0;
    double m_thrust_deduction_factor_0;

    // TODO: Donnees a ajouter dans le json
    bool m_use_advance_velocity_correction_factor = false;
    double m_propeller_design_rpm;
    double m_vessel_design_speed_ms;

    double m_thrust_coefficient_correction = 0.;
    double m_torque_coefficient_correction = 0.;

    // contains open water curve json file content
    std::string m_thruster_perf_data_json_string;
  };


  class PropellerBaseModel {

   public:
    virtual ~PropellerBaseModel();

    PropellerBaseModel(const PropellerParams &params, PropellerModelType type);

    virtual void Initialize();

    void DefineLogMessages(hermes::Message* msg);

    /// Compute the models with the specified data
    /// \param water_density in kg/m3
    /// \param u propeller velocity with respect to water (current included) expressed along x-axis of the vessel (in m/s)
    /// \param v  propeller velocity with respect to water (current included) expressed along x-axis of the vessel (in m/s)
    /// \param rpm shaft rotational velocity in round per minutes
    /// \param pitch_ratio the propeller pitch ratio. Only used for CPP
    virtual void Compute(const double &water_density,
                         const double &u_NWU,
                         const double &v_NWU,
                         const double &rpm,
                         const double &pitch_ratio) const = 0;

    PropellerModelType GetThrusterModelType() const;

    const PropellerParams &GetParameters() const;

    double GetAdvanceVelocity() const;

    double GetThrust() const;

    double GetTorque() const;

    double GetPropellerEfficiency() const;

    double GetPower() const;


   protected:

    double GetPropellerAdvanceVelocity(const double &u_NWU,
                                       const double &v_NWU) const;

    SCREW_DIRECTION GetScrewDirection() const {
      return m_params.m_screw_direction;
    }

    inline signed int GetScrewDirectionSign() const {
      return m_params.m_screw_direction == RIGHT_HANDED ? 1 : -1;
    }

   private:
    virtual void ParsePropellerPerformanceCurveJsonString() = 0;

    void ComputeAdvanceVelocityCorrectionFactor();


   protected:
    bool m_is_initialized;

    PropellerModelType m_type;

    PropellerParams m_params;
    double m_ku;

    mutable double c_uPA; // Propeller advance velocity
    mutable double c_sidewash_angle_rad;
    mutable double c_thrust_N;
    mutable double c_torque_Nm;
    mutable double c_efficiency;
    mutable double c_power_W;

  };

}

#endif //ACME_PROPELLERBASEMODEL_H
