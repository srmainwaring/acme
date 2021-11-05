//
// Created by lletourn on 05/11/2021.
//

#ifndef ACME_BRIXRUDDERMODEL_H
#define ACME_BRIXRUDDERMODEL_H

#include "RudderBaseModel.h"

namespace acme {


  /**
   * \class BrixRudderModel
   * \brief Class for computing the rudder loads, based on estimation formulas for the drag, lift and torque at rudder's
   * stock, as given by Brix in
   * Brix, J. (1993). Manoeuvring technical manual. Hamburg, Germany: Seehafen Verlag. (eqs 1.2.1 to 1.2.11)
   * Requires :
   * - the longitudinal distance from the nose to the rudder's stock
   * - the rudder frictional resistance coefficient Cf, which can be computed using the ITTC57 formula
   */
  class BrixRudderModel : public RudderBaseModel {


   public:

    /// Brix rudder model, based on estimations for the drag, lift and torque at rudder's stock
    /// \param params rudder parameters
    /// \param distanceNoseToStock_m longitudinal from the rudder's nose to the rudder's stock, in m, positive
    /// \param Cf frictional resistance coefficient, can be computed using ITTC57 formula, see SetITTC57FrictionalResistanceCoefficient method
    /// \param Cq rudder resistance coefficient, approximated to 1 by Brix for rudder with sharp upper and lower edges. Smaller values for rounded edges (see above 1.2.11)
    explicit BrixRudderModel(const RudderParams &params,
                             double distanceNoseToStock_m,
                             double Cf=0.,
                             double Cq=1.);

    void Initialize() override;

    void SetITTC57FrictionalResistanceCoefficient(double mean_velocity,
                                                  mathutils::SPEED_UNIT unit,
                                                  double nu_water = 1.15E-6);

   protected:

    void ComputeLoads(const double &water_density) const override;

    virtual void GetClCdCn(const double &attack_angle_rad,
                           double &cl,
                           double &cd,
                           double &cn) const;

    double m_d; // Longitudinal distance from the rudder nose to its stock
    double m_Cf; // ITTC57 frictional resistance coefficient
    double m_Cq; // Rudder resistance coefficient

  };

} // end namespace acme
#endif //ACME_BRIXRUDDERMODEL_H
