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
   * - the rudder resistance coefficient Cq, approximated to 1 by Brix for rudder with sharp upper and lower edges. Smaller values for rounded edges (see above 1.2.11)
   */
  class BrixRudderModel : public RudderBaseModel {


   public:

    /// Brix rudder model, based on estimations for the drag, lift and torque at rudder's stock
    /// \param params rudder parameters
    /// \param rudder_perf_data_json_string not used, only defined to be used with the PropellerRudderModel class...
    explicit BrixRudderModel(const RudderParams &params,
                             const std::string &rudder_perf_data_json_string="");

    void Initialize() override;

    virtual void GetClCdCn(const double &attack_angle_rad,
                           const double &rudder_angle_rad,
                           double &cl,
                           double &cd,
                           double &cn) const;

   protected:

    void ComputeLoads(const double &water_density) const override;

  };

} // end namespace acme
#endif //ACME_BRIXRUDDERMODEL_H
