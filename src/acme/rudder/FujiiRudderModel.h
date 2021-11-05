//
// Created by lletourn on 04/11/2021.
//

#ifndef ACME_FUJIIRUDDERMODEL_H
#define ACME_FUJIIRUDDERMODEL_H

#include "RudderBaseModel.h"

namespace acme {

  class FujiiRudderModel: public RudderBaseModel {

   public:
    FujiiRudderModel(const RudderParams params);

    void Initialize() override;

   protected:
    void ComputeLoads(const double &water_density) const override;

   private:

    double m_f_alpha;

  };

} // end namespace acme
#endif //ACME_FUJIIRUDDERMODEL_H
