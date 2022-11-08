//
// Created by lletourn on 04/11/2021.
//

#ifndef ACME_FUJIIRUDDERMODEL_H
#define ACME_FUJIIRUDDERMODEL_H

#include "RudderBaseModel.h"

namespace acme {

  class FujiiRudderModel: public RudderBaseModel {

   public:
    virtual ~FujiiRudderModel();

    FujiiRudderModel(const RudderParams params);

    virtual void Initialize() override;

    virtual void GetClCdCn(const double &attack_angle_rad,
                           const double &rudder_angle_rad,
                           double &cl,
                           double &cd,
                           double &cn) const override;

   private:

    double m_f_alpha;

  };

} // end namespace acme
#endif //ACME_FUJIIRUDDERMODEL_H
