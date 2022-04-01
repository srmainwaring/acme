//
// Created by frongere on 09/08/2021.
//

#ifndef ACME_FLAPRUDDERMODEL_H
#define ACME_FLAPRUDDERMODEL_H

#include <string>

#include "MathUtils/LookupTable2D.h"

#include "SimpleRudderModel.h"

namespace acme {

  class FlapRudderModel : public SimpleRudderModel {

   public:
    FlapRudderModel(const RudderParams params);

    void GetClCdCn(const double &attack_angle_rad,
                   const double &rudder_angle_rad,
                   double &cl,
                   double &cd,
                   double &cn) const override;

   private:
    void ParseRudderPerformanceCurveJsonString() override;

   private:
    mathutils::LookupTable2d<double> m_cl_cd_cn_coeffs;

    double m_max_flap_angle_rad;
    double m_min_flap_angle_rad;

  };

  void ParseFlapRudderJsonString(const std::string &json_string,
                             std::vector<double> &attack_angle_rad,
                             std::vector<double> &flap_angle_rad,
                             std::vector<double> &cd,
                             std::vector<double> &cl,
                             std::vector<double> &cn);

}  // end namespace acme

#endif //ACME_FLAPRUDDERMODEL_H
