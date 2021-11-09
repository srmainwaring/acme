//
// Created by frongere on 09/08/2021.
//

#ifndef ACME_SIMPLERUDDERMODEL_H
#define ACME_SIMPLERUDDERMODEL_H

#include <string>

#include "RudderBaseModel.h"

#include "MathUtils/LookupTable1D.h"
#include "MathUtils/Angles.h"

#include "RudderModelType.h"

namespace acme {


  class SimpleRudderModel: public RudderBaseModel {

   public:
    SimpleRudderModel(const RudderParams &params,
                      const std::string &perf_data_json_string);

    void Initialize() override;

    void ComputeLoads(const double &water_density) const override;

    virtual void GetClCdCn(const double &attack_angle_rad,
                           const double &rudder_angle_rad,
                           double &cl,
                           double &cd,
                           double &cn) const;

   private:

    virtual void ParseRudderPerformanceCurveJsonString();

   protected:
    std::string m_temp_perf_data_json_string;

   private:
    mathutils::LookupTable1D<double, double> m_cl_cd_cn_coeffs;

  };

  void ParseRudderJsonString(const std::string &json_string,
                             std::vector<double> &attack_angle_rad,
                             std::vector<double> &cd,
                             std::vector<double> &cl,
                             std::vector<double> &cn);


}  // end namespace acme

#endif //ACME_SIMPLERUDDERMODEL_H
