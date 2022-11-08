//
// Created by frongere on 09/08/2021.
//

#include "SimpleRudderModel.h"

#include <iostream>
#include <vector>
#include <nlohmann/json.hpp>
#include "MathUtils/Unit.h"

using json = nlohmann::json;

namespace acme {

  SimpleRudderModel::~SimpleRudderModel() {
  }

  SimpleRudderModel::SimpleRudderModel(const RudderParams &params) :
  RudderBaseModel(params),
  m_cl_cd_cn_coeffs(mathutils::LINEAR){

  }

  void SimpleRudderModel::Initialize() {
    ParseRudderPerformanceCurveJsonString();
    m_is_initialized = true;
  }

  void SimpleRudderModel::GetClCdCn(const double &attack_angle_rad,
                                    const double &rudder_angle_rad,
                                    double &cl,
                                    double &cd,
                                    double &cn) const {

    try {
      cl = m_cl_cd_cn_coeffs.Eval("cl", attack_angle_rad);
      cd = m_cl_cd_cn_coeffs.Eval("cd", attack_angle_rad);
      cn = m_cl_cd_cn_coeffs.Eval("cn", attack_angle_rad);
    } catch (std::exception &e) {
      std::cerr << "SimpleRudder : attack angle exceed interpolator range : " << e.what() << std::endl;
      exit(EXIT_FAILURE);
    }
  }


  void SimpleRudderModel::ParseRudderPerformanceCurveJsonString() {

    std::vector<double> attack_angle_rad, cd, cl, cn;

    ParseRudderJsonString(m_params.m_perf_data_json_string, attack_angle_rad, cd, cl, cn);

    m_cl_cd_cn_coeffs.SetX(attack_angle_rad);
    m_cl_cd_cn_coeffs.AddY("cd", cd);
    m_cl_cd_cn_coeffs.AddY("cl", cl);
    m_cl_cd_cn_coeffs.AddY("cn", cn);
//    m_cl_cd_cn_coeffs.PermissiveOFF();
    m_min_alpha_R_rad = *std::min_element(attack_angle_rad.begin(), attack_angle_rad.end());
    m_max_alpha_R_rad = *std::max_element(attack_angle_rad.begin(), attack_angle_rad.end());

  }

  void ParseRudderJsonString(const std::string &json_string,
                             std::vector<double> &attack_angle_rad,
                             std::vector<double> &cd,
                             std::vector<double> &cl,
                             std::vector<double> &cn) {

    json node = json::parse(json_string);
    unsigned int n;

    try {
      attack_angle_rad = node["angle_of_attack_deg"].get<std::vector<double>>();
      n = attack_angle_rad.size();
      if (!std::is_sorted(attack_angle_rad.begin(), attack_angle_rad.end()))
        throw std::runtime_error("SimpleRudderModel parser : angle_of_attack_deg not sorted");
      for (auto &angle:attack_angle_rad) {
        angle *= DEG2RAD;
      }
    } catch (json::parse_error &err) {
      std::cerr << "SimpleRudderModel parser : no angle_of_attack_deg in load_coefficients";
      exit(EXIT_FAILURE);
    } catch (const std::exception &d) {
      std::cerr << d.what();
      exit(EXIT_FAILURE);
    }

    try {
      cd = node["cd"].get<std::vector<double>>();
      if (cd.size() != n)
        throw std::runtime_error(
            "SimpleRudderModel parser : cd not covering all angle_of_attack_deg range");
    } catch (json::parse_error &err) {
      std::cerr << "SimpleRudderModel parser : no cd in load_coefficients";
      exit(EXIT_FAILURE);
    } catch (const std::exception &d) {
      std::cerr << d.what();
      exit(EXIT_FAILURE);
    }

    try {
      cl = node["cl"].get<std::vector<double>>();
      if (cl.size() != n)
        throw std::runtime_error(
            "SimpleRudderModel parser : cl not covering all angle_of_attack_deg range");
    } catch (json::parse_error &err) {
      std::cerr << "SimpleRudderModel parser : no cl in load_coefficients";
      exit(EXIT_FAILURE);
    } catch (const std::exception &d) {
      std::cerr << d.what();
      exit(EXIT_FAILURE);
    }

    try {
      cn = node["cn"].get<std::vector<double>>();
      if (cl.size() != n)
        throw std::runtime_error(
            "SimpleRudderModel parser : cn not covering all angle_of_attack_deg range");
    } catch (json::parse_error &err) {
      std::cerr << "SimpleRudderModel parser : no cn in load_coefficients. Set zeros";
      cn = std::vector<double>(cd.size(), 0.0); // Fill cn with 0, not loaded from json ...
    } catch (const std::exception &d) {
      std::cerr << d.what();
      exit(EXIT_FAILURE);
    }


  }

}  // end namespace acme