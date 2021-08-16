//
// Created by frongere on 09/08/2021.
//

#include <nlohmann/json.hpp>
#include <MathUtils/Constants.h>
#include "FlapRudderModel.h"

using json = nlohmann::json;

namespace acme {


  FlapRudderModel::FlapRudderModel(const RudderParams params, const std::string &perf_data_json_string)
      : SimpleRudderModel(params, perf_data_json_string) {
    m_type = RudderModelType::E_FLAP_RUDDER;  // Overrides the E_SIMPLE_RUDDER
  }

  void FlapRudderModel::GetClCdCn(const double &attack_angle_rad,
                                  const double &rudder_angle_rad,
                                  double &cl,
                                  double &cd,
                                  double &cn) const {

    // Getting the flap angle from the rudder angle using the linear law (only linear law currently supported)
    double flap_angle_rad = m_params.m_flap_slope * rudder_angle_rad;

    cl = m_cl_cd_cn_coeffs.Eval("cl", attack_angle_rad, flap_angle_rad);
    cd = m_cl_cd_cn_coeffs.Eval("cd", attack_angle_rad, flap_angle_rad);
    cn = m_cl_cd_cn_coeffs.Eval("cn", attack_angle_rad, flap_angle_rad);

  }

  void FlapRudderModel::ParseRudderPerformanceCurveJsonString() {
    std::string json_string;
    std::vector<double> attack_angle_rad, flap_angle_rad, cd, cl, cn;
    ParseFlapRudderJsonString(json_string, attack_angle_rad, flap_angle_rad, cd, cl, cn);
    m_cl_cd_cn_coeffs.SetX(attack_angle_rad);
    m_cl_cd_cn_coeffs.SetY(flap_angle_rad);
    m_cl_cd_cn_coeffs.AddData("cd", cd);
    m_cl_cd_cn_coeffs.AddData("cl", cl);
    m_cl_cd_cn_coeffs.AddData("cn", cn);

    m_min_alpha_R_rad = *std::min_element(attack_angle_rad.begin(), attack_angle_rad.end());
    m_max_alpha_R_rad = *std::max_element(attack_angle_rad.begin(), attack_angle_rad.end());

    m_min_flap_angle_rad = *std::min_element(flap_angle_rad.begin(), flap_angle_rad.end());
    m_max_flap_angle_rad = *std::max_element(flap_angle_rad.begin(), flap_angle_rad.end());
  }


  void ParseFlapRudderJsonString(const std::string &json_string, std::vector<double> &attack_angle_rad,
                                 std::vector<double> &flap_angle_rad, std::vector<double> &cd, std::vector<double> &cl,
                                 std::vector<double> &cn) {

    std::vector<std::vector<double>> Cd, Cl, Cn;
    int n;

    json j = json::parse(json_string);

    try {
      flap_angle_rad = j["flap_angle_deg"].get<std::vector<double>>();
      for (auto &angle:flap_angle_rad) angle *= DEG2RAD;
      n = flap_angle_rad.size();
    } catch (json::parse_error &err) {
      std::cerr << "SimpleRudderModel parser : no flap_angle_deg in load_coefficients";
      exit(EXIT_FAILURE);
    }

    try {
      attack_angle_rad = j["flow_incidence_on_main_rudder_deg"].get<std::vector<double>>();
      for (auto &angle:attack_angle_rad) angle *= DEG2RAD;
      n *= attack_angle_rad.size();
      if (!std::is_sorted(attack_angle_rad.begin(), attack_angle_rad.end()))
        throw std::runtime_error("SimpleRudderModel parser : flow_incidence_on_main_rudder_deg not sorted");
    } catch (json::parse_error &err) {
      std::cerr << "SimpleRudderModel parser : no flow_incidence_on_main_rudder_deg in load_coefficients";
      exit(EXIT_FAILURE);
    } catch (const std::exception&d) {
    std::cerr<<d.what();
    exit(EXIT_FAILURE);
  }

    try {
      Cd = j["Cd"].get<std::vector<std::vector<double>>>();
      Eigen::MatrixXd mat(flap_angle_rad.size(), attack_angle_rad.size());
      for (int i = 0; i < flap_angle_rad.size(); i++)
        mat.row(i) = Eigen::VectorXd::Map(&Cd[i][0], Cd[i].size());
      cd = {mat.data(), mat.data() + mat.rows() * mat.cols()};
      if (cd.size()!=n)
        throw std::runtime_error("SimpleRudderModel parser : Cd not covering all angles range");
    } catch (json::parse_error &err) {
      std::cerr << "SimpleRudderModel parser : no Cd in load_coefficients";
      exit(EXIT_FAILURE);
    } catch (const std::exception&d) {
      std::cerr<<d.what();
      exit(EXIT_FAILURE);
    }

    try {
      Cl = j["Cl"].get<std::vector<std::vector<double>>>();
      Eigen::MatrixXd mat(flap_angle_rad.size(), attack_angle_rad.size());
      for (int i = 0; i < flap_angle_rad.size(); i++)
        mat.row(i) = Eigen::VectorXd::Map(&Cl[i][0], Cl[i].size());
      cl = {mat.data(), mat.data() + mat.rows() * mat.cols()};
      if (cl.size()!=n)
        throw std::runtime_error("SimpleRudderModel parser : Cl not covering all angles range");
    } catch (json::parse_error &err) {
      std::cerr << "SimpleRudderModel parser : no Cl in load_coefficients";
      exit(EXIT_FAILURE);
    } catch (const std::exception&d) {
      std::cerr<<d.what();
      exit(EXIT_FAILURE);
    }

    try {
      Cn = j["Cn"].get<std::vector<std::vector<double>>>();
      Eigen::MatrixXd mat(flap_angle_rad.size(), attack_angle_rad.size());
      for (int i = 0; i < flap_angle_rad.size(); i++)
        mat.row(i) = Eigen::VectorXd::Map(&Cn[i][0], Cn[i].size());
      cn = {mat.data(), mat.data() + mat.rows() * mat.cols()};
      if (cn.size()!=n)
        throw std::runtime_error("SimpleRudderModel parser : Cn not covering all angles range");
    } catch (json::parse_error &err) {
      std::cerr << "SimpleRudderModel parser : no Cn in load_coefficients";
      exit(EXIT_FAILURE);
    } catch (const std::exception&d) {
      std::cerr<<d.what();
      exit(EXIT_FAILURE);
    }

    //TODO : gerer les changements de conventions NWU/NED et GOT/COMEFROM,
    //             les conventions d'angles ([pi,pi] ou [0, 2pi]
    //             les symetries, etc.


  }
}  // end namespace acme