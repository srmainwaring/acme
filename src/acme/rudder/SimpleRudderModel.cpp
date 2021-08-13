//
// Created by frongere on 09/08/2021.
//

#include "SimpleRudderModel.h"

#include <iostream>
#include <vector>
#include <nlohmann/json.hpp>
#include <MathUtils/Vector3d.h>
#include <MathUtils/Angles.h>
#include "MathUtils/Unit.h"

using json = nlohmann::json;

namespace acme {

  SimpleRudderModel::SimpleRudderModel(const RudderParams params, const std::string &perf_data_json_string) :
      m_params(params),
      m_temp_perf_data_json_string(perf_data_json_string),
      m_is_initialized(false),
      m_type(RudderModelType::E_SIMPLE_RUDDER) {

  }

  void SimpleRudderModel::Initialize() {
    ParseRudderPerformanceCurveJsonString();
    m_temp_perf_data_json_string.clear();
    // TODO: initialiser ici le cache des quantites optionnelles


    m_is_initialized = true;
  }

  void SimpleRudderModel::Compute(const double &water_density,
                                  const double &u_NWU,
                                  const double &v_NWU,
                                  const double &rudder_angle_deg) const {

    if (!m_is_initialized) {
      std::cerr << "Propulsion model MUST be initialized before being used." << std::endl;
      exit(EXIT_FAILURE);
    }

    double sidewash_angle_0 = std::atan2(v_NWU, u_NWU); // ou drift_angle_0 pour calculer le wake fraction

    // Estimated wake fraction
    double wr = m_params.m_hull_wake_fraction_0 * std::exp(-4. * sidewash_angle_0 * sidewash_angle_0);

    double uRA = u_NWU * (1 - wr);
    double vRA = v_NWU;

//    if (m_params.m_has_hull_influence_transverse_velocity) {
//
//      double kappa = 1.; // TODO: implementer le calcul
//      vRA *= kappa;
//    }

    // Drift angle
    double beta_R = std::atan2(vRA, uRA);

    // Attack angle
    double rudder_angle_rad = rudder_angle_deg * MU_PI_180;
    c_alpha_R_rad = rudder_angle_rad - beta_R;
    c_alpha_R_rad = mathutils::Normalize__PI_PI(c_alpha_R_rad);

    // Get coefficients
    double cl, cd, cn;
    GetClCdCn(c_alpha_R_rad, rudder_angle_rad, cl, cd, cn);

    // Forces in flow frame
    double q = 0.5 * water_density * (uRA * uRA + vRA * vRA); // stagnation pressure at rudder position
    c_lift_N = q * cl *
               m_params.m_lateral_area_m2; // FIXME: la prise en compte du signe de alpha se fait comment ? dans les tables ?
    c_drag_N = q * cd * m_params.m_lateral_area_m2; // FIXME: les tables prevoient des coeffs cd negatifs ?
    c_torque_Nm = q * cn * m_params.m_lateral_area_m2 *
                  m_params.m_chord_m; // FIXME: la prise en compte du signe de alpha se fait comment ? dans les tables ?

    // Forces in body frame
    double Cbeta = std::cos(beta_R);
    double Sbeta = std::sin(beta_R);

    c_fx_N = Cbeta * c_drag_N - Sbeta * c_lift_N;
    c_fy_N = Sbeta * c_drag_N + Cbeta * c_lift_N;

  }

  RudderModelType SimpleRudderModel::GetRudderModelType() const {
    return m_type;
  }

  const RudderParams &SimpleRudderModel::GetParameters() const {
    return m_params;
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
  } catch (std::exception& e) {
    std::cerr << "SimpleRudder : attack angle exceed interpolator range : " << e.what() << std::endl;
    exit(EXIT_FAILURE);
  }
  }


  void SimpleRudderModel::ParseRudderPerformanceCurveJsonString() {

    std::vector<double> attack_angle_rad, cd, cl, cn;
    ParseRudderJsonString(m_temp_perf_data_json_string, attack_angle_rad, cd, cl, cn);
    m_cl_cd_cn_coeffs.SetX(attack_angle_rad);
    m_cl_cd_cn_coeffs.AddY("cd", cd);
    m_cl_cd_cn_coeffs.AddY("cl", cl);
    m_cl_cd_cn_coeffs.AddY("cn", cn);

    m_min_alpha_R_rad = *std::min_element(attack_angle_rad.begin(), attack_angle_rad.end());
    m_max_alpha_R_rad = *std::max_element(attack_angle_rad.begin(), attack_angle_rad.end());

  }

  void ParseRudderJsonString(const std::string &json_string,
                             std::vector<double> &attack_angle_rad,
                             std::vector<double> &cd,
                             std::vector<double> &cl, std::vector<double> &cn) {

    std::string fc, dc;

    json node = json::parse(json_string);
    unsigned int n;

//    auto node = j["rudder"];

    try {
      fc = node["frame_convention"].get<json::string_t>();
      if (fc != "NWU" and fc != "NED")
        throw std::runtime_error("SimpleRudderModel parser : frame convention should be : NWU or NED");
    } catch (json::parse_error &err) {
      std::cerr << "SimpleRudderModel parser : no frame_convention in load_coefficients";
      exit(EXIT_FAILURE);
    } catch (const std::exception &d) {
      std::cerr << d.what();
      exit(EXIT_FAILURE);
    }

    try {
      dc = node["direction_convention"].get<json::string_t>();
      if (dc != "GOTO" and dc != "COMEFROM")
        throw std::runtime_error("SimpleRudderModel parser : frame convention should be : GOTO or COMEFROM");
    } catch (json::parse_error &err) {
      std::cerr << "SimpleRudderModel parser : no direction_convention in load_coefficients";
      exit(EXIT_FAILURE);
    } catch (const std::exception &d) {
      std::cerr << d.what();
      exit(EXIT_FAILURE);
    }

    try {
      attack_angle_rad = node["flow_incidence_on_main_rudder_deg"].get<std::vector<double>>();
      n = attack_angle_rad.size();
      if (!std::is_sorted(attack_angle_rad.begin(), attack_angle_rad.end()))
        throw std::runtime_error("SimpleRudderModel parser : flow_incidence_on_main_rudder_deg not sorted");
      for (auto &angle:attack_angle_rad) {
        angle *= DEG2RAD;
      }
      if (dc == "GOTO")
        for (auto &angle:attack_angle_rad) {
          angle =+ MU_PI;
          angle = mathutils::Normalize__PI_PI(angle);
        }
    } catch (json::parse_error &err) {
      std::cerr << "SimpleRudderModel parser : no flow_incidence_on_main_rudder_deg in load_coefficients";
      exit(EXIT_FAILURE);
    } catch (const std::exception&d) {
      std::cerr<<d.what();
      exit(EXIT_FAILURE);
    }

    try {
      cd = node["Cd"].get<std::vector<double>>();
      if (cd.size()!=n)
        throw std::runtime_error("SimpleRudderModel parser : Cd not covering all flow_incidence_on_main_rudder_deg range");
    } catch (json::parse_error &err) {
      std::cerr << "SimpleRudderModel parser : no Cd in load_coefficients";
      exit(EXIT_FAILURE);
    } catch (const std::exception&d) {
      std::cerr<<d.what();
      exit(EXIT_FAILURE);
    }

    try {
      cl = node["Cl"].get<std::vector<double>>();
      if (cl.size()!=n)
        throw std::runtime_error("SimpleRudderModel parser : Cd not covering all flow_incidence_on_main_rudder_deg range");
    } catch (json::parse_error &err) {
      std::cerr << "SimpleRudderModel parser : no Cl in load_coefficients";
      exit(EXIT_FAILURE);
    } catch (const std::exception&d) {
      std::cerr<<d.what();
      exit(EXIT_FAILURE);
    }

    try {
      cn = node["Cn"].get<std::vector<double>>();
      if (cn.size()!=n)
        throw std::runtime_error("SimpleRudderModel parser : Cd not covering all flow_incidence_on_main_rudder_deg range");
      if (fc == "NED")
        for (auto& coeff : cn) {
          coeff *= -1;
        }
    } catch (json::parse_error &err) {
      std::cerr << "SimpleRudderModel parser : no Cn in load_coefficients";
      exit(EXIT_FAILURE);
    } catch (const std::exception&d) {
      std::cerr<<d.what();
      exit(EXIT_FAILURE);
    }




  }

}  // end namespace acme
