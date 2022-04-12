//
// Created by lletourn on 08/11/2021.
//

#include <nlohmann/json.hpp>
#include "acme/acme.h"
#include "MathUtils/VectorGeneration.h"

using namespace acme;
using json = nlohmann::json;


void comp_Technip(){

  auto params = RudderParams();
  params.m_lateral_area_m2 = 15.061;
  params.m_chord_m = 3.3;
  params.m_height_m = 4.600;
  params.m_flap_slope = 0.8;
  params.m_distance_nose_stock_m = 1.231; // distance nose to rudder stock
  params.m_Cf = compute_ITTC57_frictional_resistance_coefficient(params.m_chord_m, 10.);

  auto brix_rudder = BrixRudderModel(params);
//  rudder.SetITTC57FrictionalResistanceCoefficient(U_ms, mathutils::MS);
  brix_rudder.Initialize();

  std::ifstream tmp_buffer("/home/lletourn/Documents/tools/minos/data/rdx022/flap_rudder.json");
  json node = json::parse(tmp_buffer);
  params.m_perf_data_json_string = node["rudder"]["load_coefficients"].dump();
  auto flap_rudder = FlapRudderModel(params);
  flap_rudder.Initialize();

  auto angles = mathutils::linspace(-20, 20, 41);

  std::fstream myfile;
  myfile.open("rudder_coeffs.csv", std::ios::out);
  myfile << "angle;cd_brix;cl_brix;cn_brix;cd_flap;cl_flap;cn_flap;" << std::endl;
  myfile << "deg;;;;;;;" << std::endl;
  for (auto &angle: angles) {
    double cd_brix,cl_brix,cn_brix,cd_flap,cl_flap,cn_flap;
    brix_rudder.GetClCdCn(angle * DEG2RAD, 0., cl_brix, cd_brix, cn_brix);
    flap_rudder.GetClCdCn(angle * DEG2RAD, 0., cl_flap, cd_flap, cn_flap);
    myfile << angle << ";" << cd_brix << ";" << cl_brix << ";" << cn_brix << ";"
           << cd_flap << ";" << cl_flap << ";" << cn_flap << ";" << std::endl;
  }
  myfile.close();

}

void comp_TARA(){

  auto params = RudderParams();
  params.m_lateral_area_m2 = 36;
  params.m_chord_m = 5.2;
//  params.m_height_m = 4.600;
//  params.m_flap_slope = 0.8;
  params.m_distance_nose_stock_m = 2.6; // distance nose to rudder stock
  params.m_Cf = compute_ITTC57_frictional_resistance_coefficient(params.m_chord_m, 10.);

  auto brix_rudder = BrixRudderModel(params);
//  rudder.SetITTC57FrictionalResistanceCoefficient(U_ms, mathutils::MS);
  brix_rudder.Initialize();

  std::ifstream tmp_buffer("/home/lletourn/Documents/tools/minos/data/rdx027/rudder.json");
  json node = json::parse(tmp_buffer);
  params.m_perf_data_json_string = node["rudder"]["load_coefficients"].dump();
  auto TARA_rudder = SimpleRudderModel(params);
  TARA_rudder.Initialize();

  auto angles = mathutils::linspace(-25, 25, 51);

  std::fstream myfile;
  myfile.open("rudder_coeffs_TARA.csv", std::ios::out);
  myfile << "angle;cd_brix;cl_brix;cn_brix;cd_flap;cl_flap;cn_flap;" << std::endl;
  myfile << "deg;;;;;;;" << std::endl;
  for (auto &angle: angles) {
    double cd_brix, cl_brix, cn_brix, cd_tara, cl_tara, cn_tara;
    brix_rudder.GetClCdCn(angle * DEG2RAD, 0., cl_brix, cd_brix, cn_brix);
    TARA_rudder.GetClCdCn(angle * DEG2RAD, 0., cl_tara, cd_tara, cn_tara);

    myfile << angle << ";" << cd_brix << ";" << cl_brix << ";" << cn_brix << ";"
           << -cd_tara << ";" << cl_tara << ";" << cn_tara << ";" << std::endl;
  }
  myfile.close();

}


void comp_Total(){

  auto params = RudderParams();
  params.m_lateral_area_m2 = 1.848;
  params.m_chord_m = 1.1;
  params.m_height_m = 1.68;
//  params.m_flap_slope = 0.8;
  params.m_distance_nose_stock_m = 0.5; // distance nose to rudder stock
  params.m_Cf = compute_ITTC57_frictional_resistance_coefficient(params.m_chord_m, 10.);

  auto brix_rudder = BrixRudderModel(params);
//  rudder.SetITTC57FrictionalResistanceCoefficient(U_ms, mathutils::MS);
  brix_rudder.Initialize();

  auto angles = mathutils::linspace(-25, 15, 41);

  std::fstream myfile;
  myfile.open("rudder_coeffs_Total.csv", std::ios::out);
  myfile << "angle;cd_brix;cl_brix;cn_brix;cd_flap;cl_flap;cn_flap;" << std::endl;
  myfile << "deg;;;;;;;" << std::endl;
  for (auto &angle: angles) {
    double cd_brix,cl_brix,cn_brix;
    brix_rudder.GetClCdCn(angle * DEG2RAD, 0., cl_brix, cd_brix, cn_brix);

    auto Cd = -3.755E-4 * (angle+5*0) * (angle+5*0) - 1.902E-2;
    auto Cl = 4.686E-2 * (angle+5*0);
    auto Cn = 2.257E-2 * (angle+5*0);

    myfile << angle << ";" << cd_brix << ";" << cl_brix << ";" << cn_brix << ";"
           << Cd << ";" << Cl << ";" << Cn << ";" << std::endl;
  }
  myfile.close();

}

int main() {

  comp_Technip();
  comp_Total();
  comp_TARA();

  return 0;
}
