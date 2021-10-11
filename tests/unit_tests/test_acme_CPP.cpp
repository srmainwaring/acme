// ==========================================================================
// FRyDoM - frydom-ce.org
//
// Copyright (c) Ecole Centrale de Nantes (LHEEA lab.) and D-ICE Engineering.
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDoM.
//
// ==========================================================================

#include "acme/acme.h"
#include "gtest/gtest.h"

using namespace acme;

template<typename T>
std::string str(T begin, T end) {
  std::stringstream ss;
  bool first = true;
  ss << "[";
  for (; begin != end; begin++) {
    if (!first)
      ss << ", ";
    ss << *begin;
    first = false;
  }
  ss << "]";
  return ss.str();
}

TEST(TestCPP, parser) {

  mathutils::LookupTable2d<double> ct_cq_coeffs;

  std::vector<double> beta_in = {-180.0, -140.0, -100.0, -60.0, -20.0, 20.0, 60.0, 100.0, 140.0, 180.0};
  std::vector<double> pitch_ratio_in = {-1.0, -0.5, 0.0, 0.5, 1.0};
  std::vector<std::vector<double>> ct_in = {{0.09802344228991167,   0.4988215184225952,  0.8231778824097602, 0.6578735041816796, -0.030677575975738707, -0.18100516740735045, -0.48816928539246107, -0.5422513249948423, -0.241955827686086,  0.09760901400000001},
                                            {0.02180769973655439,   0.6186028062460024,  1.1077294146069343, 0.9309423820559046, 0.11061832508836672,   -0.27180694740557126, -0.6533257797843176,  -0.7060443450793658, -0.2945433086645531, 0.022564886255851284},
                                            {-0.000816229983420568, 0.5549690270137281,  1.1296393536152063, 1.0118939720603464, 0.2773688034374328,    -0.3159968292076525,  -1.0262724516816766,  -1.1171690884802659, -0.5106919252572178, 0.0008162299834205433},
                                            {-0.02820610781981364,  0.4006411665291089,  0.8950633072029262, 0.8124318138730758, 0.3060672901735377,    -0.1790999500696448,  -1.2017792679629602,  -1.3874039779622673, -0.7185487731319289, -0.0272596246706925},
                                            {-0.1626816899999993,   0.44866883773853117, 0.8944023203183769, 0.8032479305281881, 0.275724013778679,     0.017939934846272,    -1.1427245405281872,  -1.3839567584729409, -0.7788844482089198, -0.16337240381651863}};
  std::vector<std::vector<double>> cq_in = {{-0.0114106274032369,   -0.05096469487178818,  -0.07146106086730822, -0.06198631583205713, 0.010279686325922563,  0.03675019835103964,   0.07774793189095229,  0.0664279441760814,   0.026284701001934016,  -0.011165988600000013},
                                            {-0.003522504562149637, -0.037319865766268494, -0.05041659824975429, -0.03801876017127842, -0.002641774188784713, 0.018157851375276573,  0.05486183456224583,  0.0612421985324825,   0.02205770495245041,   -0.002775729928764877},
                                            {0.0022641186088044707, -0.004736600934521865, 0.012580274323343773, 0.008846389031002174, 0.0009727528373047617, 0.0010769429488141113, 0.009630382758339157, 0.010144557233001053, -0.006136809490287035, 0.0022641186088044776},
                                            {-0.00346966241095603,  0.03117658791007869,   0.07708674165515654,  0.0675686590540254,   0.02166644052249731,   -0.004430378368613751, -0.04909244207457484, -0.06520860571480078, -0.04369741490843816,  -0.004403130702686983},
                                            {-0.018609980999999907, 0.05512484306005881,   0.10925670286110455,  0.12550483840789847,  0.055566309226283736,  0.015411367752487103,  -0.10401960640789848, -0.12289300851577122, -0.08100901238416293,  -0.019017712338728045}
  };

  std::stringstream ss;
  ss << R"({"beta_deg": )" << str(beta_in.begin(), beta_in.end())
     << R"(, "p_d": )" << str(pitch_ratio_in.begin(), pitch_ratio_in.end())
     << R"(, "ct": [)";
  bool first = true;
  for (auto &coeff : ct_in) {
    if (!first) ss << ",\n";
    ss << str(coeff.begin(), coeff.end());
    first = false;
  }
  ss << R"(], "cq": [)";
  first = true;
  for (auto &coeff : cq_in) {
    if (!first) ss << ",\n";
    ss << str(coeff.begin(), coeff.end());
    first = false;
  }
  ss << "]}";
//  std::cout<<ss.str()<<std::endl;

//  std::string open_water_data_table = R"({"beta_deg": [-180.0,-140.0,-100.0,-60.00000000000001,-20.000000000000004,20.000000000000004,59.999999999999986,99.99999999999999,140.0,180.0],
//      "p_d": [ -1.0, -0.5, 0.0, 0.5, 1.0 ],
//      "screw_direction": "RIGHT_HANDED",
//      "ct": [
//        [0.09802344228991167, 0.4988215184225952, 0.8231778824097602, 0.6578735041816796, -0.030677575975738707, -0.18100516740735045, -0.48816928539246107, -0.5422513249948423, -0.241955827686086, 0.09760901400000001],
//        [ 0.02180769973655439, 0.6186028062460024, 1.1077294146069343, 0.9309423820559046, 0.11061832508836672, -0.27180694740557126, -0.6533257797843176, -0.7060443450793658, -0.2945433086645531, 0.022564886255851284],
//        [ -0.000816229983420568, 0.5549690270137281, 1.1296393536152063, 1.0118939720603464, 0.2773688034374328, -0.3159968292076525, -1.0262724516816766, -1.1171690884802659, -0.5106919252572178, 0.0008162299834205433],
//        [ -0.02820610781981364, 0.4006411665291089, 0.8950633072029262, 0.8124318138730758, 0.3060672901735377, -0.1790999500696448, -1.2017792679629602, -1.3874039779622673, -0.7185487731319289, -0.0272596246706925],
//        [ -0.1626816899999993, 0.44866883773853117, 0.8944023203183769, 0.8032479305281881, 0.275724013778679, 0.017939934846272, -1.1427245405281872, -1.3839567584729409, -0.7788844482089198, -0.16337240381651863]
//      ],
//      "cq": [
//        [ -0.0114106274032369, -0.05096469487178818, -0.07146106086730822, -0.06198631583205713, 0.010279686325922563, 0.03675019835103964, 0.07774793189095229, 0.0664279441760814, 0.026284701001934016, -0.011165988600000013],
//        [ -0.003522504562149637, -0.037319865766268494, -0.05041659824975429, -0.03801876017127842, -0.002641774188784713, 0.018157851375276573, 0.05486183456224583, 0.0612421985324825, 0.02205770495245041, -0.002775729928764877],
//        [ 0.0022641186088044707, -0.004736600934521865, 0.012580274323343773, 0.008846389031002174, 0.0009727528373047617, 0.0010769429488141113, 0.009630382758339157, 0.010144557233001053, -0.006136809490287035, 0.0022641186088044776],
//        [ -0.00346966241095603, 0.03117658791007869, 0.07708674165515654, 0.0675686590540254, 0.02166644052249731, -0.004430378368613751, -0.04909244207457484, -0.06520860571480078, -0.04369741490843816, -0.004403130702686983],
//        [ -0.018609980999999907, 0.05512484306005881, 0.10925670286110455, 0.12550483840789847, 0.055566309226283736, 0.015411367752487103, -0.10401960640789848, -0.12289300851577122, -0.08100901238416293, -0.019017712338728045]
//      ]
//    })";

  std::vector<double> beta, pitch_ratio, ct, cq;
  ParseCPPJsonString(ss.str(), beta, pitch_ratio, ct, cq);
  ct_cq_coeffs.SetX(beta);
  ct_cq_coeffs.SetY(pitch_ratio);
  ct_cq_coeffs.AddData("ct", ct);
  ct_cq_coeffs.AddData("cq", cq);

  EXPECT_EQ(pitch_ratio, pitch_ratio_in);

  for (int i = 0; i < beta_in.size(); i++) {
    EXPECT_NEAR(beta[i] * MU_180_PI, beta_in[i], 1E-6);
    for (int j = 0; j < pitch_ratio_in.size(); j++) {
//      std::cout << "(i,j) = (" << i << ',' << j << ')' << std::endl;
      EXPECT_NEAR(ct_in[j][i], ct_cq_coeffs.Eval("ct", beta[i], pitch_ratio[j]), 1E-5);
      EXPECT_NEAR(cq_in[j][i], ct_cq_coeffs.Eval("cq", beta[i], pitch_ratio[j]), 1E-5);
    }
  }


}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}