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


class TestFPP1Q : public testing::Test {

 public:
  TestFPP1Q() : propeller(params, open_water_data_table) {}

 protected:

  /// Initialization of the environment
  void SetUp() override;

 public:

//  const std::string open_water_data_table = R"({"j":[0,0.5,1], "kt":[1,0.5,0], "kq":[0.5,1,0.5]})";
  const std::string open_water_data_table = R"({"j": [0, 0.01010101, 0.02020202, 0.03030303, 0.04040404, 0.05050505, 0.06060606, 0.07070707, 0.08080808, 0.09090909, 0.1010101, 0.11111111, 0.12121212, 0.13131313, 0.14141414, 0.15151515, 0.16161616, 0.17171717, 0.18181818, 0.19191919, 0.2020202, 0.21212121, 0.22222222, 0.23232323, 0.24242424, 0.25252525, 0.26262626, 0.27272727, 0.28282828, 0.29292929, 0.3030303, 0.31313131, 0.32323232, 0.33333333, 0.34343434, 0.35353535, 0.36363636, 0.37373737, 0.38383838, 0.39393939, 0.4040404, 0.41414141, 0.42424242, 0.43434343, 0.44444444, 0.45454545, 0.46464646, 0.47474747, 0.48484848, 0.49494949, 0.50505051, 0.51515152, 0.52525253, 0.53535354, 0.54545455, 0.55555556, 0.56565657, 0.57575758, 0.58585859, 0.5959596, 0.60606061, 0.61616162, 0.62626263, 0.63636364, 0.64646465, 0.65656566, 0.66666667, 0.67676768, 0.68686869, 0.6969697, 0.70707071, 0.71717172, 0.72727273, 0.73737374, 0.74747475, 0.75757576, 0.76767677, 0.77777778, 0.78787879, 0.7979798, 0.80808081, 0.81818182, 0.82828283, 0.83838384, 0.84848485],
      "kt": [0.35422196, 0.35142214, 0.34857537, 0.34568216, 0.34274302, 0.33975846, 0.33672898, 0.3336551, 0.33053732, 0.32737615, 0.32417211, 0.3209257, 0.31763743, 0.31430781, 0.31093735, 0.30752656, 0.30407595, 0.30058602, 0.29705729, 0.29349026, 0.28988544, 0.28624335, 0.28256449, 0.27884937, 0.2750985,  0.27131239, 0.26749155, 0.26363649, 0.25974771, 0.25582573, 0.25187106, 0.24788419, 0.24386565, 0.23981594, 0.23573558, 0.23162506, 0.2274849,  0.22331562, 0.2191177,  0.21489168, 0.21063805, 0.20635733, 0.20205002, 0.19771663, 0.19335768, 0.18897366, 0.1845651,  0.1801325, 0.17567636, 0.17119721, 0.16669554, 0.16217186, 0.15762669, 0.15306054, 0.14847391, 0.14386731, 0.13924125, 0.13459624, 0.12993279, 0.12525141, 0.12055261, 0.1158369,  0.11110478, 0.10635677, 0.10159337, 0.09681509, 0.09202245, 0.08721595, 0.08239609, 0.0775634,  0.07271838, 0.06786154, 0.06299338, 0.05811442, 0.05322516, 0.04832612, 0.0434178,  0.03850072, 0.03357537, 0.02864228, 0.02370195, 0.01875488, 0.0138016,  0.0088426, 0.0038784],
      "kq": [0.04336206,  0.04307767,  0.04278789,  0.04249279,  0.04219242,  0.04188681, 0.04157602,  0.04126011,  0.04093911,  0.04061307,  0.04028205,  0.03994609, 0.03960525,  0.03925957,  0.03890909,  0.03855388,  0.03819397,  0.03782941, 0.03746027,  0.03708657,  0.03670838,  0.03632574, 0.0359387,   0.0355473, 0.03515161,  0.03475165, 0.0343475,   0.03393919,  0.03352677,  0.03311029, 0.0326898, 0.03226534,  0.03183698, 0.03140475,  0.03096871,  0.0305289, 0.03008537,  0.02963817, 0.02918735,  0.02873296,  0.02827505,  0.02781366, 0.02734885,  0.02688066,  0.02640914,  0.02593434,  0.02545632,  0.02497511, 0.02449077,  0.02400334,  0.02351288,  0.02301943,  0.02252305,  0.02202377, 0.02152166,  0.02101675,  0.0205091,   0.01999876, 0.01948577,  0.01897018, 0.01845205, 0.01793141,  0.01740833, 0.01688284,  0.016355,    0.01582486, 0.01529246,  0.01475786,  0.0142211,   0.01368222,  0.01314129,  0.01259835, 0.01205344,  0.01150662,  0.01095793,  0.01040742, 0.00985515,  0.00930116, 0.0087455,  0.00818822,  0.00762936,  0.00706898,  0.00650712,  0.00594384, 0.00537918]
    })";

  PropellerParams params = {2., 0.25, 0.2, SCREW_DIRECTION::RIGHT_HANDED};
  FPP1Q propeller;

};

void TestFPP1Q::SetUp() {
  propeller.Initialize();
}

TEST_F(TestFPP1Q, forces) {

  // J = NaN
  propeller.Compute(1025, 0., 0., 0., 0.);
  EXPECT_EQ(propeller.GetThrust(), 0.);
  EXPECT_EQ(propeller.GetTorque(), 0.);
  EXPECT_EQ(propeller.GetPower(), 0.);
  EXPECT_NEAR(propeller.GetPropellerEfficiency(), 0., 1E-5);

  // J = +inf
  propeller.Compute(1025, 1., 0., 0., 0.);
  EXPECT_EQ(propeller.GetThrust(), 0.);
  EXPECT_EQ(propeller.GetTorque(), 0.);
  EXPECT_EQ(propeller.GetPower(), 0.);
  EXPECT_NEAR(propeller.GetPropellerEfficiency(), 0., 1E-5);

  // J = 0
  propeller.Compute(1025, 0., 0., 60., 0.);
  EXPECT_NEAR(propeller.GetThrust(), 4647.39, 1E-2);
  EXPECT_NEAR(propeller.GetTorque(), 1422.28, 1E-2);
  EXPECT_NEAR(propeller.GetPower(), 8936.42, 1E-2);
  EXPECT_EQ(propeller.GetPropellerEfficiency(), 0.);

  // J = 0.5
  propeller.Compute(1025, 1., 0., 60, 0.);
  EXPECT_NEAR(propeller.GetThrust(), 2923.02, 1E-2);
  EXPECT_NEAR(propeller.GetTorque(), 970.284, 1E-2);
  EXPECT_NEAR(propeller.GetPower(), 6096.47, 1E-2);
  EXPECT_NEAR(propeller.GetPropellerEfficiency(), 0.449494, 1E-6);

  // Lateral flow velocity
  propeller.Compute(1025, 0, 1, 60, 0.);
  EXPECT_NEAR(propeller.GetThrust(), 4647.39, 1E-2);
  EXPECT_NEAR(propeller.GetTorque(), 1422.28, 1E-2);
  EXPECT_NEAR(propeller.GetPower(), 8936.42, 1E-2);
  EXPECT_EQ(propeller.GetPropellerEfficiency(), 0.);

  // 20 degree side wash angle
  propeller.Compute(1025, std::cos(20*DEG2RAD), std::sin(20*DEG2RAD), 60, 0.);
  EXPECT_NEAR(propeller.GetThrust(), 2798.61, 1E-2);
  EXPECT_NEAR(propeller.GetTorque(), 936.852, 1E-3);
  EXPECT_NEAR(propeller.GetPower(), 5886.42, 1E-2);
  EXPECT_NEAR(propeller.GetPropellerEfficiency(), 0.4727, 1E-4);

  // Double the rpm
  propeller.Compute(1025, std::cos(20*DEG2RAD), std::sin(20*DEG2RAD), 120, 0.);
  EXPECT_NEAR(propeller.GetThrust(), 15272.6, 1E-1);
  EXPECT_NEAR(propeller.GetTorque(), 4831.72, 1E-2);
  EXPECT_NEAR(propeller.GetPower(), 60717.1, 1E-1);
  EXPECT_NEAR(propeller.GetPropellerEfficiency(), 0.250089, 1E-6);
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}