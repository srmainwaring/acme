//
// Created by lletourn on 05/11/2021.
//


#include "gtest/gtest.h"
#include "acme/acme.h"

using namespace acme;

class TestBrixRudder : public BrixRudderModel {

 public:

  TestBrixRudder(const RudderParams &params, double d, double Cf=0);

  void GetClCdCn(const double &attack_angle_rad,
                 const double &rudder_angle_rad,
                 double &cl,
                 double &cd,
                 double &cn) const override;

};

TestBrixRudder::TestBrixRudder(const RudderParams &params, double d, double Cf) : BrixRudderModel(params, d, Cf) {

}

void TestBrixRudder::GetClCdCn(const double &attack_angle_rad, const double &rudder_angle_rad, double &cl, double &cd,
                               double &cn) const {
  BrixRudderModel::GetClCdCn(attack_angle_rad, rudder_angle_rad, cl, cd, cn);
}

void test_coefficients(double alpha, TestBrixRudder& rudder, double d, double Cf){

  auto Ar = rudder.GetParameters().m_lateral_area_m2;
  auto c = rudder.GetParameters().m_chord_m;
  auto aspect_ratio = Ar /(c*c);

  auto ca = cos(alpha);
  auto sa = sin(alpha);

  // Lift coefficient
  auto Cl1 = 2 * MU_PI * aspect_ratio * (aspect_ratio + 1) / pow(aspect_ratio+2, 2) * sa;
  auto Cl2 = sa * abs(sa) * ca;
  auto cl = Cl1 + Cl2;

  // Drag coefficient
  auto Cd1 = 1.1 * cl * cl / (MU_PI * aspect_ratio);
  auto Cd2 = pow(abs(sa),3) + 2.5 * Cf;
  auto cd = Cd1 + Cd2;

  // Torque coefficient at rudder nose
  auto Cqn = -(Cl1*ca + Cd1*sa) * (0.47 - (aspect_ratio+2)/(4*(aspect_ratio+1)))
             - 0.75 *(Cl2*ca + Cd2*sa);

  // Torque coefficient at rudder stock
  auto cn = Cqn + d/c * (cl * ca + cd * sa);

  double cd_, cl_, cn_;
  rudder.GetClCdCn(alpha, 0., cl_, cd_, cn_);

  EXPECT_NEAR(cd, cd_, 1E-6);
  EXPECT_NEAR(cl, cl_, 1E-6);
  EXPECT_NEAR(cn, cn_, 1E-6);
}

TEST(BrixRudder, coefficients) {

  auto params = RudderParams();
  params.m_lateral_area_m2 = 4;
  params.m_chord_m = 1;
  auto d = 0.5; // distance nose to rudder stock

  double U_ms = 5;
  // Reynolds number
  auto Re = U_ms * params.m_chord_m / 1.15E-6;
  // Frictional coefficient from ITTC57
  auto Cf = 0.075 / pow(log10(Re - 2), 2);

  auto rudder = TestBrixRudder(params, d, Cf);
//  rudder.SetITTC57FrictionalResistanceCoefficient(U_ms, mathutils::MS);
  rudder.Initialize();

  test_coefficients(0., rudder, d, Cf);
  test_coefficients(10*DEG2RAD, rudder, d, Cf);
  test_coefficients(-10*DEG2RAD, rudder, d, Cf);

}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
