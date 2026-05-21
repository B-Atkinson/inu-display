#include <gtest/gtest.h>
#include <cmath>

#include <cmath>
#include <iostream>

#include "MagneticDeclination.hpp"


TEST(AssociatedLegendreDerivativesTest, PDerivativeValuesMatchExpected) {
    static constexpr double expected[13][13] = {
        {1.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0.4, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {-0.26, 1.2, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {-0.44, -0.3, 6, 15, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {-0.113, -1.88, 0.9, 42, 105, 0, 0, 0, 0, 0, 0, 0, 0},
        {0.27064, -1.317, -10.92, 23.1, 378, 945, 0, 0, 0, 0, 0, 0, 0},
        {0.292636, 1.09704, -13.587, -78.12, 359.1, 4158, 10395, 0, 0, 0, 0, 0, 0},
        {-0.0145904, 2.487268, 3.34152, -153.531, -637.56, 5613.3, 54054, 135135, 0, 0, 0, 0, 0},
        {-0.2669993, 0.878184, 23.72202, -27.9972, -1943.865, -5405.4, 94594.5, 810810, 2027025, 0, 0, 0, 0},
        {-0.18876356, -2.0517201, 18.270648, 249.74334, -1113.5124, -27432.405, -37837.8, 1743241.5, 13783770, 34459425, 0, 0, 0},
        {0.0968390644, -2.70832364, -15.2606619, 319.145112, 2801.25846, -26562.1356, -426621.195, 91891.8, 35148613.5, 261891630, 654729075, 0, 0},
        {0.24555306736, -0.0180997476, -38.60414844, -70.7305599, 5588.534952, 31394.02266, -595642.6476, -7215803.595, 15713497.8, 772580308.5, 5499724230, 13749310575, 0},
        {0.099488209276, 2.93939690928, -15.6769560948, -568.75030212, 1174.4555823, 101974.168296, 295441.32618, -13607889.0948, -130814869.185, 623302079.4, 18424076170.5, 126493657290, 316234143225}
    };
    MagneticDeclination t = MagneticDeclination();

    t.CalculateMthLthOrderAssociatedLegrandreFunctionDerivatives(12, 12, 0.4);

    double* arr = t.GetMthLthOrderAssociatedLegrandreFunctionDerivatives();

    for (int i = 0; i < 13; i++) {
        for (int j = 0; j < 13; j++) {
            ASSERT_NEAR(arr[i * 13 + j], expected[i][j], 1e-6 * i) << "Index i: " + std::to_string(i) + " and j: " + std::to_string(j);
        }
    }
}

TEST(AssociatedLegendreDerivativesTest, PDerivativeValuesMatchWikipediaValues) {
    MagneticDeclination t = MagneticDeclination();

    t.CalculateMthLthOrderAssociatedLegrandreFunctionDerivatives(12, 12, 1);

    double* arr = t.GetMthLthOrderAssociatedLegrandreFunctionDerivatives();

    double x = 0.8632;

    ASSERT_NEAR(arr[0], 1.0, 0.00001);
    ASSERT_NEAR((std::pow(-1, 1) * std::pow(1 - x * x, 1.0 / 2.0)) * arr[13 * 1 + 1], -std::sqrt((1 - x * x)), 0.00001);
    ASSERT_NEAR((std::pow(-1, 2) * std::pow(1 - x * x, 2.0 / 2.0)) * arr[13 * 2 + 2], 3 * (1 - x*x), 0.00001);
    ASSERT_NEAR((std::pow(-1, 3) * std::pow(1 - x * x, 3.0 / 2.0)) * arr[13 * 3 + 3], -15 * std::sqrt(std::pow(1 - x*x, 3)), 0.00001);
    
    x = 0.2;

    t.CalculateMthLthOrderAssociatedLegrandreFunctionDerivatives(12, 12, x);

    arr = t.GetMthLthOrderAssociatedLegrandreFunctionDerivatives();

    ASSERT_NEAR(
        std::pow(-1, 4) * std::pow(1 - x * x, 4.0 / 2.0) * arr[13 * 5 + 4],
        174.1824,
        0.000001
    );

    ASSERT_NEAR((std::pow(-1, 2) * std::pow(1 - x * x, 2.0 / 2.0)) * arr[13 * 3 + 2], 15 * x * (1 - x*x), 0.00001);
}

TEST(AssociatedLegendreDerivativesTest, LegrandrePolynomialValuesMatchExpected) {
    MagneticDeclination t = MagneticDeclination();

    double x = 0.2;
    int n = 6;
    int m = 5;

    t.LoadCOF("C:/Users/maxid/Workspace/IMU_GPS_KF/build/Debug/WMM.COF");

    t.SetAssociatedPolynomialMatrix(x);

    double real = t.associatedLegendrePolynomial(x, n, m);

    EXPECT_NEAR(-1877.29520956, real, 0.0001);

    x = 0.932;
    n = 11;
    m = 2;

    t.SetAssociatedPolynomialMatrix(x);

    real = t.associatedLegendrePolynomial(x, n, m);

    EXPECT_NEAR(39.4229083879, real, 0.0001);
}

TEST(AssociatedLegendreDerivativesTest, pLegenValuesMatchExpected) {
    MagneticDeclination t = MagneticDeclination();

    double x = 0.2;
    int n = 6;
    int m = 5;

    t.LoadCOF("C:/Users/maxid/Workspace/IMU_GPS_KF/build/Debug/WMM.COF");

    t.SetAssociatedPolynomialMatrix(x);

    double real = t.pLegen(x, n, m);

    EXPECT_NEAR(pow(-1, m) * -1877.29520956, real, 0.0001);

    x = 0.932;
    n = 11;
    m = 2;

    t.SetAssociatedPolynomialMatrix(x);

    real = t.associatedLegendrePolynomial(x, n, m);

    EXPECT_NEAR(pow(-1, m) * 39.4229083879, real, 0.0001);
}

TEST(AssociatedLegendreDerivativesTest, pHatValuesMatchExpected) {
    MagneticDeclination t = MagneticDeclination();

    double x = 0.10121;
    int n = 8;
    int m = 5;

    t.LoadCOF("C:/Users/maxid/Workspace/IMU_GPS_KF/build/Debug/WMM.COF");

    t.SetAssociatedPolynomialMatrix(x);

    double real = t.pHat(x, n, m);

    EXPECT_NEAR(-8.04524134213, real, 0.0001);

    x = 0.832;
    n = 12;
    m = 3;

    t.SetAssociatedPolynomialMatrix(x);

    real = t.pHat(x, n, m);

    EXPECT_NEAR(-3.43132206218, real, 0.0001);

    x = 0.532;
    n = 3;
    m = 1;

    t.SetAssociatedPolynomialMatrix(x);

    real = t.pHat(x, n, m);

    EXPECT_NEAR(0.215249227119, real, 0.0001);

    x = 0.13231;
    n = 8;
    m = 0;

    t.SetAssociatedPolynomialMatrix(x);

    real = t.pHat(x, n, m);

    EXPECT_NEAR(0.117206523949, real, 0.0001);
}

// TEST(AssociatedLegendreDerivativesTest, dPHatdPhiPrimeValuesMatchExpected) {
//     MagneticDeclination t = MagneticDeclination();

//     double x = 13.12;
//     int n = 5;
//     int m = 2;

//     t.LoadCOF("C:/Users/maxid/Workspace/IMU_GPS_KF/build/Debug/WMM.COF");

//     t.SetAssociatedPolynomialMatrix(x);

//     double real = t.dPHatdPhiPrime(x, n, m);

//     EXPECT_NEAR(5.85139124478, real, 0.0001);

//     x = -0.2532;
//     n = 11;
//     m = 1;

//     t.SetAssociatedPolynomialMatrix(x);

//     real = t.dPHatdPhiPrime(x, n, m);

//     EXPECT_NEAR(-0.959113360328, real, 0.0001);

//     x = -12.8123;
//     n = 2;
//     m = 0;

//     t.SetAssociatedPolynomialMatrix(x);

//     real = t.dPHatdPhiPrime(x, n, m);

//     EXPECT_NEAR(-0.708397693788, real, 0.0001);
// }

