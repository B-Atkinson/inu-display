#ifndef MAGNETIC_DECLINATION_H
#define MAGNETIC_DECLINATION_H

#pragma once

#include <map>
#include <string>

struct NOAA_COF_COEFFS {
    int n;
    int m;
    double g;
    double h;
    double gdot;
    double hdot;
};

class MagneticDeclination
{
public:
    MagneticDeclination();
    ~MagneticDeclination();

    static const int N;     // WMM Degree of expansion
    static const double A;  // Semi major axis, meters
    static const double F;  // Flattening
    static const double E;  // Eccentricity

    static void LoadCOF(const std::string& filePath);

    static double declination(double lambda, double phi, double h, double t);

    static double v(double lambda, double phiPrime, double r, double t);
    static double g(int n, int m, int t);
    static double h(int n, int m, int t);

    static double pHat(double u, int n, int m);
    static double dPHatdPhiPrime(double phiPrime, int n, int m);
    static double pLegen(double u, double n, double m);
    static double associatedLegendrePolynomial(double u, int n, int m);
    static double* CalculateMthLthOrderAssociatedLegrandreFunctionDerivatives(unsigned M, unsigned L, double x);
    static double Binomial(int n, int r);
    static double FallingFactorial(double n, int m);

    static double xPrime(double lambda, double phiPrime, double r, int t);
    static double yPrime(double lambda, double phiPrime, double r, int t);
    static double zPrime(double lambda, double phiPrime, double r, int t);

    static double xPrimeDot(double lambda, double phiPrime, double r, int t);
    static double yPrimeDot(double lambda, double phiPrime, double r, int t);
    static double zPrimeDot(double lambda, double phiPrime, double r, int t);

    static double p(double r, double h, double phi);
    static double z(double r, double h, double phi);
    static double radiusPrimeVerticle(double phi);

    static double getG(int n, int m);
    static double getGDot(int n, int m);
    static double getH(int n, int m);
    static double getHDot(int n, int m);

    static unsigned long long partialFactorial(unsigned int start, unsigned int end);

    static void SetAssociatedPolynomialMatrix(double x);

private:
    static double *legendrePolynomialMatrix;
    static std::map<std::pair<int, int>, NOAA_COF_COEFFS> m_coeffMap;
    static double m_epoch;
};

#endif