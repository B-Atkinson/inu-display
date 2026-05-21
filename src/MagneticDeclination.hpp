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

    void LoadCOF(const std::string& filePath);

    double declination(double lambda, double phi, double h, double t);

    double v(double lambda, double phiPrime, double r, double t);
    double g(int n, int m, int t);
    double h(int n, int m, int t);

    double pHat(double u, int n, int m);
    double dPHatdPhiPrime(double phiPrime, int n, int m);
    double pLegen(double u, double n, double m);
    double associatedLegendrePolynomial(double u, int n, int m);
    void CalculateMthLthOrderAssociatedLegrandreFunctionDerivatives(unsigned M, unsigned L, double x);
    double* GetMthLthOrderAssociatedLegrandreFunctionDerivatives();
    double Binomial(int n, int r);
    double FallingFactorial(double n, int m);

    double xPrime(double lambda, double phiPrime, double r, int t);
    double yPrime(double lambda, double phiPrime, double r, int t);
    double zPrime(double lambda, double phiPrime, double r, int t);

    double xPrimeDot(double lambda, double phiPrime, double r, int t);
    double yPrimeDot(double lambda, double phiPrime, double r, int t);
    double zPrimeDot(double lambda, double phiPrime, double r, int t);

    double p(double r, double h, double phi);
    double z(double r, double h, double phi);
    double radiusPrimeVerticle(double phi);

    double getG(int n, int m);
    double getGDot(int n, int m);
    double getH(int n, int m);
    double getHDot(int n, int m);

    unsigned long long partialFactorial(unsigned int start, unsigned int end);

    void SetAssociatedPolynomialMatrix(double x);

private:
    double *legendrePolynomialMatrix = static_cast<double*>(calloc(13 * 13, sizeof(double)));
    std::map<std::pair<int, int>, NOAA_COF_COEFFS> m_coeffMap;
    double m_epoch;
};

#endif