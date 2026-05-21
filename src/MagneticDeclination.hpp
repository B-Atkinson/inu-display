#ifndef MAGNETIC_DECLINATION_H
#define MAGNETIC_DECLINATION_H

#pragma once

#include <map>
#include <string>

struct NOAA_COF_COEFFS {
    NOAA_COF_COEFFS() {
        this->n = 0;
        this->m = 0;
        this->g = 0;
        this->h = 0;
        this->gdot = 0;
        this->hdot = 0;
    }

    NOAA_COF_COEFFS &operator=(const NOAA_COF_COEFFS& coeffs) {
        this->n = coeffs.n;
        this->m = coeffs.m;
        this->g = coeffs.g;
        this->h = coeffs.h;
        this->gdot = coeffs.gdot;
        this->hdot = coeffs.hdot;

        return *this;
    }

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

    void LoadCOF(const std::string& filePath);

    double CalculateDeclination(double lambda, double phi, double h, double t);

    double v(double lambda, double phiPrime, double r, double t);
    double g(int n, int m, double t);
    double h(int n, int m, double t);

    double pHat(double u, int n, int m);
    double dPHatdPhiPrime(double phiPrime, int n, int m);
    double pLegen(double u, double n, double m);

    double AssociatedLegendrePolynomial(double u, int n, int m);
    void CalculateMthLthOrderAssociatedLegrandreFunctionDerivatives(unsigned M, unsigned L, double x);
    double* GetMthLthOrderAssociatedLegrandreFunctionDerivatives();

    double xPrime(double lambda, double phiPrime, double r, double t);
    double yPrime(double lambda, double phiPrime, double r, double t);
    double zPrime(double lambda, double phiPrime, double r, double t);

    double p(double r, double h, double phi);
    double z(double r, double h, double phi);

    double GetG(int n, int m);
    double GetGDot(int n, int m);
    double GetH(int n, int m);
    double GetHDot(int n, int m);

    void SetAssociatedPolynomialMatrix(double x);

private:
    double Binomial(int n, int r);
    double FallingFactorial(double n, int m);

private:
    static const int N;     // WMM Degree of expansion
    static const double A;  // Semi major axis, meters
    static const double F;  // Flattening
    static const double E;  // Eccentricity

    double *m_legendrePolynomialMatrix = static_cast<double*>(calloc(13 * 13, sizeof(double)));
    std::map<std::pair<int, int>, NOAA_COF_COEFFS> m_coeffMap;
    double m_epoch;
};

#endif