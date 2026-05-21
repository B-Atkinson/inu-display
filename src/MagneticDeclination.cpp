#include <cmath>
#include <Eigen/Dense>
#include <iostream>

#include "MagneticDeclination.hpp"

const int MagneticDeclination::N = 12;
const double MagneticDeclination::A = 6378137;
const double MagneticDeclination::F = 1.0 / 298.257223563;
const double MagneticDeclination::E = sqrt( F * ( 2 - F ) );

MagneticDeclination::MagneticDeclination()
{
    this->m_epoch = 2025.0;
}

MagneticDeclination::~MagneticDeclination()
{
    // delete[] legendrePolynomialMatrix;
}

void MagneticDeclination::SetAssociatedPolynomialMatrix(double x) {
    CalculateMthLthOrderAssociatedLegrandreFunctionDerivatives(N, N, x);
}

double MagneticDeclination::declination(double lambda, double phi, double h, double t)
{
    double sinPhi = sin(phi);
    double rc = A / sqrt(1 - E * E * sinPhi * sinPhi);
    double _p = p(rc, h, phi);
    double _z = z(rc, h, phi);
    double r = sqrt(_p * _p + _z * _z);
    double phiPrime = asin(_z/ r);

    SetAssociatedPolynomialMatrix(sin(phiPrime));
    
    double x = xPrime(lambda, phiPrime, r, t) * cos(phiPrime - phi) - zPrime(lambda, phiPrime, r, t) * sin(phiPrime - phi);

    double y = yPrime(lambda, phiPrime, r, t);

    return atan2(y, x);
}

double MagneticDeclination::v(double lambda, double phiPrime, double r, double t)
{
    double geoRefRadiusMeters = 6371200;
    
    double outerTerm = 0;
    for(int n = 1; n <= N; n++)
    {
        double innerTerm = 0;
        for(int m = 0; m <= n; m++)
        {
            innerTerm += (g(n, m, t) * cos(m * lambda) + h(n, m, t) * sin(m * lambda)) * pHat(sin(phiPrime), n, m);
        }

        outerTerm += pow(geoRefRadiusMeters / r, n + 1.0) * innerTerm;
    }

    return geoRefRadiusMeters * outerTerm;
}

double MagneticDeclination::g(int n, int m, int t)
{
    return getG(n, m) + (t - this->m_epoch) * getGDot(n, m);
}

double MagneticDeclination::h(int n, int m, int t)
{
    return getH(n, m) + (t - this->m_epoch) * getHDot(n, m);
}

double MagneticDeclination::xPrime(double lambda, double phiPrime, double r, int t)
{
    double geoRefRadiusMeters = 6371200;
    double outerTerm = 0;

    for(int n = 1; n <= N; n++)
    {
        double innerTerm = 0;
        for(int m = 0; m <= n; m++)
        {
            innerTerm += (g(n, m, t) * cos(m * lambda) + h(n, m, t) * sin(m * lambda)) * dPHatdPhiPrime(sin(phiPrime), n, m);
        }

        outerTerm += pow(geoRefRadiusMeters / r, n + 2) * innerTerm;
    }

    return -1.0 * outerTerm;
}

double MagneticDeclination::yPrime(double lambda, double phiPrime, double r, int t)
{
    double geoRefRadiusMeters = 6371200;
    double outerTerm = 0;

    for(int n = 1; n <= N; n++)
    {
        double innerTerm = 0;
        for(int m = 0; m <= n; m++)
        {
            innerTerm += m * (g(n, m, t) * sin(m * lambda) - h(n, m, t) * cos(m * lambda)) * pHat(sin(phiPrime), n, m);
        }

        outerTerm += pow(geoRefRadiusMeters / r, n + 2) * innerTerm;
    }

    return (1.0 / cos(phiPrime)) * outerTerm;
}

double MagneticDeclination::zPrime(double lambda, double phiPrime, double r, int t)
{
    double geoRefRadiusMeters = 6371200;
    double outerTerm = 0;

    for(int n = 1; n <= N; n++)
    {
        double innerTerm = 0;
        for(int m = 0; m <= n; m++)
        {
            innerTerm += (g(n, m, t) * cos(m * lambda) + h(n, m, t) * sin(m * lambda)) * pHat(sin(phiPrime), n, m);
        }

        outerTerm += (n + 1) * pow(geoRefRadiusMeters / r, n + 2) * innerTerm;
    }

    return -1.0 * outerTerm;
}

double MagneticDeclination::xPrimeDot(double lambda, double phiPrime, double r, int t)
{
    double geoRefRadiusMeters = 6371200;
    double outerTerm = 0;

    for(int n = 1; n <= N; n++)
    {
        double innerTerm = 0;
        for(int m = 0; m <= n; m++)
        {
            innerTerm += (getGDot(n, m) * cos(m * lambda) + getHDot(n, m) * sin(m * lambda)) * dPHatdPhiPrime(sin(phiPrime), n, m);
        }

        outerTerm += pow(geoRefRadiusMeters / r, n + 2) * innerTerm;
    }

    return -1.0 * outerTerm;
}

double MagneticDeclination::yPrimeDot(double lambda, double phiPrime, double r, int t)
{
    double geoRefRadiusMeters = 6371200;
    double outerTerm = 0;

    for(int n = 1; n <= N; n++)
    {
        double innerTerm = 0;
        for(int m = 0; m <= n; m++)
        {
            innerTerm += m * (getGDot(n, m) * sin(m * lambda) - getHDot(n, m) * cos(m * lambda)) * pHat(sin(phiPrime), n, m);
        }

        outerTerm += pow(geoRefRadiusMeters / r, n + 2) * innerTerm;
    }

    return (1.0 / cos(phiPrime)) * outerTerm;
}

double MagneticDeclination::zPrimeDot(double lambda, double phiPrime, double r, int t)
{
    double geoRefRadiusMeters = 6371200;
    double outerTerm = 0;

    for(int n = 1; n <= N; n++)
    {
        double innerTerm = 0;
        for(int m = 0; m <= n; m++)
        {
            innerTerm += (getGDot(n, m) * cos(m * lambda) + getHDot(n, m) * sin(m * lambda)) * pHat(sin(phiPrime), n, m);
        }

        outerTerm += (n + 1) * pow(geoRefRadiusMeters / r, n + 2) * innerTerm;
    }

    return -1.0 * outerTerm;
}

double MagneticDeclination::pHat(double u, int n, int m)
{
    if(m == 0)
    {
        return pLegen(u, n, m);
    }

    double coeff = sqrt(2.0 * (1.0 / partialFactorial(n, (n + m))));
    return coeff * pLegen(u, n, m);
}

double MagneticDeclination::pLegen(double u, double n, double m)
{
    return pow(-1, m) * associatedLegendrePolynomial(u, n, m);
}

double MagneticDeclination::associatedLegendrePolynomial(double u, int n, int m)
{
    double coeff = 1 - u * u;
    double sign = (m & 0x1) == 0 ? 1.0 : -1.0;

    return sign * pow(coeff, m / 2.0) * legendrePolynomialMatrix[(N + 1) * n + m];
}

double MagneticDeclination::dPHatdPhiPrime(double phiPrime, int n, int m)
{
    double firstTerm = (n + 1) * tan(phiPrime) * pHat(sin(phiPrime), n, m);
    double secondTerm = sqrt((n + 1) * (n + 1) - m * m) * (1.0 / cos(phiPrime)) * pHat(sin(phiPrime), n + 1, m);
    return firstTerm - secondTerm;
}

double MagneticDeclination::p(double r, double h, double phi)
{
    return (r + h) * cos(phi);
}

double MagneticDeclination::z(double r, double h, double phi)
{
    return (r * (1 - E * E) + h) * sin(phi);
}

double MagneticDeclination::radiusPrimeVerticle(double phi)
{
    double coeff = sin(phi);
    return A / sqrt(1 - E * E * coeff * coeff);
}

unsigned long long MagneticDeclination::partialFactorial(unsigned int start, unsigned int end)
{
    unsigned long long n = 1;
    for(int i = start; i <= end; i++)
    {
        n *= i;
    }

    return n;
}

double MagneticDeclination::FallingFactorial(double n, int m) {
    if (m == 0) return 1.0;
 
    double result = 1.0;
    for (int i = 0; i < m; ++i) {
        result *= (n - i);
    }
    return result;
}

double MagneticDeclination::Binomial(int n, int r) {
    if (r < 0 || r > n) return 0.0;
    r = std::min(r, n - r);
    return FallingFactorial(n, r) / FallingFactorial(r, r);
}

void 
MagneticDeclination::CalculateMthLthOrderAssociatedLegrandreFunctionDerivatives(unsigned M,unsigned L,double x) {
    const unsigned rows = L + 1;
    const unsigned cols = M + 1;

    free(this->legendrePolynomialMatrix);

    this->legendrePolynomialMatrix = static_cast<double*>(calloc(rows * cols, sizeof(double)));

    auto index = [cols](unsigned l, unsigned m) {
        return cols * l + m;
    };

    this->legendrePolynomialMatrix[index(0, 0)] = 1.0;

    for (unsigned l = 1; l <= L; l++) {
        double twoToTheL = std::ldexp(1.0, static_cast<int>(l)); 

        for (unsigned m = 0; m <= M; m++) {
            double combinatorialSum = 0.0;

            if (m <= l) {
                unsigned kStart = (l + m + 1) / 2; 

                for (unsigned k = kStart; k <= l; k++) {
                    int twoKL = 2 * static_cast<int>(k) - static_cast<int>(l);
                    int exponent = twoKL - static_cast<int>(m);
                    double sign = ((k + l) & 0x1) == 0 ? 1.0 : -1.0;
                    double factorialTerm = sign * Binomial(2 * k, l) * Binomial(l, k);

                    combinatorialSum += factorialTerm * FallingFactorial(twoKL, m) * std::pow(x, exponent);
                }
            }

            this->legendrePolynomialMatrix[index(l, m)] = combinatorialSum / twoToTheL;
        }
    }
}

double* MagneticDeclination::GetMthLthOrderAssociatedLegrandreFunctionDerivatives() {
    return this->legendrePolynomialMatrix;
}

double MagneticDeclination::getG(int n, int m) {
    std::pair<int, int> key = std::make_pair(n, m);

    NOAA_COF_COEFFS coeff = this->m_coeffMap[key];

    return coeff.g;
}

double MagneticDeclination::getGDot(int n, int m) {
    std::pair<int, int> key = std::make_pair(n, m);

    NOAA_COF_COEFFS coeff = this->m_coeffMap[key];

    return coeff.gdot;
}

double MagneticDeclination::getH(int n, int m) {
    std::pair<int, int> key = std::make_pair(n, m);

    NOAA_COF_COEFFS coeff = this->m_coeffMap[key];

    return coeff.h;
}   

double MagneticDeclination::getHDot(int n, int m) {
    std::pair<int, int> key = std::make_pair(n, m);

    NOAA_COF_COEFFS coeff = this->m_coeffMap[key];

    return coeff.hdot;
}

void MagneticDeclination::LoadCOF(const std::string& filePath)
{
    // std::cout << "cof1" << std::endl;

    FILE* fp = fopen(filePath.c_str(), "r");
    if (!fp)
        throw std::runtime_error("Failed to open COF file: " + filePath);
    // std::cout << "cof2" << std::endl;

    char model_name[64];
    char release_date[64];

    fscanf(fp, "%lf %s %s", &this->m_epoch, model_name, release_date);
    // std::cout << "cof3" << std::endl;

    NOAA_COF_COEFFS coeffs;
    // std::cout << "cof4" << std::endl;

    while (fscanf(fp, "%3d%3d%10.1f%10.1f%11.1f%11.1f",
        &coeffs.n, &coeffs.m,
        &coeffs.g, &coeffs.h,
        &coeffs.gdot, &coeffs.hdot) == 6)
    {
        if (coeffs.n == 999)
            break;

        m_coeffMap[{coeffs.n, coeffs.m}] = coeffs;
    }
    // std::cout << "cof5" << std::endl;

    fclose(fp);
}
