#ifndef GPSCONVERSION_H
#define GPSCONVERSION_H

#include <cmath>




/**
 * \brief Converts lat,lon coordinates into ENU frame
 *
 * The implementation here is according to the paper:
 * - "Conversion of Geodetic coordinates to the Local Tangent Plane" Version 2.01.
 * - "The basic reference for this paper is J.Farrell & M.Barth 'The Global Positioning System & Inertial Navigation'"
 * - Also helpful is Wikipedia: http://en.wikipedia.org/wiki/Geodetic_datum
 * - Taken from https://gist.github.com/govert/1b373696c9a27ff4c72a
 */
class GPSConversion {

public:

    // WGS-84 geodetic constants
    static constexpr double a = 6378137;              // WGS-84 Earth semimajor axis (m)
    static constexpr double b = 6356752.3142;         // WGS-84 Earth semiminor axis (m)

    /**
     * Converts WGS-84 Geodetic point (lat, lon, h) to the
     * Earth-Centered Earth-Fixed (ECEF) coordinates (x, y, z).
     */
    static void GeodeticToEcef(double lat, double lon, double h, double& x, double& y, double& z) {

        double f = (a - b) / a;      // Ellipsoid Flatness
        double e_sq = f * (2 - f);   // Square of Eccentricity

        // Convert to radians in notation consistent with the paper:
        double lambda = DegreeToRadian(lat);
        double phi = DegreeToRadian(lon);
        double s = sin(lambda);
        double N = a / sqrt(1 - e_sq * s * s);

        double sin_lambda = sin(lambda);
        double cos_lambda = cos(lambda);
        double cos_phi = cos(phi);
        double sin_phi = sin(phi);

        x = (h + N) * cos_lambda * cos_phi;
        y = (h + N) * cos_lambda * sin_phi;
        z = (h + (1 - e_sq) * N) * sin_lambda;
    }

    /**
     * Converts the Earth-Centered Earth-Fixed (ECEF) coordinates (x, y, z) to
     * East-North-Up coordinates in a Local Tangent Plane that is centered at the
     * (WGS-84) Geodetic point (lat0, lon0, h0).
     */
    static void EcefToEnu(double x, double y, double z, double lat0, double lon0, double h0,
                          double& xEast, double& yNorth, double& zUp) {

        double f = (a - b) / a;      // Ellipsoid Flatness
        double e_sq = f * (2 - f);   // Square of Eccentricity

        // Convert to radians in notation consistent with the paper:
        double lambda = DegreeToRadian(lat0);
        double phi = DegreeToRadian(lon0);
        double s = sin(lambda);
        double N = a / sqrt(1 - e_sq * s * s);

        double sin_lambda = sin(lambda);
        double cos_lambda = cos(lambda);
        double cos_phi = cos(phi);
        double sin_phi = sin(phi);

        double x0 = (h0 + N) * cos_lambda * cos_phi;
        double y0 = (h0 + N) * cos_lambda * sin_phi;
        double z0 = (h0 + (1 - e_sq) * N) * sin_lambda;

        double xd, yd, zd;
        xd = x - x0;
        yd = y - y0;
        zd = z - z0;

        // This is the matrix multiplication
        xEast = -sin_phi * xd + cos_phi * yd;
        yNorth = -cos_phi * sin_lambda * xd - sin_lambda * sin_phi * yd + cos_lambda * zd;
        zUp = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd;
    }

    /**
     * Converts the geodetic WGS-84 coordinated (lat, lon, h) to
     * East-North-Up coordinates in a Local Tangent Plane that is centered at the
     * (WGS-84) Geodetic point (lat0, lon0, h0).
     */
    static void GeodeticToEnu(double lat, double lon, double h, double lat0, double lon0, double h0,
                              double& xEast, double& yNorth, double& zUp) {
        double x, y, z;
        GeodeticToEcef(lat, lon, h, x, y, z);
        EcefToEnu(x, y, z, lat0, lon0, h0, xEast, yNorth, zUp);
    }

private:

    /**
     * Converts degrees to radians
     * \param angle The angle in degrees
     * \return Angle converted into radians
     */
    static double DegreeToRadian(double angle) {
        return M_PI * angle / 180.0;
    }




};

#endif //GPSCONVERSION_H