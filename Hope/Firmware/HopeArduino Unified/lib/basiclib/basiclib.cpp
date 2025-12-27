#include <Arduino.h>
#include <basiclib.h>
#include <ArduinoEigenDense.h>

int parsecommand(uint8_t command){
    switch (command)
    {
    case 0x4:

        Serial.println("triggering camera");
        digitalWrite(UART_TX_PIN,LOW);
        delay(1000);
        digitalWrite(UART_TX_PIN, HIGH);

        // packet testpacket;
        // testpacket.r.uptime = millis();
        // testpacket.r.command = 0xFE;
        // testpacket.r.battvoltage = 80.0;
        // testpacket.r.checksum = 0xAB;
        // testpacket.r.lat = 24.5;
        // testpacket.r.lat = 117.5;
        // radio.sendpacket(testpacket);
        break;
    
    default:
        break;
    }
    return 0;
}

float getbatteryvoltage(){
    int rawvalue = analogRead(PIN_BATTSENSE);

    int mappedvalue = map(rawvalue,0,1023,0,330);

    float realval = float(mappedvalue)/(51.0/(51.0+30.0));

    return realval/100;

}



// WGS84 Earth parameters
constexpr double a = 6378137.0;           // Semi-major axis (meters)
constexpr double f = 1 / 298.257223563;   // Flattening
constexpr double e2 = f * (2 - f);        // Square of eccentricity
constexpr double b = a * (1 - f);
constexpr double ep2 = (a*a - b*b) / (b*b);

// Convert latitude, longitude, and altitude to ECEF
Eigen::Vector3d gpsToECEF(double lat, double lon, double alt) {
    double lat_rad = lat * M_PI / 180.0;
    double lon_rad = lon * M_PI / 180.0;

    double N = a / sqrt(1 - e2 * sin(lat_rad) * sin(lat_rad));
    
    double x = (N + alt) * cos(lat_rad) * cos(lon_rad);
    double y = (N + alt) * cos(lat_rad) * sin(lon_rad);
    double z = ((1 - e2) * N + alt) * sin(lat_rad);

    return Eigen::Vector3d(x, y, z);
}

// Compute the rotation matrix from ECEF to ENU
Eigen::Matrix3d ecefToEnuMatrix(double lat, double lon) {
    double lat_rad = lat * M_PI / 180.0;
    double lon_rad = lon * M_PI / 180.0;

    Eigen::Matrix3d R;
    R << -sin(lon_rad),  cos(lon_rad), 0,
         -sin(lat_rad) * cos(lon_rad), -sin(lat_rad) * sin(lon_rad), cos(lat_rad),
          cos(lat_rad) * cos(lon_rad),  cos(lat_rad) * sin(lon_rad), sin(lat_rad);

    return R;
}

// Convert GPS coordinates to local ENU coordinates
Eigen::Vector3d gpsToENU(double lat, double lon, double alt, 
                         double refLat, double refLon, double refAlt) {
    Eigen::Vector3d refECEF = gpsToECEF(refLat, refLon, refAlt);
    Eigen::Vector3d pointECEF = gpsToECEF(lat, lon, alt);

    Eigen::Vector3d deltaECEF = pointECEF - refECEF;
    Eigen::Matrix3d R = ecefToEnuMatrix(refLat, refLon);

    return R * deltaECEF;
}


bool trilaterate(const Matrix3d &points, const Vector3d &radii,
    Vector3d &solution1, Vector3d &solution2) {

    // Extract the three reference points
    Vector3d p1 = points.row(0);
    Vector3d p2 = points.row(1);
    Vector3d p3 = points.row(2);

    // Extract the radii
    double r1 = radii(0);
    double r2 = radii(1);
    double r3 = radii(2);

    // Compute unit vector ex (direction from p1 to p2)
    Vector3d ex = p2 - p1;
    double d = ex.norm();  // Distance between p1 and p2
    if (d == 0) return false;  // Prevent division by zero
    ex.normalize();

    // Compute the vector from p1 to p3
    Vector3d temp = p3 - p1;

    // Compute i as dot product temp ⋅ ex
    double i = temp.dot(ex);

    // Compute unit vector ey (perpendicular to ex in the plane of p1, p2, p3)
    Vector3d ey = temp - i * ex;
    double j = ey.norm();
    if (j == 0) return false;  // Prevent division by zero
    ey.normalize();

    // Compute unit vector ez (perpendicular to the plane formed by p1, p2, p3)
    Vector3d ez = ex.cross(ey);

    // Solve for x, y
    double x = (r1 * r1 - r2 * r2 + d * d) / (2 * d);
    double y = (r1 * r1 - r3 * r3 + i * i + j * j - 2 * i * x) / (2 * j);
    double zSquared = r1 * r1 - x * x - y * y;

    if (zSquared < 0) return false;  // No real solution

    double z1 = std::sqrt(zSquared);
    double z2 = -z1;  // Second solution

    // Compute both possible solutions in original space
    solution1 = p1 + x * ex + y * ey + z1 * ez;
    solution2 = p1 + x * ex + y * ey + z2 * ez;

    return true;
}


// ENU -> ECEF (relative to reference)
Eigen::Vector3d enuToECEF(const Eigen::Vector3d& enu,
                          double refLat_deg, double refLon_deg, double refAlt) {
    Eigen::Matrix3d R_ecef2enu = ecefToEnuMatrix(refLat_deg, refLon_deg);
    Eigen::Matrix3d R_enu2ecef = R_ecef2enu.transpose();
    Eigen::Vector3d refECEF = gpsToECEF(refLat_deg, refLon_deg, refAlt);
    return refECEF + R_enu2ecef * enu;
}

// ECEF -> Geodetic (lat[deg], lon[deg], alt[m]) using Bowring’s formula
void ecefToLLA(const Eigen::Vector3d& ecef,
               double& lat_deg, double& lon_deg, double& alt) {
    double x = ecef.x(), y = ecef.y(), z = ecef.z();

    double p = std::hypot(x, y);
    double lon = std::atan2(y, x);

    // Auxiliary latitude
    double theta = std::atan2(z * a, p * b);

    double sin_theta = std::sin(theta), cos_theta = std::cos(theta);

    double lat = std::atan2(z + ep2 * b * sin_theta * sin_theta * sin_theta,
                            p - e2  * a * cos_theta * cos_theta * cos_theta);

    double sin_lat = std::sin(lat);
    double N = a / std::sqrt(1.0 - e2 * sin_lat * sin_lat);
    double h = p / std::cos(lat) - N;

    lat_deg = lat * 180.0 / M_PI;
    lon_deg = lon * 180.0 / M_PI;
    alt = h;
}

// Convenience: ENU -> LLA in one call
void enuToLLA(const Eigen::Vector3d& enu,
              double refLat_deg, double refLon_deg, double refAlt,
              double& lat_deg, double& lon_deg, double& alt) {
    Eigen::Vector3d ecef = enuToECEF(enu, refLat_deg, refLon_deg, refAlt);
    ecefToLLA(ecef, lat_deg, lon_deg, alt);
}



