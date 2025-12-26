#if !defined(BASICLIB)
#define BASICLIB

#include <ArduinoEigenDense.h>

using namespace Eigen;

#define PIN_MISO 28
#define PIN_CS   25
#define PIN_SCK  26
#define PIN_MOSI 27

//Radio pins
#define PIN_DIO1 9
#define PIN_DIO2 1
#define PIN_DIO3 0 // jumpered to RST
#define PIN_BUSY 10 // High when busy
#define PIN_TXCOEN 11 // Active high
#define PIN_RST 17 // active low

#define PIN_LED 8

// Use pins 4 and 5 for UART1
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define UART_TX_PIN 12
#define UART_RX_PIN 13

#define PIN_BATTSENSE A3

union packet
{
    struct{
    uint8_t checksum;
    uint32_t uptime;
    int32_t lat;
    int32_t lon;
    int32_t alt;
    uint16_t battvoltage;
    }r;
    uint8_t data[sizeof(r)];
};

struct stationdata
{
    int ID;
    float lat;
    float lon;
    float alt;
    float distance;
    double xcoord;
    double ycoord;
    double zcoord;
};



int parsecommand(uint8_t command);

float getbatteryvoltage();

Eigen::Vector3d gpsToECEF(double lat, double lon, double alt);

Eigen::Matrix3d ecefToEnuMatrix(double lat, double lon);

Eigen::Vector3d gpsToENU(double lat, double lon, double alt, 
                         double refLat, double refLon, double refAlt);

bool trilaterate(const Matrix3d &points, const Vector3d &radii,
    Vector3d &solution1, Vector3d &solution2);

Eigen::Vector3d enuToECEF(const Eigen::Vector3d& enu,
                          double refLat_deg, double refLon_deg, double refAlt);

void ecefToLLA(const Eigen::Vector3d& ecef,
               double& lat_deg, double& lon_deg, double& alt);
               
void enuToLLA(const Eigen::Vector3d& enu,
              double refLat_deg, double refLon_deg, double refAlt,
              double& lat_deg, double& lon_deg, double& alt);
    

#endif // BASICLIB