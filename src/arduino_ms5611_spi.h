#ifndef ARDUINO_MS5611_SPI
#define ARDUINO_MS5611_SPI

#include <SPI.h>

// TODO:
// * Do the CRC check.

class KalmanFilter {
public:
  // measurement_error - will hold constant at the error level of your sensor. (If err is +-X, use 2X)
  // estimate_error - probably good to start at half of measurement_error.
  KalmanFilter(float measurement_error, float estimate_error);

  float tick(float measurement);
  
  void reset();

  float get_estimate() { return m_estimate; }

  int update_count;
private:
  bool m_initted;
  float m_estimate;
  float m_measurement_error;
  float m_estimate_error;
  float m_initial_estimate_error;
};

class MS5611_SPI {
public:
  // The argument order was chosen to allow me to look at a breadboard and, without knowing which
  // line was mosi or sck, just enter the processor's IO pins in the order that they're connected
  // to the ms5611. 
  // csb_pin:  ms5611 pin 4 or 5 (they're supposedly internally connected. I use 5.) 
  //     CSB LOW = Chip Selected.
  // miso_pin: ms5611 pin 6
  // mosi_pin: ms5611 pin 7
  // sck_pin:  ms5611 pin 8
  MS5611_SPI(int csb_pin, int miso_pin, int mosi_pin, int sck_pin, int sck_delay);

  // Returns true if new values were computed. It will take AT LEAST 4 calls to tick() to get new
  // values. The first call will trigger a request to the sensor, asking for a pressure value. The
  // subsequent calls will do nothing until 9ms have passed. After the delay, the pressure result
  // is fetched from the chip and the temperature is requested. After another 9ms have passed, the
  // next tick() will fetch the temperature result and compute calibrated temp and pressure values
  // to store in the class instance. When this happens, you'll get a true result from the call.
  // Otherwise, it will return false.
  bool tick();

  //get temp in centigrade.
  float get_temperature();

  //get pressure in millibar.
  float get_pressure();

  //get altitude in meters.
  float get_altitude();
  
  static float bar_and_temp_to_alt(float bar, float temp);

  void reset();
  
private:
  bool m_initted;
  
  static const uint8_t FACTORY_DATA_ADDR = 0xa0; // No info in datasheet other than "factory data".
  static const uint8_t SENS_T1_ADDR = 0xa2;      // "Pressure sensitivity"
  static const uint8_t OFF_T1_ADDR = 0xa4;       // "Pressure offset"
  static const uint8_t TCS_ADDR = 0xa6;          // "Temperature coefficient of pressure sensitivity"
  static const uint8_t TCO_ADDR = 0xa8;          // "Temperature coefficient of pressure offset"
  static const uint8_t T_REF_ADDR = 0xaa;        // "Reference temperature"
  static const uint8_t TEMPSENS_ADDR = 0xac;     // "Temperature coefficient of the temperature"
  static const uint8_t CRC_ADDR = 0xae;          // Checksum
  
  // Pardon the all-caps, nonstandard naming. I'm keeping as close to the datasheet as possible.
  uint16_t FACTORY_DATA;               // No info in datasheet other than "factory data".
  uint16_t SENS_T1;                    // "Pressure sensitivity"
  uint16_t OFF_T1;                     // "Pressure offset"
  uint16_t TCS = 0xa6;                 // "Temperature coefficient of pressure sensitivity"
  uint16_t TCO = 0xa8;                 // "Temperature coefficient of pressure offset"
  uint16_t T_REF = 0xaa;               // "Reference temperature"
  uint16_t TEMPSENS = 0xac;            // "Temperature coefficient of the temperature"
  uint16_t CRC;                        // Checksum
  int32_t  TEMP;                       // Computed temperature
  int32_t  P;                          // Computed pressure
  int32_t  D2;                         // Raw pressure data from sensor
  int32_t  D1;                         // Raw temperature data from sensor

  // I'm adding "_pin" to each of these because MOSI, MISO, SCK are already defined on arduino.
  int MOSI_pin;
  int MISO_pin;
  int SCK_pin;
  int CSB_pin;
  int SCK_delay;

  int tick_state = 0;
  uint32_t tick_conversion_request_time = 0;

  void bit_out(int bit);
  int bit_in();
  uint32_t spi_command(uint8_t command, unsigned int wait_time, int result_len);
  bool time_elapsed(uint32_t last_timestamp);
};

#endif
