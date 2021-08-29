#ifndef ARDUINO_MS5611_SPI
#define ARDUINO_MS5611_SPI

#include <Arduino.h>
#include <SPI.h>

// TODO:
// * Do the CRC check.

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

  //get temp in hundredths of a degree centigrade. (centi-centigrades?) e.g., 2557 is 25.57C
  int32_t get_temperature_int();

  //get pressure in hundredths of a millibar (centi-milibars?) e.g., 100009 is 1000.09 mbar
  int32_t get_pressure_int();

  //get temp in centigrade.
  float get_temperature_float();

  //get pressure in millibar.
  float get_pressure_float();
  
  void reset();

private:
  static const byte FACTORY_DATA_ADDR = 0xa0; // No info in datasheet other than "factory data".
  static const byte SENS_T1_ADDR = 0xa2;      // "Pressure sensitivity"
  static const byte OFF_T1_ADDR = 0xa4;       // "Pressure offset"
  static const byte TCS_ADDR = 0xa6;          // "Temperature coefficient of pressure sensitivity"
  static const byte TCO_ADDR = 0xa8;          // "Temperature coefficient of pressure offset"
  static const byte T_REF_ADDR = 0xaa;        // "Reference temperature"
  static const byte TEMPSENS_ADDR = 0xac;     // "Temperature coefficient of the temperature"
  static const byte CRC_ADDR = 0xae;          // Checksum
  
  // Pardon the all-caps naming. I'm keeping as close to the datasheet as possible.
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
  uint32_t spi_command(byte command, unsigned int wait_time, int result_len);
  bool time_elapsed(uint32_t last_timestamp);
};

#endif
