#ifndef ARDUINO_MS5611_SPI
#define ARDUINO_MS5611_SPI

#include <SPI.h>

// TODO:
// * Do the CRC check.

class MS5611_SPI {
public:
  // mosi_pin: ms5611 pin 7
  // miso_pin: ms5611 pin 6
  // sck_pin:  ms5611 pin 8
  // csb_pin:  ms5611 pin 4 or 5 (they're supposedly internally connected. I use 5.)
  MS5611_SPI(int mosi_pin, int miso_pin, int sck_pin, int csb_pin, int sck_delay);
  void update();
  int32_t get_temperature();
  int32_t get_pressure();
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

  // I'm adding "_pin" to each of these because MOSI, MISO, SCK are already defined on arduino.
  int MOSI_pin;
  int MISO_pin;
  int SCK_pin;
  int CSB_pin;
  int SCK_delay;

  void bit_out(int bit);
  int bit_in();
  uint32_t spi_command(byte command, unsigned int wait_time, int result_len);
};

#endif
