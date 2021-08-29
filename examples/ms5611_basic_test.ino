#include "arduino_ms5611_spi.h"

//
// CHANGE THIS TO WHATEVER PINS YOU'RE USING TO CONNECT TO THE MS5611.
// Arg0: pin that is connected to ms5611 pins 4/5
// Arg1: pin that is connected to ms5611 pin 6
// Arg2: pin that is connected to ms5611 pin 7
// Arg3: pin that is connected to ms5611 pin 8
// Arg4: Time, in microseconds, to delay between rising and falling clock cycles. 2usec should be fine.
MS5611_SPI ms5611(5, 27, 26, 25, 2);



void setup() {
  Serial.begin(115200);
  delay(100);
  ms5611.reset();
}

void loop() {
  while(!ms5611.tick()) {
    // tick() will return false until there is a new temp/pressure to read.
  }
  Serial.print(ms5611.get_temperature());
  Serial.print("C ");
  Serial.print(ms5611.get_pressure());
  Serial.println("mbar");

  delay(10);
}
