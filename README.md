# arduino_ms5611_spi

I've seen one or two I²C libraries for the MS5611 Temperature and Pressure sensor, but
while I²C is easier to implement and less taxing on the arduino (I think), it produces
less reliable results from the sensor. SPI will give a better, more accurate and
consistent result. If I recall the info on the datasheet properly, it has something to
do with the power consumption of the I²C hardware on the sensor drawing power 
unpredictably, causing voltage fluctuation that affects the accuracy of the sensor.

There is one major limitation of this library and one caveat about its use. The
limitation is that it is completely synchronous. Before I finish my current project,
I'll be fixing this. You'll have the option of setting it up to be synchronous or to
have to call a function to see if data are available. The caveat comes from the sensor
datasheet. **If you choose to alter this library to be asynchronous**, yourself, remember
that having any other traffic on the MS5611's SPI channel during conversion (the 
8500us time between request for data and its retrieval) will damage the fidelity of
the reading.

I should have an example in the repo, but you'll have to make do with my latest test:
```
#include "arduino_ms5611_spi.h"

MS5611_SPI *sensors[2];

void setup() {
  Serial.begin(19200);
  pinMode(LED_BUILTIN, OUTPUT);

  sensors[0] = new MS5611_SPI( 7,  6,  8, 5, 100);
  sensors[1] = new MS5611_SPI(11, 10, 12, 9, 100);
  sensors[0]->reset();
  sensors[1]->reset();
}

int count = 0;

void loop() {
  Serial.print(count++);
  for(int sensor=0; sensor<2; sensor++) {
    sensors[sensor]->update();
    Serial.print(" T: ");
    Serial.print(sensors[sensor]->get_temperature());
    Serial.print(" P: ");
    Serial.print(sensors[sensor]->get_pressure());
  }
  Serial.println();

  delay (100);
}
```
