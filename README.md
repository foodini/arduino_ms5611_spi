# arduino_ms5611_spi

I've seen one or two I²C libraries for the MS5611 Temperature and Pressure sensor, but
while I²C is easier to implement and less taxing on the arduino (I think), it produces
less reliable results from the sensor, according to the datasheet. It has something to
do with the power consumption of the I²C hardware on the sensor drawing power 
unpredictably, causing voltage fluctuation that affects the accuracy of the sensor. 
SPI gives a better, more accurate and consistent result. For my projects, I need more
than one ms5611, but having them on the same SPI bus again means that the bus activity
causes the sensor to behave erratically. So, I needed an SPI driver that could operate
on any set of pins. Here it is.

The library is written to be as asynchronous as possible. The MS5611 requires a 8500us
delay between requesting a "conversion" of data (the process of reading the analog
state and storing it in the sensor's memory), and the retrieval of that result. I wait
9000us to ensure that a fast arduino clock doesn't cause the lib to jump the gun. Each
update to temp/pressure requires two of these reads. The library has an internal
series of states it passes through with calls to tick():

1. Request conversion of pressure
1. (After ensuring that 9000us have passed)
   1. Retrieve raw pressure data from sensor
   1. Request conversion of temperature
1. (After ensuring that 9000us have passed)
   1. Retrieve raw temperature data from sensor
   1. Reset state so next call to tick goes back to the beginning.
   1. Return true to indicate new data available.

If you're calling tick() frequently because you need up-to-the-instant data, I suggest
that you call tick() again, immediately after receiving a true result so the pressure
conversion starts without further delay. I didn't want to build this into the library
because users who only query the sensor infrequently would potentially get stale
pressure data. (If the library requested conversion of pressure immediately after
updating its internal computations, but the user didn't need another update for an hour,
that update would be getting the pressure data from the previous hour.)

**A word of warning:** While it is possible to run multiple devices on the same SPI channel,
you will get inaccurate results if you are driving the SPI bus while a conversion is under
way. The modulation of those pins causes voltage fluctuations that, while minor, are enough
to gravely impact accuracy. If you need data in a constant, fast stream, either separate
your sensors onto two different SPI busses (4 pins for each), or call tick() on one of them
until it returns true, then tick() the other. (In this case, you'll just need the three SPI
pins, plus 1 "chip select" pin for each sensor.)
