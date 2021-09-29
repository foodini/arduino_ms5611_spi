#include "arduino.h"
#include "arduino_ms5611_spi.h"

// TODO:
// * Do the CRC check.

KalmanFilter::KalmanFilter(float measurement_error, float estimate_error) :
  m_measurement_error(measurement_error),
  m_estimate_error(estimate_error),
  m_initial_estimate_error(estimate_error),
  m_estimate(0.0f),
  m_initted(false),
  update_count(0)
{
  
}

float KalmanFilter::tick(float measurement) {
  update_count++;
  if (!m_initted) {
    m_estimate = measurement;
    m_initted = true;
  }
  float previous_estimate = m_estimate;
  float k = m_estimate_error / (m_estimate_error + m_measurement_error);
  m_estimate = previous_estimate + k * (measurement - previous_estimate);
  m_estimate_error = (1.0f - k) * m_estimate_error;
  return m_estimate;
}

void KalmanFilter::reset() {
  m_estimate_error = m_initial_estimate_error;
  update_count=0;
}

MS5611_SPI::MS5611_SPI(int csb_pin, int miso_pin, int mosi_pin, int sck_pin, int sck_delay) :
    m_initted(false),
    CSB_pin(csb_pin),
    MISO_pin(miso_pin),
    MOSI_pin(mosi_pin),
    SCK_pin(sck_pin),
    SCK_delay(sck_delay)
#if MS5611_KALMAN
    ,kalman_initted(false)
#endif
{
  pinMode(SCK_pin, OUTPUT);
  digitalWrite(SCK_pin, LOW);
  pinMode(MOSI_pin, OUTPUT);
  digitalWrite(MOSI_pin, LOW);
  pinMode(MISO_pin, INPUT);
  pinMode(CSB_pin, OUTPUT);
  digitalWrite(CSB_pin, HIGH);
}

// Output on falling, capture on rising.
void MS5611_SPI::bit_out(int bit) {
  digitalWrite(MOSI_pin, bit?HIGH:LOW);
  delayMicroseconds(SCK_delay);
  digitalWrite(SCK_pin, HIGH);
  delayMicroseconds(SCK_delay);
  digitalWrite(SCK_pin, LOW);
}

// Output on falling, capture on rising.
int MS5611_SPI::bit_in() {
  int bit;
  delayMicroseconds(SCK_delay);
  digitalWrite(SCK_pin, HIGH);
  bit = digitalRead(MISO_pin);
  delayMicroseconds(SCK_delay);
  digitalWrite(SCK_pin, LOW);
  return bit;
}

// SPI Mode 0. Most Significant Bit (MSB) first. Output on falling edge, capture on rising.
// I'm not using the arduino's built-in SPI library for a couple reasons;
// * The SPI lib assumes that you are receiving 8 bits for every 8 bits sent or 16 for 16.
//   This is simply not true for the ms5611.
// * I could possibly work around the bad 8/16 arduino lib assumption, but it would mean
//   having SCK/SCLK oscillating while conversions are being done, which causes conversion
//   noise.
// * Having 2 ms5611s on the same arduino means having them on separate busses - if I want
//   to be talking to one while the other is converting. This is, again, because I don't want
//   to be running the bus clock while a sensor is converting.
uint32_t MS5611_SPI::spi_command(byte command, unsigned int wait_time, int result_len) {
  digitalWrite(CSB_pin, LOW);
  delayMicroseconds(SCK_delay);
  digitalWrite(SCK_pin, LOW);
  for(int i=0; i<8; i++) {
    bit_out(((0x80>>i)&command)?HIGH:LOW);
  }
  digitalWrite(MOSI_pin, LOW);

  delayMicroseconds(wait_time);

  long int result = 0;
  for(int i=0; i<result_len; i++) {
    result <<= 1;
    result |= bit_in();
  }

  delayMicroseconds(SCK_delay);
  digitalWrite(CSB_pin, HIGH);
  delayMicroseconds(SCK_delay);

  return result;
}

// "reset" is the name of the action on the ms5611 datasheet. It MUST BE CALLED before
// any temp or pressure conversions, however, it takes 3500 microseconds to run and 
// MUST therefore NOT be called by the constructor. If the user tries to tick without
// calling reset(), the call will be made for them.
void MS5611_SPI::reset() {
  spi_command(0x1e, 3500, 0);
  // The required delay time is 2.8ms, but I've had issues w/ badly clocked arduinos, so
  // I'm giving myself some padding.

  FACTORY_DATA = spi_command(FACTORY_DATA_ADDR, 50, 16);
  SENS_T1 = spi_command(SENS_T1_ADDR, 50, 16);
  OFF_T1 = spi_command(OFF_T1_ADDR, 50, 16);
  TCS = spi_command(TCS_ADDR, 50, 16);
  TCO = spi_command(TCO_ADDR, 50, 16);
  T_REF = spi_command(T_REF_ADDR, 50, 16);
  TEMPSENS = spi_command(TEMPSENS_ADDR, 50, 16);
  CRC = spi_command(CRC_ADDR, 50, 16);

  m_initted = true;
}

bool MS5611_SPI::tick() {
  switch(tick_state) {
  case 0:
    if(!m_initted) {
      Serial.println("reset() has not been called. Doing so automatically.");
      reset();      
    }
    // Request conversion of pressure.
    spi_command(0x58, 0, 0);              // OSR = 4096 for best resolution
    tick_conversion_request_time = micros();
    tick_state = 1;
    break;
  case 1:
    if (time_elapsed(tick_conversion_request_time)) {
      D2 = spi_command(0x00, 0, 24);      // Get "digital pressure value"
      tick_state = 2;
      // Request conversion of temperature.
      spi_command(0x48, 0, 0);              // OSR = 4096 for best resolution
      tick_conversion_request_time = micros();
      tick_state = 2;
    }
    break;
  case 2:
    if (time_elapsed(tick_conversion_request_time)) {
      D1 = spi_command(0x00, 0, 24);      // Get "digital temperature value"    
      int64_t dT = D2 - (T_REF<<8);

      TEMP = (int32_t)(2000 + ((dT * TEMPSENS)/(1<<23)));
    
      int64_t OFF = ((int64_t)OFF_T1<<16) + (((int64_t)TCO*dT)/(1<<7));
      int64_t SENS = ((int64_t)SENS_T1<<15) + (((int64_t)TCS*dT)/(1<<8));
      P = (((D1*SENS)>>21) - OFF)/(1<<15);

      // Do the "second order temperature compensation."
      if (TEMP < 2000) {
        int64_t T2 = (dT * dT)/(1<<31);
        int64_t OFF2 = 5 - (TEMP - 2000) * (TEMP - 2000) / 2;
        int64_t SENS2 = 5 - (TEMP - 2000) * (TEMP - 2000) / 4;
        if (TEMP < -1500) {
          OFF2 = OFF2 + 7 - (TEMP + 1500) * (TEMP + 1500);
          SENS2 = SENS2 + 11 - (TEMP + 1500) * (TEMP + 1500) / 2;
        }
        TEMP -= T2;
        OFF -= OFF2;
        SENS -= SENS2;
      }

      tick_state = 0;
    }
    break;
  }

  return tick_state == 0;
}

#if MS5611_KALMAN
void MS5611_SPI::reset_kalman_filter() {
  // Doing this will leave the temp/pressure estimates where they are, effectively using them as a new
  // initial state for a new kalman sequence.
  kalman_pressure_err_estimate = 3;  // From the datasheet, error is plus-or-minus 1.5mbar.
  kalman_temp_err_estimate = 1.6;
}
#endif

float MS5611_SPI::get_temperature() {
#if MS5611_KALMAN
    return kalman_temp_estimate;
#else
    return TEMP/100.0;
#endif
}

float MS5611_SPI::get_pressure() {
#if MS5611_KALMAN
  return kalman_pressure_estimate;
#else
  return P/100.0;
#endif
}

// I make this statically available so users who use the kalman filter can get at it.
float MS5611_SPI::bar_and_temp_to_alt(float millibar, float temp) {
  return (pow(1013.25/millibar, 1.0 / 5.257) - 1.0) * (temp + 273.15) / 0.0065;
}

float MS5611_SPI::get_altitude() {
  return MS5611_SPI::bar_and_temp_to_alt(get_pressure(), get_temperature());
}

bool MS5611_SPI::time_elapsed(uint32_t last_timestamp) {
  int32_t now = micros();

  // If last_timestamp + 8400 > 2^32-1, you can't just compare last_timestamp + 8400
  // to now. But, now-last_timestamp should be the time since last_timestamp, even if 
  // now < last_timestamp - as long as we're SURE it's cast to an unsigned. (I've seen
  // the arduino return negative when subtracting uint8_t (!!!), so the cast is just
  // my current flavor of paranoia.
  if ((int32_t)(now - last_timestamp) > 8400)
    return true;
    
  return false;
}
