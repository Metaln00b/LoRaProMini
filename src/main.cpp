#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>
#include <LowPower.h>
#include <OneWire.h>
#include <EEPROM.h>
#include <CRC32.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

// ++++++++++++++++++++++++++++++++++++++++
//
// CONSTANTS
//
// ++++++++++++++++++++++++++++++++++++++++

// LoRa module pins
#define LORA_DIO0 8
#define LORA_DIO1 7
#define LORA_DIO2 LMIC_UNUSED_PIN
#define LORA_RESET 9
#define LORA_CS SS

// 1-Wire Bus
#define ONEWIREBUS 6

// External interrupt pins
#define INTERRUPT_PIN0 2
#define INTERRUPT_PIN1 3

// Battery
#define BAT_SENSE_PIN A0 // Analoge Input Pin

// BME I2C Adresses
#define I2C_ADR_BME 0x76

// Voltage calibration interval
#define VOL_DEBUG_INTERVAL 1000 // in ms

// Start address in EEPROM for structure 'cfg'
#define CFG_START 0

// Config size
#define CFG_SIZE 82
#define CFG_SIZE_WITH_CHECKSUM 86

// LORA MAX RANDOM SEND DELAY
#define LORA_MAX_RANDOM_SEND_DELAY 20

// GPS
#define GPSBaud 38400
#define GPS_RX 4
#define GPS_TX 5
#define TX_INTERVAL 120
#define GPS_FIX_RETRY_DELAY 10 // wait this many seconds when no GPS fix is received to retry
#define SHORT_TX_INTERVAL 20 // when driving, send packets every SHORT_TX_INTERVAL seconds
#define MOVING_KMPH 5.0 // if speed in km/h is higher than MOVING_HMPH, we assume that car is moving


// ++++++++++++++++++++++++++++++++++++++++
//
// LOGGING
//
// ++++++++++++++++++++++++++++++++++++++++

#ifdef LOG_DEBUG
#define log_d(s, ...) Serial.print(s, ##__VA_ARGS__);
#define log_d_ln(s, ...) Serial.println(s, ##__VA_ARGS__);
#define logHex_d(b, c) printHex(b, c);
const boolean LOG_DEBUG_ENABLED = true;
#else
#define log_d(s, ...)
#define log_d_ln(s, ...)
#define logHex_d(b, c)
const boolean LOG_DEBUG_ENABLED = false;
#endif

// ++++++++++++++++++++++++++++++++++++++++
//
// LIBS
//
// ++++++++++++++++++++++++++++++++++++++++

OneWire oneWire(ONEWIREBUS);

// ++++++++++++++++++++++++++++++++++++++++
//
// ENUMS
//
// ++++++++++++++++++++++++++++++++++++++++

enum _activationMethode
{
  OTAA = 1,
  ABP = 2
};

enum _StateByte
{
  STATE_ITR_TRIGGER = 0b0001,
  STATE_ITR0 = 0b0010,
  STATE_ITR1 = 0b0100,
};

// ++++++++++++++++++++++++++++++++++++++++
//
// VARS
//
// ++++++++++++++++++++++++++++++++++++++++

#ifdef CONFIG_MODE
const boolean CONFIG_MODE_ENABLED = true;
#else
const boolean CONFIG_MODE_ENABLED = false;
#endif

// LoRa LMIC
static osjob_t sendjob;
const lmic_pinmap lmic_pins = {
    .nss = LORA_CS,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LORA_RESET,
    .dio = {LORA_DIO0, LORA_DIO1, LORA_DIO2},
};

typedef struct
{
  uint8_t CONFIG_IS_VALID;          // 1 byte
  uint16_t SLEEPTIME;               // 2 byte - (Deep) Sleep time between data acquisition and transmission
  float BAT_SENSE_VPB;              // 4 byte - Volts per Bit. See documentation
  float BAT_MIN_VOLTAGE;            // 4 byte - Minimum voltage for operation, otherwise the node continues to sleep
  uint8_t WAKEUP_BY_INTERRUPT_PINS; // 1 byte - 0 = Disabled, 1 = Enabled
  uint8_t CONFIRMED_DATA_UP;        // 1 byte - 0 = Unconfirmed Data Up, 1 = Confirmed Data Up
  uint8_t ACTIVATION_METHOD;        // 1 byte - 1 = OTAA, 2 = ABP

  // ABP
  u1_t NWKSKEY[16]; // 16 byte - NwkSKey, network session key in big-endian format (aka msb).
  u1_t APPSKEY[16]; // 16 byte - AppSKey, application session key in big-endian format (aka msb).
  u4_t DEVADDR;     //  4 byte - DevAddr, end-device address in big-endian format (aka msb)
  // OTAA
  u1_t APPEUI[8];  //  8 byte - EUIs must be little-endian format, so least-significant-byte (aka lsb)
  u1_t DEVEUI[8];  //  8 byte - EUIs must be in little-endian format, so least-significant-byte (aka lsb)
  u1_t APPKEY[16]; // 16 byte - AppSKey, application session key in big-endian format (aka msb).

} configData_t;
configData_t cfg; // Instance 'cfg' is a global variable with 'configData_t' structure nowlong nextPacketTime;

long nextPacketTime;

unsigned long prepareCount = 0;

// These callbacks are used in over-the-air activation
void os_getArtEui(u1_t *buf)
{
  memcpy(buf, cfg.APPEUI, 8);
}
void os_getDevEui(u1_t *buf)
{
  memcpy(buf, cfg.DEVEUI, 8);
}
void os_getDevKey(u1_t *buf)
{
  memcpy(buf, cfg.APPKEY, 16);
}

// ++++++++++++++++++++++++++++++++++++++++
//
// MAIN CODE
//
// ++++++++++++++++++++++++++++++++++++++++

SoftwareSerial serial(GPS_RX, GPS_TX); // RX, TX
TinyGPSPlus gps;

void clearSerialBuffer()
{
  while (Serial.available())
  {
    Serial.read();
  }
}

float readBat()
{
  uint16_t value = 0;
  uint8_t numReadings = 5;

  for (uint8_t i = 0; i < numReadings; i++)
  {
    value = value + analogRead(BAT_SENSE_PIN);

    // 1ms pause adds more stability between reads.
    delay(1);
  }

  value = value / numReadings;

  float batteryV = value * cfg.BAT_SENSE_VPB;
  if (CONFIG_MODE_ENABLED)
  {
    Serial.print(F("Analoge voltage: "));
    Serial.print(((1.1 / 1024.0) * value), 2);
    Serial.print(F(" V | Analoge value: "));
    Serial.print(value);
    Serial.print(F(" ("));
    Serial.print(((100.0 / 1023.0) * value), 1);
    Serial.print(F("% of Range) | Battery voltage: "));
    Serial.print(batteryV, 1);
    Serial.print(F(" V ("));
    Serial.print(batteryV, 2);
    Serial.print(F(" V, VPB="));
    Serial.print(cfg.BAT_SENSE_VPB, 10);
    Serial.println(F(")"));
  }

  return batteryV;
}

void printHex(byte buffer[], size_t arraySize)
{
  unsigned c;
  for (size_t i = 0; i < arraySize; i++)
  {
    c = buffer[i];

    if (i != 0)
      Serial.write(32); // Space

    c &= 0xff;
    if (c < 16)
      Serial.write(48); // 0
    Serial.print(c, HEX);
  }
}

void float2Bytes(float val, byte* bytes_array)
{
  // Create union of shared memory space
  union {
    float float_variable;
    byte temp_array[4];
  } u;
  // Overite bytes of union with float variable
  u.float_variable = val;
  // Assign bytes to input array
  memcpy(bytes_array, u.temp_array, 4);
}

void readConfig()
{
  EEPROM.get(CFG_START, cfg);
}

void setConfig()
{
  byte cfgbuffer[CFG_SIZE_WITH_CHECKSUM];
  char buffer[3];
  char *pEnd;
  uint32_t crc_data = 0;
  CRC32 crc;

  for (uint8_t i = 0; i < CFG_SIZE_WITH_CHECKSUM; i++)
  {
    if (!Serial.readBytes(buffer, 2))
    {
      break;
    }

    // added null-terminator for strtol
    buffer[3] = '\0';

    // convert null-terminated char buffer with hex values to int
    cfgbuffer[i] = (byte)strtol(buffer, &pEnd, 16);

    // Add bytes to crc calculation
    if (i < CFG_SIZE)
    {
      // Add hex chars from buffer without null-terminator
      crc.update(buffer, 2);
    }
    else
    {
      // Store checksum from serial input
      // later compare
      crc_data <<= 8;
      crc_data |= cfgbuffer[i];
    }

    // Pad to 8 bit
    // Add zero prefix to serial output
    cfgbuffer[i] &= 0xff;
    if (cfgbuffer[i] < 16)
      Serial.write(48); // 0

    Serial.print(cfgbuffer[i], HEX);
    Serial.print(" ");
  }

  Serial.println();

  Serial.print(F("> CHECKSUM (Serial Input): 0x"));
  Serial.print(crc_data, HEX);
  Serial.println();

  uint32_t crc_calculated = crc.finalize();
  Serial.print(F("> CHECKSUM (Calculated):   0x"));
  Serial.println(crc_calculated, HEX);

  if (crc_data == crc_calculated)
  {

    Serial.println(F("> Checksum correct. Configuration saved!"));

    for (uint8_t i = CFG_START; i < CFG_SIZE; i++)
    {
      EEPROM.write(i, (uint8_t)cfgbuffer[i]);
    }

    readConfig();
  }
  else
  {
    Serial.println(F("> INVALID CHECKSUM! Process aborted! "));
  }
}

void showConfig(bool raw = false)
{
  Serial.print(F("> CONFIG_IS_VALID: "));
  if (cfg.CONFIG_IS_VALID == 1)
  {
    Serial.println(F("yes"));
  }
  else
  {
    Serial.println(F("no"));
  }
  Serial.print(F("> SLEEPTIME: "));
  Serial.println(cfg.SLEEPTIME, DEC);
  Serial.print(F("> BAT_SENSE_VPB: "));
  Serial.println(cfg.BAT_SENSE_VPB, DEC);
  Serial.print(F("> BAT_MIN_VOLTAGE: "));
  Serial.println(cfg.BAT_MIN_VOLTAGE, DEC);
  Serial.print(F("> WAKEUP_BY_INTERRUPT_PINS: "));
  switch (cfg.WAKEUP_BY_INTERRUPT_PINS)
  {
  case 0:
    Serial.println(F("Disabled"));
    break;
  case 1:
    Serial.println(F("Enabled"));
    break;
  default:
    Serial.println(F("Unkown"));
    break;
  }
  Serial.print(F("> CONFIRMED_DATA_UP: "));
  switch (cfg.CONFIRMED_DATA_UP)
  {
  case 0:
    Serial.println(F("Unconfirmed Data Up"));
    break;
  case 1:
    Serial.println(F("Confirmed Data Up"));
    break;
  default:
    Serial.println(F("Unkown"));
    break;
  }
  Serial.print(F("> ACTIVATION_METHOD: "));
  switch (cfg.ACTIVATION_METHOD)
  {
  case 1:
    Serial.println(F("OTAA"));
    break;
  case 2:
    Serial.println(F("ABP"));
    break;
  default:
    Serial.println(F("Unkown"));
    break;
  }
  Serial.print(F("> NWKSKEY (MSB): "));
  printHex(cfg.NWKSKEY, sizeof(cfg.NWKSKEY));
  Serial.print(F("\n> APPSKEY (MSB): "));
  printHex(cfg.APPSKEY, sizeof(cfg.APPSKEY));
  Serial.print(F("\n> DEVADDR (MSB): "));
  Serial.print(cfg.DEVADDR, HEX);
  Serial.print(F("\n> APPEUI (LSB): "));
  printHex(cfg.APPEUI, sizeof(cfg.APPEUI));
  Serial.print(F("\n> DEVEUI (LSB): "));
  printHex(cfg.DEVEUI, sizeof(cfg.DEVEUI));
  Serial.print(F("\n> APPKEY (MSB): "));
  printHex(cfg.APPKEY, sizeof(cfg.APPKEY));
  Serial.println();

  if (raw)
  {
    Serial.print(F("> RAW ("));
    Serial.print((uint32_t)sizeof(cfg), DEC);
    Serial.print(F(" bytes): "));
    byte c;
    for (uint8_t i = CFG_START; i < sizeof(cfg); i++)
    {
      c = EEPROM.read(i);
      c &= 0xff;
      if (c < 16)
        Serial.write(48); // 0
      Serial.print(c, HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
}

void eraseConfig()
{
  for (uint8_t i = CFG_START; i < sizeof(cfg); i++)
  {
    EEPROM.write(i, 0);
  }
}

void serialWait()
{
  while (!Serial.available())
  {
    yield();
    delay(50);
  }
}

void serialMenu()
{
  unsigned long timer = millis();

  Serial.println(F("\n== CONFIG MENU =="));
  Serial.println(F("[1] Show current config"));
  Serial.println(F("[2] Set new config"));
  Serial.println(F("[3] Erase current config"));
  Serial.println(F("[4] Voltage calibration"));
  Serial.print(F("Select: "));
  serialWait();

  if (Serial.available() > 0)
  {
    switch (Serial.read())
    {
    case '1':
      Serial.println(F("\n\n== SHOW CONFIG =="));
      showConfig(true);
      break;

    case '2':
      Serial.println(F("\n\n== SET CONFIG =="));
      Serial.print(F("Past new config: "));
      serialWait();
      if (Serial.available() > 0)
      {
        setConfig();
      }
      break;

    case '3':
      Serial.println(F("\n\n== ERASE CONFIG =="));
      Serial.print(F("Are you sure you want to erase the EEPROM? [y]es or [n]o: "));
      serialWait();
      Serial.println();
      if (Serial.available() > 0)
      {
        if (Serial.read() == 'y')
        {
          eraseConfig();
          Serial.println(F("Successfull!"));
          readConfig();
        }
        else
        {
          Serial.println(F("Aborted!"));
        }
      }
      break;

    case '4':
      Serial.println(F("\n\n== VOLTAGE CALIBRATION =="));
      while (!Serial.available())
      {
        if (millis() - timer > VOL_DEBUG_INTERVAL)
        {
          readBat();
          timer = millis();
        }
      }

      break;
    }
  }
  clearSerialBuffer();
}

void do_send(osjob_t *j)
{
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND)
  {
    // Serial.println(F("OP_TXRXPEND, not sending"));
  }
  else
  {
    // Unsigned 16 bits integer, 0 up to 65,535
    uint16_t bat = 0;

    // Battery
    bat = readBat() * 100;

    unsigned long previousMillis = millis();

    while((previousMillis + 1000) > millis())
    {
        while (serial.available() )
        {
            char data = serial.read();
            gps.encode(data);
        }
    }

    // if (gps.location.isValid() && 
    //   gps.location.age() < 2000 &&
    //   gps.hdop.isValid() &&
    //   gps.hdop.value() <= 300 &&
    //   gps.hdop.age() < 2000 &&
    //   gps.altitude.isValid() && 
    //   gps.altitude.age() < 2000 )
    // 00 01 52 26    82 7F 4A 42    6E 5A F5 40   00 00 00 00    41 B7 97 3D    00 00 00 00
    if (gps.location.isValid())
    {
      double lat = gps.location.lat();
      double lon = gps.location.lng();
      double alt = gps.altitude.meters();
      double kmph = gps.speed.kmph();
      uint32_t sats = gps.satellites.value();

      byte buffer[24];
      buffer[0] = 0x00;
      buffer[1] = bat >> 8;
      buffer[2] = bat;
      buffer[3] = (VERSION_MAJOR << 4) | (VERSION_MINOR & 0xf);

      float2Bytes(lat, &buffer[4]); // 4-7
      float2Bytes(lon, &buffer[8]); // 8-11
      float2Bytes(alt, &buffer[12]); // 12-15
      float2Bytes(kmph, &buffer[16]); // 16-19
      buffer[20] = sats >> 24;
      buffer[21] = sats >> 16;
      buffer[22] = sats >> 8;
      buffer[23] = sats;

      log_d("Prepare pck #");
      log_d_ln(++prepareCount);
      // log_d(F("> FW: v"));
      // log_d(VERSION_MAJOR);
      // log_d(F("."));
      // log_d(VERSION_MINOR);
      // log_d(F("> Batt: "));
      // log_d_ln(bat);
      // log_d(F("> Pins: "));
      // log_d_ln(buffer[0], BIN);
      // log_d(F("> BME Temp: "));
      // log_d_ln(temp1);
      // log_d(F("> BME Humi: "));
      // log_d_ln(humi1);
      // log_d(F("> BME Pres: "));
      // log_d_ln(press1);
      // log_d(F("> DS18x Temp: "));
      // log_d_ln(temp2);
      log_d(F("> Payload: "));
      logHex_d(buffer, sizeof(buffer));
      log_d_ln();

      // Prepare upstream data transmission at the next possible time.
      LMIC_setTxData2(1, buffer, sizeof(buffer), cfg.CONFIRMED_DATA_UP);
      log_d_ln(F("Pck queued"));
    }
    else
    {
      log_d_ln(F("No GPS fix"));
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(GPS_FIX_RETRY_DELAY), do_send);
    }
  }
}

void lmicStartup()
{
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  if (cfg.ACTIVATION_METHOD == ABP)
  {
    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    LMIC_setSession(0x13, cfg.DEVADDR, cfg.NWKSKEY, cfg.APPSKEY);

#if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set. The LMIC doesn't let you change
    // the three basic settings, but we show them here.
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI); // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK, DR_FSK), BAND_MILLI);   // g2-band
// TTN defines an additional channel at 869.525Mhz using SF9 for class B
// devices' ping slots. LMIC does not have an easy way to define set this
// frequency and support for class B is spotty and untested, so this
// frequency is not configured here.
#elif defined(CFG_us915) || defined(CFG_au915)
    // NA-US and AU channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
#elif defined(CFG_as923)
// Set up the channels used in your country. Only two are defined by default,
// and they cannot be changed.  Use BAND_CENTI to indicate 1% duty cycle.
// LMIC_setupChannel(0, 923200000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
// LMIC_setupChannel(1, 923400000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);

// ... extra definitions for channels 2..n here
#elif defined(CFG_kr920)
// Set up the channels used in your country. Three are defined by default,
// and they cannot be changed. Duty cycle doesn't matter, but is conventionally
// BAND_MILLI.
// LMIC_setupChannel(0, 922100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
// LMIC_setupChannel(1, 922300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
// LMIC_setupChannel(2, 922500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);

// ... extra definitions for channels 3..n here.
#elif defined(CFG_in866)
// Set up the channels used in your country. Three are defined by default,
// and they cannot be changed. Duty cycle doesn't matter, but is conventionally
// BAND_MILLI.
// LMIC_setupChannel(0, 865062500, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
// LMIC_setupChannel(1, 865402500, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
// LMIC_setupChannel(2, 865985000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);

// ... extra definitions for channels 3..n here.
#else
#error Region not supported
#endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);
    // Disable ADR
    LMIC_setAdrMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7, 14);

    // #else
  }
  else
  {
    LMIC_setClockError(MAX_CLOCK_ERROR * 5 / 100); // fix OTAA joining for Arduino Pro Mini (https://github.com/matthijskooijman/arduino-lmic#problems-with-downlink-and-otaa)
  }
  // #endif
}

void onEvent(ev_t ev)
{
  switch (ev)
  {
    case EV_JOINING:
      log_d_ln(F("Joining..."));
      break;

    case EV_JOINED:
      log_d_ln(F("Joined!"));

      if (cfg.ACTIVATION_METHOD == OTAA)
      {
        //   u4_t netid = 0;
        //   devaddr_t devaddr = 0;
        //   u1_t nwkKey[16];
        //   u1_t artKey[16];
        //   LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
        //   log_d(F("> NetID: "));
        //   log_d_ln(netid, DEC);
        //   log_d(F("> DevAddr: ")); // (MSB)
        //   log_d_ln(devaddr, HEX);
        //   log_d(F("> AppSKey: ")); // (MSB)
        //   logHex_d(artKey, sizeof(artKey));
        //   log_d_ln();
        //   log_d(F("> NwkSKey: ")); // (MSB)
        //   logHex_d(nwkKey, sizeof(nwkKey));
        //   log_d_ln();
        log_d_ln();

        // Enable ADR explicit (default is alreay enabled)
        LMIC_setAdrMode(1);
        // enable link check validation
        LMIC_setLinkCheckMode(1);

        // Ok send our first data in 10 ms
        os_setTimedCallback(&sendjob, os_getTime() + ms2osticks(10), do_send);
      }
      break;

    case EV_JOIN_FAILED:
      log_d_ln(F("Join failed"));
      lmicStartup(); // Reset LMIC and retry
      break;

    case EV_REJOIN_FAILED:
      log_d_ln(F("Rejoin failed"));
      lmicStartup(); // Reset LMIC and retry
      break;

    case EV_TXSTART:
      // log_d_ln(F("EV_TXSTART"));
      break;

    case EV_TXCOMPLETE:
      log_d(F("TX done #")); // (includes waiting for RX windows)
      log_d_ln(LMIC.seqnoUp);
      if (LMIC.txrxFlags & TXRX_ACK)
      {
        log_d_ln(F("> Got ack"));
      }
      if (LMIC.txrxFlags & TXRX_NACK)
      {
        log_d_ln(F("> Got NO ack"));
      }
      nextPacketTime = (gps.speed.kmph() > MOVING_KMPH ? SHORT_TX_INTERVAL : TX_INTERVAL); // depend on current GPS speed
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(nextPacketTime), do_send);
      log_d(F("Next in "));
      log_d(nextPacketTime);
      log_d_ln(F("s"));
      break;

    case EV_JOIN_TXCOMPLETE:
      log_d_ln(F("NO JoinAccept"));
      break;

    case EV_TXCANCELED:
      log_d_ln(F("TX canceled!"));
      break;

    case EV_BEACON_FOUND:
    case EV_BEACON_MISSED:
    case EV_BEACON_TRACKED:
    case EV_RFU1:
    case EV_LOST_TSYNC:
    case EV_RESET:
    case EV_RXCOMPLETE:
    case EV_LINK_DEAD:
    case EV_LINK_ALIVE:
    case EV_SCAN_FOUND:

    case EV_RXSTART:
    default:
      log_d(F("Unknown Evt: "));
      log_d_ln((unsigned)ev);
      break;
  }
}

void setup()
{
  // use the 1.1 V internal reference
  analogReference(INTERNAL);

  if (LOG_DEBUG_ENABLED)
  {
    while (!Serial)
    {
      ; // wait for Serial to be initialized
    }
    Serial.begin(9600);
    delay(100); // per sample code on RF_95 test
  }
  
  serial.begin(GPSBaud);

  log_d(F("\n= Starting LoRaProMini v"));
  log_d(VERSION_MAJOR);
  log_d(F("."));
  log_d(VERSION_MINOR);
  log_d_ln(F(" ="));

  readConfig();

  if (CONFIG_MODE_ENABLED)
  {
    Serial.println(F("CONFIG MODE ENABLED!"));
    Serial.println(F("LORA DISABLED!"));
  }
  else
  {
    if (!cfg.CONFIG_IS_VALID)
    {
      log_d_ln(F("INVALID CONFIG!"));
      while (true)
      {
      }
    }
  }

  // Start LoRa stuff if not in config mode
  if (!CONFIG_MODE_ENABLED)
  {
    // LMIC init
    os_init();

    // Reset the MAC state. Session and pending data transfers will be discarded.
    lmicStartup();

    Serial.print(F("Join mode "));

    // ABP Mode
    if (cfg.ACTIVATION_METHOD == ABP)
    {
      Serial.println(F("ABP"));
      // Start job in ABP Mode
      do_send(&sendjob);
    }

    // OTAA Mode
    else if (cfg.ACTIVATION_METHOD == OTAA)
    {
      Serial.println(F("OTAA"));
      // Start job (sending automatically starts OTAA too)
      // Join the network, sending will be started after the event "Joined"
      LMIC_startJoining();
    }
  }
}

void loop()
{
  if (CONFIG_MODE_ENABLED)
  {
    serialMenu();
  }
  else
  {
    os_runloop_once();
  }
}
