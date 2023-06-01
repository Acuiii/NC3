#include "STM32LowPower.h"
#include <STM32RTC.h>
#include <IWatchdog.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <PCD8544.h>

#define LED PA15
//#define RX1 PA10
//#define TX1 PA9
#define RX2 PA3
#define TX2 PA2
#define RX3 PB11
#define TX3 PB10
#define loraNSS PA4
#define loraRST PB0
#define loraDIO0 PB14
#define loraDIO1 PB13
#define loraDIO2 PB12
#define BATPIN PA1
#define displayRST PB9
#define displayCE PB8
#define displayDC PB7
#define displayDIN PB6
#define displayCLK PB5
#define displayBL PB4
#define GATEL1 PB15
#define GATEL2 PB1
#define GATEL3 PB2
#define DIP0 PA11
#define DIP1 PA12
#define PROG_MEASURE 0
#define PROG_TX_WAITING 1
#define PROG_TX_DONE 2
#define PROG_SLEEP 3
#define PROG_JOIN_FAILED 4
#define PROG_REINIT 5
#define MODE_ONE_MINT 0
#define MODE_FIFTN_MINT 1
#define MODE_ONE_HOUR 2
#define MODE_HALF_MINT 3
#define LORA_DATA_SIZE 21

STM32RTC &rtc = STM32RTC::getInstance();
//HardwareSerial Serial(RX1, TX1);
HardwareSerial Serial2(RX2, TX2); // If choosing generic f103, then Serial2 by default is on Serial, So will conflict if declare here
HardwareSerial Serial3(RX3, TX3);

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8] = {0x86, 0x34, 0xFC, 0xD5, 0xDB, 0xAB, 0xA6, 0x14};
void os_getArtEui(u1_t *buf) {
  memcpy_P(buf, APPEUI, 8);
}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8] = {0x24, 0xF1, 0xF6, 0xAE, 0xC9, 0x5F, 0x5F, 0xAC};
void os_getDevEui(u1_t *buf) {
  memcpy_P(buf, DEVEUI, 8);
}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t PROGMEM APPKEY[16] = {0xD8, 0xD2, 0x2F, 0x30, 0x8C, 0x7D, 0x84, 0x60, 0xBC, 0x5E, 0xF2, 0xD8, 0x56, 0x23, 0x66, 0x05};
void os_getDevKey(u1_t *buf) {
  memcpy_P(buf, APPKEY, 16);
}

static osjob_t sendjob;

// Pin mapping
// Adapted for Feather M0 per p.10 of [feather]
const lmic_pinmap lmic_pins = {
  .nss = loraNSS, // chip select on feather (rf95module) CS
  .rxtx = LMIC_UNUSED_PIN,
  .rst = loraRST, // reset pin
  .dio = {loraDIO0, loraDIO1, loraDIO2},
};

//Adafruit_PCD8544 display = Adafruit_PCD8544(displayCLK, displayDIN, displayDC, displayCE, displayRST);
static PCD8544 lcd(displayCLK, displayDIN, displayDC, displayRST, displayCE);

static uint8_t programming = PROG_TX_WAITING;
static uint8_t run_mode = MODE_ONE_MINT;
static bool event_received_flag = false;
static uint32_t time_passed = 0;

static uint16_t WAKEUP_INT = 59;
const String D_ID = "Z-T:";

const uint8_t seconds = 0;
const uint8_t minutes = 40;
const uint8_t hours = 21;
const uint8_t days = 24;
const uint8_t months = 9;
const uint8_t years = 21; // in byte size

static const float BATT_OFFSET = 0.33;

//const uint8_t LORA_DATA_SIZE = 22;
// currently max size receive by TTS somehow cap at 20 only
static uint8_t lora_data[LORA_DATA_SIZE];

void onEvent(ev_t ev)
{
  lcd.clear();
  lcd.setCursor(0, 1);

  //    strcpy(event_msg, "");
  //    Serial.print(os_getTime());
  //    Serial.print(": ");
  switch (ev)
  {
    case EV_SCAN_TIMEOUT:
      //        strcat(event_msg, "Scan timeout");
      // strcpy(event_msg, "Scan timeout");
      lcd.print("Scan timeout");
      //        Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      lcd.print("Beacon found");
      //        Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      lcd.print("Beacon missed");
      //        Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      lcd.print("Beacon tracked");
      //        Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      lcd.print("Joining");
      //        Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      {
        lcd.print("Joined");
        //        Serial.println(F("EV_JOINED"));

        //https://github.com/mcci-catena/arduino-lmic/blob/master/examples/helium-otaa/helium-otaa.ino
        //LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
        // Disable link check validation (automatically enabled
        // during join, but not supported by TTN at this time).
        //        LMIC_setLinkCheckMode(0);
        // Disable link check validation (automatically enabled
        // during join, but because slow data rates change max TX
        // size, we don't use it in this example.
        LMIC_setLinkCheckMode(0);
        programming = PROG_MEASURE;
      }
      break;
    case EV_RFU1:
      lcd.print("RFU1");
      //        Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
    {
      lcd.print("Join failed");
      //        Serial.println(F("EV_JOIN_FAILED"));
      programming = PROG_JOIN_FAILED;
    }
      break;
    case EV_REJOIN_FAILED:
    {
      lcd.print("Rejoin failed");
      //        Serial.println(F("EV_REJOIN_FAILED"));
      programming = PROG_JOIN_FAILED;
    } 
      break;
    case EV_TXCOMPLETE:
      {
        lcd.print("TX complete");
        //        Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
        if (LMIC.txrxFlags & TXRX_ACK)
        {
          lcd.setCursor(0, 0);
          lcd.print("Received ack");
          //            Serial.println(F("Received ack"));
        }
        if (LMIC.dataLen != 0 || LMIC.dataBeg != 0)
        {
          // Data was received. Extract port number if any
          u1_t bPort = 0;
          if (LMIC.txrxFlags & TXRX_PORT)
            bPort = LMIC.frame[LMIC.dataBeg - 1];
          event_received_flag = true;
          //            Serial.println(F("Received "));
          //            Serial.println(LMIC.dataLen);
          //            Serial.println(F(" bytes of payload"));
          // handle port #, pMessage, nMessage
          // nMessage might be zero
          // bPort, LMIC.frame + LMIC.dataBeg, LMIC.datalen
        }
        // Schedule next transmission
        programming = PROG_TX_DONE;
      }
      break;
    case EV_LOST_TSYNC:
      lcd.print("Lost Tsync");
      //        Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      lcd.print("Reset");
      //        Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      lcd.print("Rx complete");
      //        Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      lcd.print("Link dead");
      //        Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      lcd.print("Link alive");
      //        Serial.println(F("EV_LINK_ALIVE"));
      break;
    default:
      lcd.print("Waiting");
      //        Serial.println(F("Unknown event"));
      break;
  }
}

void initfunc(osjob_t *j)
{
  //reset MAC state
  LMIC_reset();
  
  LMIC_setAdrMode(0);
  LMIC_setLinkCheckMode(0);
  LMIC_setClockError(MAX_CLOCK_ERROR * 90 / 100);
  LMIC.dn2Dr = DR_SF12;
  
  // start joining
  LMIC_startJoining();
  // init done - onEvent() callback will be invoked...
}

void do_send(osjob_t *j)
{
  lcd.clear();
  lcd.setCursor(0, 0);

  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND)
  {
    //        Serial.println(F("OP_TXRXPEND, not sending"));
    lcd.print("OP TXRX pend");
  }
  else
  {
    // Prepare upstream data transmission at the next possible time.
    lcd.print("Packet queued");
    LMIC_setTxData2(1, lora_data, sizeof(lora_data), 0);

    //        Serial.println(F("Packet queued"));
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void do_send_ack(osjob_t *j)
{

  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND)
  {
    //        Serial.println(F("OP_TXRXPEND, not sending"));
    lcd.print("OP TXRX pend");
  }
  else
  {
    // Prepare upstream data transmission at the next possible time.
    static uint8_t ack[1] = {0};
    LMIC_setTxData2(1, ack, sizeof(ack), 0);
  }
}

void alarmMatch(void *data)
{
  next_wakeup();
}

float batt_measure(uint16_t a)
{
  float r1 = 30000.00;
  float r2 = 10000.00;

  // let analog_voltage_measured = (a*3.3)/1023.0; // 2^10
  float analog_voltage_measured = (float(a) * 3.3 ) / 4096.0; // 2^12
  // float analog_voltage_measured = (float(a) * 3.3) / 1024.0; // 2^10 // (NOT 1023)

  return (((analog_voltage_measured * (r1 + r2)) / r2) - BATT_OFFSET);
}

void laser2lora(uint8_t laserbuffer[15], uint8_t loradataindex)
{
  uint16_t laser_lora = 0;
  if (laserbuffer[0] == 'M')
  {
    if (laserbuffer[2] == 'E')
    {
      lora_data[loradataindex + 3] = (((laserbuffer[4] - 48) * 10) + laserbuffer[5] - 48);
    }
    else
    {
      uint16_t five = 0;
      uint16_t four = (laserbuffer[3] - 48) * 1000;
      uint16_t three = (laserbuffer[5] - 48) * 100;
      uint16_t two = (laserbuffer[6] - 48) * 10;
      uint16_t one = (laserbuffer[7] - 48);
      if ((laserbuffer[2] > 47) && (laserbuffer[2] < 58))
      {
        five = (laserbuffer[2] - 48) * 10000;
      }
      laser_lora = five + four + three + two + one;
      lora_data[loradataindex + 0] = laser_lora >> 8; // Laser measurement MSB
      lora_data[loradataindex + 1] = laser_lora;      // Laser measurement LSB

      four = (laserbuffer[10] - 48) * 1000;
      three = (laserbuffer[11] - 48) * 100;
      two = (laserbuffer[12] - 48) * 10;
      one = (laserbuffer[13] - 48);
      laser_lora = four + three + two + one;
      lora_data[loradataindex + 2] = laser_lora;
      //            lora_data[loradataindex + 3] = laser_lora;
    }
  }
}

void next_wakeup() {
  switch (run_mode)
  {
    case MODE_ONE_MINT:
      {
        rtc.setAlarmEpoch(rtc.getEpoch() + 59);
      }
      break;
    case MODE_FIFTN_MINT:
      {
        int16_t get_mint = rtc.getMinutes();
        if (get_mint < 30) {
          if (get_mint < 15) {
            get_mint = 15 - get_mint;
          } else {
            get_mint = 30 - get_mint;
          }
        } else {
          if (get_mint < 45) {
            get_mint = 45 - get_mint;
          } else {
            get_mint = 60 - get_mint;
          }
        }
        get_mint = abs(get_mint);
        uint32_t wakeup_seconds = get_mint * 60;
        rtc.setAlarmEpoch(rtc.getEpoch() + wakeup_seconds);
      }
      break;
    case MODE_ONE_HOUR:
      {
        int16_t get_mint = rtc.getMinutes();
        get_mint = 60 - get_mint;
        get_mint = abs(get_mint);
        uint32_t wakeup_seconds = get_mint * 60;
        rtc.setAlarmEpoch(rtc.getEpoch() + wakeup_seconds);
      }
      break;
    case MODE_HALF_MINT:
      {
        rtc.setAlarmEpoch(rtc.getEpoch() + 29);
      }
      break;
    default:
      {
        rtc.setAlarmEpoch(rtc.getEpoch() + 59);
      }
      break;
  }
}

void downlink_handler(uint8_t cur_hours, uint8_t cur_mins) {

  if (LMIC.dataLen == 0) {
    programming = PROG_TX_WAITING;
    os_setCallback(&sendjob, do_send_ack);

  } else if (LMIC.dataLen > 1) {
    programming = PROG_SLEEP;
    time_passed = rtc.getEpoch() - time_passed;
    lcd.setCursor(0, 5);
    lcd.print("DL:");
    lcd.print(run_mode);
    lcd.print(":");
    lcd.print(time_passed);

    if (LMIC.frame[LMIC.dataBeg] == 'H' && LMIC.frame[LMIC.dataBeg + 1] == 'T' && LMIC.dataLen > 3) {
      uint8_t nhours = 0;
      uint8_t nminutes = 0;
      uint8_t nseconds = rtc.getSeconds();

      nhours = cur_hours + LMIC.frame[LMIC.dataBeg + 2];
      while (nhours > 23) {
        nhours -= 24;
      }

      nminutes = cur_mins + LMIC.frame[LMIC.dataBeg + 3];
      while (nminutes > 59) {
        nminutes -= 60;
      }

      rtc.setTime(nhours, nminutes, nseconds);

      next_wakeup();

      //              for (int i = 2; i < LMIC.dataLen; i++)
      //              {
      //                  lcd.print(LMIC.frame[LMIC.dataBeg + i]);
      //              }

    } else if (LMIC.frame[LMIC.dataBeg] == 'C' && LMIC.frame[LMIC.dataBeg + 1] == 'M' && LMIC.dataLen > 2) {
      if (LMIC.frame[LMIC.dataBeg + 2] >= 0 && LMIC.frame[LMIC.dataBeg + 2] < 4) {
        run_mode = LMIC.frame[LMIC.dataBeg + 2];
        
        next_wakeup();
      } else {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Unknown cmd");
        lcd.setCursor(0, 1);
        lcd.print("Run Mode: ");
        lcd.print(LMIC.frame[LMIC.dataBeg + 2]);
        delay(10000);
      }
    } else if (LMIC.frame[LMIC.dataBeg] == 'H' && LMIC.frame[LMIC.dataBeg + 1] == 'K') {
      lcd.clear();
      lcd.setCursor(0, 1);
      lcd.print("Rebooting...");
      delay(1000);
      lcd.setCursor(0, 2);
      lcd.print(".....");
      delay(1000);
      lcd.setCursor(0, 3);
      lcd.print("...");
      IWatchdog.begin(1000000);
      delay(3000);
    }
  } else {
    programming = PROG_SLEEP;
    time_passed = rtc.getEpoch() - time_passed;
    lcd.setCursor(0, 5);
    lcd.print("DL:");
    lcd.print(run_mode);
    lcd.print(":");
    lcd.print(time_passed);
  }
  //  lcd.print(":");
}

void setup()
{
  rtc.setClockSource(STM32RTC::LSE_CLOCK);
  rtc.begin();

  lcd.begin(84, 48);
  //    lcd.setContrast(40);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Initialzing...");

  analogReadResolution(12);

  pinMode(LED, OUTPUT);
  pinMode(BATPIN, INPUT);
  pinMode(displayBL, OUTPUT);

  digitalWrite(GATEL1, HIGH); // set OFF Laser1 first
  pinMode(GATEL1, OUTPUT);    // Always change to OUTPUT the last step!
  digitalWrite(GATEL2, HIGH); // set OFF Laser2 first
  pinMode(GATEL2, OUTPUT);    // Always change to OUTPUT the last step!
  digitalWrite(GATEL3, HIGH); // set OFF Laser3 first
  pinMode(GATEL3, OUTPUT);    // Always change to OUTPUT the last step!

  pinMode(DIP0, INPUT_PULLDOWN);
  pinMode(DIP1, INPUT_PULLDOWN);

  digitalWrite(displayBL, HIGH); // OFF BACKLIGHT

  int dip0 = digitalRead(DIP0);
  int dip1 = digitalRead(DIP1);
  String wkup_mode = "1 min mode";

  if (dip1 == LOW && dip0 == LOW)
  {
    WAKEUP_INT = 59;
    wkup_mode = "1 min mode";
    run_mode = MODE_ONE_MINT;
  }
  else if (dip1 == LOW && dip0 == HIGH)
  {
    WAKEUP_INT = 899;
    wkup_mode = "15 mins mode";
    run_mode = MODE_FIFTN_MINT;
  }
  else if (dip1 == HIGH && dip0 == LOW)
  {
    WAKEUP_INT = 3599;
    wkup_mode = "1 hour mode";
    run_mode = MODE_ONE_HOUR;
  }
  else if (dip1 == HIGH && dip0 == HIGH)
  {
    WAKEUP_INT = 29;
    wkup_mode = "30 sec mode";
    run_mode = MODE_HALF_MINT;
  }

  lcd.setCursor(0, 1);
  lcd.print(wkup_mode);

  if (!rtc.isTimeSet())
  {
    //    Serial.println("Time did not persist.");
    lcd.setCursor(0, 2);
    lcd.print("Time reset");
    rtc.setTime(hours, minutes, seconds);
    rtc.setDate(days, months, years);
  }
  else
  {
    //    Serial.println("Time persisted!");
    lcd.setCursor(0, 2);
    lcd.print("Time recover");
    lcd.setCursor(0, 3);
    lcd.print(rtc.getHours());
    lcd.print("/");
    lcd.print(rtc.getMinutes());
    lcd.print("/");
    lcd.print(rtc.getSeconds());
  }

  //  while (true) {
  //    delay(10000);
  //  }

  // DEBUG delay
  bool init_blink = true;
  for (int i = 0; i < 10; i++)
  {
    if (init_blink)
    {
      init_blink = !init_blink;
      digitalWrite(LED, LOW); // OFF LED
    }
    else
    {
      init_blink = !init_blink;
      digitalWrite(LED, HIGH); // ON LED
    }
    delay(500);
  }

  while (!Serial)
    ;
  Serial.begin(19200);
  while (!Serial2)
    ;
  Serial2.begin(19200);
  while (!Serial3)
    ;
  Serial3.begin(19200);

  // LMIC init
  os_init();

  // setup initial job
  // will be executed when os_runloop_once() is called
  programming = PROG_TX_WAITING;
  os_setCallback(&sendjob, initfunc);

  // Reset the MAC state. Session and pending data transfers will be discarded.
  //    LMIC_reset();

  //    LMIC_setLinkCheckMode(0);
  //    LMIC_setAdrMode(1);

  // Configure low power
  LowPower.begin();
  LowPower.enableWakeupFrom(&rtc, alarmMatch, &WAKEUP_INT);
  rtc.setAlarmEpoch(rtc.getEpoch() + WAKEUP_INT);
}

void loop()
{
  static uint32_t counter = 1;
  //    static uint32_t ndl_count = 0;
  static uint8_t bufferSerial1[15];
  static uint8_t bufferSerial2[15];
  static uint8_t bufferSerial3[15];
  static float bat = 0.0;
  static uint8_t cur_hours = 0;
  static uint8_t cur_mins = 0;

  //    while(true) {
  //      lcd.clear();
  //      lcd.setCursor(0, 0);
  //      lcd.print(LMIC_MAX_FRAME_LENGTH);
  //      lcd.setCursor(0, 1);
  //      lcd.print(MAX_LEN_PAYLOAD);
  //      delay(10000);
  //    }

  switch (programming)
  {
    case PROG_MEASURE:
      {
        digitalWrite(LED, HIGH); // ON LED

        LMIC_setAdrMode(0);
        LMIC_setLinkCheckMode(0);
        LMIC_setClockError(MAX_CLOCK_ERROR * 90 / 100);
        LMIC.dn2Dr = DR_SF12;

        cur_hours = rtc.getHours();
        cur_mins = rtc.getMinutes();

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Measuring...");

        //Read battery
        // read repeat two analog readings
        uint16_t analogVal = 0;
        analogVal = analogRead(BATPIN);
        delay(250);
        analogVal = analogRead(BATPIN);
        delay(250);
        bat = batt_measure(analogVal);

        digitalWrite(GATEL1, LOW); // ON Laser1
        delay(500);

        // FLUSH SERIAL
        lcd.setCursor(0, 2);
        lcd.print("S1");
        while (Serial.available() > 0)
        {
          // Serial.read();
          lcd.write(Serial.read());
        }

        Serial.write(0x4D);
        delay(4000);

        uint8_t serialSize = 0;
        while (Serial.available() > 0)
        {
          if (serialSize > 13)
          {
            Serial.read();
            continue;
          }
          bufferSerial1[serialSize] = Serial.read();
          serialSize += 1;
        }

        // TURN OFF LASER FIRST
        digitalWrite(GATEL1, HIGH); // OFF Laser1
        delay(500);

        digitalWrite(GATEL2, LOW); // ON Laser2
        delay(500);

        lcd.setCursor(0, 3);
        lcd.print("S2");
        while (Serial2.available() > 0)
        {
          // Serial2.read();
          lcd.write(Serial2.read());
        }

        Serial2.write(0x4D);
        delay(4000);

        serialSize = 0;
        while (Serial2.available() > 0)
        {
          if (serialSize > 13)
          {
            Serial2.read();
            continue;
          }
          bufferSerial2[serialSize] = Serial2.read();
          serialSize += 1;
        }

        digitalWrite(GATEL2, HIGH); // OFF Laser2
        delay(500);

        digitalWrite(GATEL3, LOW); // ON Laser3
        delay(500);

        lcd.setCursor(0, 4);
        lcd.print("S3");
        while (Serial3.available() > 0)
        {
          // Serial3.read();
          lcd.write(Serial3.read());
        }

        Serial3.write(0x4D);
        delay(4000);

        serialSize = 0;
        while (Serial3.available() > 0)
        {
          if (serialSize > 13)
          {
            Serial3.read();
            continue;
          }
          bufferSerial3[serialSize] = Serial3.read();
          serialSize += 1;
        }

        digitalWrite(GATEL3, HIGH); // OFF Laser3
        //        delay(500);

        // Dont turn off laser for calibration mode
        // if (WAKEUP_INT != 60) {
        //   digitalWrite(MOSFETPIN, LOW);   // OFF LASER
        // } else {
        //   Serial.write(0x4F);
        //   Serial2.write(0x4F);
        //   Serial3.write(0x4F);
        // }

        uint8_t int_part = (uint8_t)bat;
        float float_part = bat - (float)int_part;
        uint8_t float_int = float_part * 100;

        // Flush lora data
        for (int i = 0; i < LORA_DATA_SIZE; i++)
        {
          lora_data[i] = 0;
        }
        lora_data[0] = counter >> 24; // counter MSB
        lora_data[1] = counter >> 8;  // counter middle
        lora_data[2] = counter;       // counter LSB
        lora_data[3] = cur_hours;    // timee MSB
        lora_data[4] = cur_mins;         // timee LSB
        lora_data[5] = int_part;      // Bat int part
        lora_data[6] = float_int;     // Bat float part

        laser2lora(bufferSerial1, 7);
        laser2lora(bufferSerial2, 11);
        laser2lora(bufferSerial3, 15);

        lora_data[19] = time_passed >> 8;     // data #20
        lora_data[20] = time_passed;     // data #21

        lcd.setCursor(0, 5);
        lcd.print("SENDING...");
        // display.display();

        programming = PROG_TX_WAITING;
        os_setCallback(&sendjob, do_send);
        counter += 1;
        //                programming = PROG_TX_DONE;
        time_passed = rtc.getEpoch();
      }
      break;

    case PROG_TX_WAITING:
      {
        os_runloop_once();
      }
      break;

    case PROG_TX_DONE:
      {
        // setBackupRegister(FCNT_ADDR, LMIC.seqnoUp);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("C:");
        lcd.print(counter - 1);
        lcd.print(" B:");
        lcd.print(bat);
        lcd.setCursor(0, 1);
        lcd.print(D_ID);
        lcd.print(rtc.getHours());
        lcd.write(':');
        lcd.print(rtc.getMinutes());

        lcd.setCursor(0, 2);
        // display.print(LMIC.seqnoUp);
        for (int i = 0; i < 15; i++)
        {
          lcd.write(bufferSerial1[i]);
        }

        lcd.setCursor(0, 3);
        for (int i = 0; i < 15; i++)
        {
          lcd.write(bufferSerial2[i]);
        }

        lcd.setCursor(0, 4);
        for (int i = 0; i < 15; i++)
        {
          lcd.write(bufferSerial3[i]);
        }

        if (event_received_flag)
        {
          event_received_flag = false;
          downlink_handler(cur_hours, cur_mins);

        } else {
          programming = PROG_SLEEP;
          time_passed = rtc.getEpoch() - time_passed;
          lcd.setCursor(0, 5);
          lcd.print("OK:");
          lcd.print(run_mode);
          lcd.print(":");
          lcd.print(time_passed);
        }
      }
      break;

    case PROG_SLEEP:
      {
        programming = PROG_MEASURE;
        digitalWrite(LED, LOW); // OFF LED
        // Ensure all program done before sleep
        for (int i = 0; i < 4; i++)
        {
          os_runloop_once();
          delay(250);
        }

        //#define loraNSS PA4
        //#define loraRST PB0
        //#define loraDIO0 PB14
        //#define loraDIO1 PB13
        //#define loraDIO2 PB12

        digitalWrite(PA5, LOW);//sck
        pinMode(PA5, OUTPUT);
        digitalWrite(PA7, LOW);//mosi
        pinMode(PA7, OUTPUT);
        pinMode(PA6, INPUT_PULLUP);//miso
        digitalWrite(lmic_pins.nss, LOW);//nss
        pinMode(lmic_pins.nss, OUTPUT);
        pinMode(loraDIO0, INPUT_PULLUP);
        pinMode(loraDIO1, INPUT_PULLUP);
        pinMode(loraDIO2, INPUT_PULLUP);
        pinMode(lmic_pins.rst, INPUT_PULLUP);


        LowPower.deepSleep();
      }
      break;

    case PROG_JOIN_FAILED:
      {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Join Failed");
        lcd.setCursor(0, 1);
        lcd.print("Retry in 15min");
        lcd.setCursor(0, 2);
        lcd.print("Sleeping...");
        
        programming = PROG_REINIT;
        digitalWrite(LED, LOW); // OFF LED
        // Ensure all program done before sleep
        for (int i = 0; i < 4; i++)
        {
          os_runloop_once();
          delay(250);
        }

        digitalWrite(PA5, LOW);//sck
        pinMode(PA5, OUTPUT);
        digitalWrite(PA7, LOW);//mosi
        pinMode(PA7, OUTPUT);
        pinMode(PA6, INPUT_PULLUP);//miso
        digitalWrite(lmic_pins.nss, LOW);//nss
        pinMode(lmic_pins.nss, OUTPUT);
        pinMode(loraDIO0, INPUT_PULLUP);
        pinMode(loraDIO1, INPUT_PULLUP);
        pinMode(loraDIO2, INPUT_PULLUP);
        pinMode(lmic_pins.rst, INPUT_PULLUP);

        rtc.setAlarmEpoch(rtc.getEpoch() + 899);
        LowPower.deepSleep();
      }
      break;
      
    case PROG_REINIT:
      {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Initialzing...");
        
        digitalWrite(LED, HIGH); // ON LED
        
        programming = PROG_TX_WAITING;
        os_setCallback(&sendjob, initfunc);
      }
      break;
      

    default:

      {
        IWatchdog.begin(1000000);
        delay(3000);
      }

      break;
  }
}
