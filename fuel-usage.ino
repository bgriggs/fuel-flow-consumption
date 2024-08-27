#include <mcp_can.h>
#include <SPI.h>

#define SAVE_FREQMS 30000  // EEPROM limited to 100000 writes so limit this frequency
#define EEPROM_ADDR 0

uint32_t K = 68000;
const int SPI_CS_PIN = 17;  // CANBed
MCP_CAN CAN(SPI_CS_PIN);    // Set CS pin
unsigned long lastFuelStatus = 0;
volatile unsigned long fuelPulses;
const float fuelResetThresholdGals = 4.0;
unsigned long fuelFullResetThresholdStarted = 0;
const uint32_t fuelFullResetThresholdMs = 10000;

typedef struct {
  unsigned long fuelPulses;
  float fuelUsedGals;
  float fuelRemainingGals;
  float fuelConsumptionGalMin;
  float fuelRemainingMins;
  float fuelConsumptionGalLap;
  int lapsRemaining;
} FuelStatus;

typedef struct {
  float capacityGals;
  unsigned long lastLapMs;
  float speedMph;
  float fuelLevelGals;
  bool fuelFull;
  bool autoReset;
} Parameters;
Parameters params;

bool metricUnits = 0;

void setup() {
  memset(&params, 0, sizeof(params));
  startFlowMeter();
  Serial.begin(115200);
  // while (!Serial)
  //   ;

  metricUnits = getMetric();

  uint8_t sp = getCanSpeed();
  Serial.print("CAN Speed=");
  Serial.println(sp);
  while (CAN_OK != CAN.begin(sp)) {
    Serial.println("CAN BUS FAIL!");
    delay(100);
  }

  // Mask 0 is for filter 0 and 1
  // ID mask - 7FF accepts all
  // CAN.init_Mask(0, 0, 0x7FF);
  // CAN.init_Filt(0, 0, 0x338);
  // CAN.init_Filt(1, 0, 0x339);

  // Mask 1 is for filter 2-5.
  // These must be set to non-zero otherwise will allow all traffic through
  // CAN.init_Mask(1, 0, 0x7FF);
  // CAN.init_Filt(2, 0, 0x339);
  // CAN.init_Filt(3, 0, 0x339);
  // CAN.init_Filt(4, 0, 0x339);
  // CAN.init_Filt(5, 0, 0x339);

  CAN.init_Mask(0, CAN_STDID, 0x3FF);  
  CAN.init_Mask(1, CAN_EXTID, 0x1FFFFFFF);  
  CAN.init_Filt(0, CAN_STDID, 0);
  CAN.init_Filt(1, CAN_STDID, 0);
  CAN.init_Filt(2, CAN_EXTID, 0x100B0001);
  CAN.init_Filt(3, CAN_EXTID, 0x100B0002);
  CAN.init_Filt(4, CAN_EXTID, 0x100B0002);
  CAN.init_Filt(5, CAN_EXTID, 0x100B0002);

  Serial.println("CAN BUS OK!");
}

void loop() {  
  bool reset = 0;
  static unsigned long lastRest = 0;
  recieveCanParams(&reset);
  recieveSerial();

  // Check for manual reset
  if (reset && (millis() - lastRest) > 1000) {
    resetFuel();
    lastRest = millis();
  }

  if ((millis() - lastFuelStatus) > 500) {
    FuelStatus fs = getFuelStatus(params.capacityGals, params.lastLapMs);

    // Check to see if status should be reset due to refueling
    if (params.autoReset) {
      reset = checkAutoResetConditions(fs.fuelUsedGals);
      if (reset) { resetFuel(); }
    }

    printDebug(fs);

    // fs.fuelPulses = 12345678;
    //  fs.fuelUsedGals = 5.55123;
    //  fs.fuelConsumptionGalMin = 0.1553456;
    //  fs.fuelRemainingGals = 8.2134213;
    //  fs.fuelRemainingMins = 33.333;
    //  fs.lapsRemaining = 21;
    //  fs.fuelConsumptionGalLap = 0.41;
    transmitFuelData(fs);

    lastFuelStatus = millis();
  }
  delay(50);
}

void printDebug(FuelStatus fs) {
    Serial.print("pulses=");
    Serial.print(fs.fuelPulses);
    Serial.print(",galsUsed=");
    Serial.print(fs.fuelUsedGals);
    Serial.print(",fuelRemaining=");
    Serial.print(fs.fuelRemainingGals);
    Serial.print(",consGalMin=");
    Serial.print(fs.fuelConsumptionGalMin);
    Serial.print(",remMins=");
    Serial.print(fs.fuelRemainingMins);
    Serial.print(",lapsRem=");
    Serial.println(fs.lapsRemaining);
}

void transmitFuelData(FuelStatus fs) {
  // fuelPulses[4], fuelUsedGals[2], fuelConsumptionGalMin[2]
  unsigned char fuelData1[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
  // fuelRemainingGals[2], fuelRemainingMins[2] * 60, lapsRemaining[2]
  unsigned char fuelData2[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
  unsigned char fuelData3[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

  // Message 0x100B0003
  fuelData1[0] = (unsigned char)((fs.fuelPulses & 0xFF000000) >> 24);
  fuelData1[1] = (unsigned char)((fs.fuelPulses & 0x00FF0000) >> 16);
  fuelData1[2] = (unsigned char)((fs.fuelPulses & 0x0000FF00) >> 8);
  fuelData1[3] = (unsigned char)((fs.fuelPulses & 0x000000FF));
  uint16_t fu = (uint16_t)(checkConvertGals(fs.fuelUsedGals) * 100);
  fuelData1[4] = (unsigned char)((fu & 0xFF00) >> 8);
  fuelData1[5] = (unsigned char)((fu & 0x00FF));

  uint16_t fc;
  if (metricUnits) {
    // cc / sec
    fc = (uint16_t)(fs.fuelConsumptionGalMin * 63.0902);
  }
  else {
    fc = (uint16_t)(fs.fuelConsumptionGalMin * 10000);
  }
  fuelData1[6] = (unsigned char)((fc & 0xFF00) >> 8);
  fuelData1[7] = (unsigned char)((fc & 0x00FF));

  // Message 0x100B0004
  uint16_t frg = (uint16_t)(checkConvertGals(fs.fuelRemainingGals) * 100);
  fuelData2[0] = (unsigned char)((frg & 0xFF00) >> 8);
  fuelData2[1] = (unsigned char)((frg & 0x00FF));
  uint16_t fr = (uint16_t)(fs.fuelRemainingMins * 60);
  fuelData2[2] = (unsigned char)((fr & 0xFF00) >> 8);
  fuelData2[3] = (unsigned char)((fr & 0x00FF));
  uint16_t lr = (uint16_t)fs.lapsRemaining;
  fuelData2[4] = (unsigned char)((lr & 0xFF00) >> 8);
  fuelData2[5] = (unsigned char)((lr & 0x00FF));
  uint16_t gl = (uint16_t)(fs.fuelConsumptionGalLap * 10000);
  fuelData2[6] = (unsigned char)((gl & 0xFF00) >> 8);
  fuelData2[7] = (unsigned char)((gl & 0x00FF));

  // Message 0x100B0005
  // liters / lap
  uint16_t lilap = (uint16_t)(fs.fuelConsumptionGalLap * 3.7854117 * 10000);
  fuelData3[0] = (unsigned char)((lilap & 0xFF00) >> 8);
  fuelData3[1] = (unsigned char)((lilap & 0x00FF));

  // for (int i = 0; i < 8; i++) {
  //   Serial.print(fuelData1[i], HEX);
  //   Serial.print(",");
  // }
  // Serial.println();
  // for (int i = 0; i < 8; i++) {
  //   Serial.print(fuelData2[i], HEX);
  //   Serial.print(",");
  // }
  // Serial.println();

  CAN.sendMsgBuf(0x100B0003, CAN_EXTID, 8, fuelData1);
  CAN.sendMsgBuf(0x100B0004, CAN_EXTID, 8, fuelData2);
  CAN.sendMsgBuf(0x100B0005, CAN_EXTID, 8, fuelData3);
}

void recieveCanParams(bool* reset) {
  if (CAN_MSGAVAIL == CAN.checkReceive()) {
    unsigned long id = 0;
    unsigned char len = 0;
    unsigned char buf[8];
    CAN.readMsgBufID(&id, &len, buf);

    //  Serial.print(id, HEX);
    //  Serial.print("=");
    //  for (int i = 0; i < 8; i++) {
    //    Serial.print(buf[i], HEX);
    //    Serial.print(",");
    //  }
    //  Serial.println();

    // Fuel capacity, 2 bytes x100
    // Last lap, ms 4 bytes
    // Autoreset, 1 byte
    if (id == 0x100B0001) {
      uint16_t c = (uint16_t)buf[0] << 8;
      c += (uint16_t)buf[1];
      params.capacityGals = checkConvertLiters(c / 100.0);

      params.lastLapMs = ((unsigned long)buf[2]) << 24;
      params.lastLapMs += (unsigned long)buf[3] << 16;
      params.lastLapMs += (unsigned long)buf[4] << 8;
      params.lastLapMs += (unsigned long)buf[5];

      params.autoReset = buf[6];
    }
    // Speed, x10 2 bytes
    // Fuel Level, 2 bytes x100
    // Fuel full, 1 byte
    // Fuel reset, 1 byte
    else if (id == 0x100B0002) {
      uint16_t s = (uint16_t)buf[0] << 8;
      s += (uint16_t)buf[1];
      params.speedMph = checkConvertKph(s / 10.0);
      uint16_t fl = (uint16_t)buf[2] << 8;
      fl += (uint16_t)buf[3];
      params.fuelLevelGals = checkConvertLiters(fl / 100.0);
      params.fuelFull = buf[4];
      *reset = buf[5];
    }

    Serial.print("cap=");
    Serial.print(params.capacityGals);
    Serial.print(",lastLapMs=");
    Serial.print(params.lastLapMs);
    Serial.print(",autoReset=");
    Serial.print(params.autoReset);
    Serial.print(",speed=");
    Serial.print(params.speedMph);
    Serial.print(",fuelLevel=");
    Serial.print(params.fuelLevelGals);
    Serial.print(",fuelFull=");
    Serial.print(params.fuelFull);
    Serial.print(",fuelReset=");
    Serial.println(*reset);
  }
}

bool checkAutoResetConditions(float fuelUsedGals) {
  if (params.fuelFull && fuelFullResetThresholdStarted == 0) {
    fuelFullResetThresholdStarted = millis();
  }
  else { // Fuel light goes off, reset the count
    fuelFullResetThresholdStarted = 0;
  }

  unsigned long fuelFullOnForMs = millis() - fuelFullResetThresholdStarted;
  
  // Fuel is full and has been on constantly beyond the threshold. This avoids fuel sloshing on the sensor when coming to a stop, such as during a red.
  return params.fuelFull && fuelFullOnForMs > fuelFullResetThresholdMs && 
    fuelUsedGals > fuelResetThresholdGals && // Fuel used must be passed a reasonable amout to make sense for a pit stop refuel
    params.speedMph < 3.0;
}

// Gals to liters
float checkConvertGals(float gals) {
  if (metricUnits) {
    return gals * 3.7854117;
  }
  return gals;
}

// Liters to Gallons
float checkConvertLiters(float liters) {
  if (metricUnits) {
    return liters * 0.26417205;
  }
  return liters;
}

// KPH to MPH
float checkConvertKph(float kph) {
  if (metricUnits) {
    return kph * 0.62137119;
  }
  return kph;
}
