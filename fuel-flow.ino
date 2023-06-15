//#define FLOWSENSORPIN 0  // RX
#define K 68000
#define SAVE_FREQMS 30000  // EEPROM limited to 100000 writes so limit this frequency
#define EEPROM_ADDR 0

volatile unsigned long fuelPulses;
volatile uint8_t lastflowpinstate;

unsigned long lastFuelConsSave = 0;

typedef struct {
  unsigned long time;
  unsigned long pulses;
} FuelReading;

// Consumption rate history
const unsigned int HIST_FREQMS = 100;
const byte MAX_HISTORY = 120;
FuelReading pulseHistory[MAX_HISTORY];
unsigned int pulseHistoryPos = 0;
unsigned long lastFuelReading = 0;
uint8_t sensorPin = 0;

// Laps history
unsigned long lastLapTime;
const byte MAX_LAPS = 3;
unsigned long lapTimes[MAX_LAPS];
byte lapTimePos = 0;

void startFlowMeter() {
  float savedGals = loadFuelCons();
  resetFuel();
  fuelPulses = savedGals * K;
  sensorPin = getFlowPin();
  pinMode(sensorPin, INPUT_PULLUP);
  digitalWrite(sensorPin, HIGH);
  lastflowpinstate = digitalRead(sensorPin);

  // Interrupt every ms
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);
}

void resetFuel() {
  fuelPulses = 0;
  pulseHistoryPos = 0;
  lapTimePos = 0;
  saveFuelCons(0.0);
}

FuelStatus getFuelStatus(float fuelCapacityGals, unsigned long lapTimeMs) {
  unsigned long now = millis();
  if ((now - lastFuelReading) >= HIST_FREQMS) {
    // Move history window
    if (pulseHistoryPos >= MAX_HISTORY - 1) {
      for (int i = 0; i < MAX_HISTORY - 1; i++) {
        pulseHistory[i] = pulseHistory[i + 1];
      }
      //memmove(&pulseHistory[0], &pulseHistory[1], sizeof(pulseHistory[0]) * (MAX_HISTORY - 1));
      //memset(&pulseHistory[MAX_HISTORY - 1], 0, sizeof(pulseHistory[0]));
      pulseHistoryPos = MAX_HISTORY - 1;
    }

    FuelReading fr;
    fr.time = now;
    fr.pulses = fuelPulses;
    pulseHistory[pulseHistoryPos] = fr;
    pulseHistoryPos++;
    lastFuelReading = now;
  }

  // Move laps window
  if (lapTimePos >= MAX_LAPS) {
    // memmove(&lapTimes, &lapTimes[1], sizeof(lapTimes[0]) * (MAX_LAPS - 1));
    // memset(&lapTimes[MAX_LAPS - 1], 0, sizeof(lapTimes[0]));
    for (int i = 0; i < MAX_LAPS - 1; i++) {
      lapTimes[i] = lapTimes[i + 1];
    }
    lapTimePos = MAX_LAPS - 1;
  }

  // Update last lap time
  if (lapTimeMs > 0 && lastLapTime != lapTimeMs) {
    lapTimes[lapTimePos] = lapTimeMs;
    lastLapTime = lapTimeMs;
    lapTimePos++;
  }

  FuelStatus s;
  memset(&s, 0, sizeof(s));
  s.fuelPulses = fuelPulses;
  s.fuelUsedGals = fuelPulses / (float)K;

  if (fuelCapacityGals > 0.0) {
    s.fuelRemainingGals = fuelCapacityGals - s.fuelUsedGals;
    if (s.fuelRemainingGals < 0)
      s.fuelRemainingGals = 0;
  }

  // Calculate consumption when there is sufficent history
  if (pulseHistoryPos == MAX_HISTORY) {
    unsigned long durationms = pulseHistory[pulseHistoryPos - 1].time - pulseHistory[0].time;
    unsigned long usage = pulseHistory[pulseHistoryPos - 1].pulses - pulseHistory[0].pulses;
    float usedGals = usage / (float)K;
    float usedGalSec = usedGals / ((float)durationms / 1000.0);
    s.fuelConsumptionGalMin = usedGalSec * 60.0;
    if (s.fuelConsumptionGalMin < 0)
      s.fuelConsumptionGalMin = 0;
  }

  // Calculate time remaining
  if (s.fuelRemainingGals > 0 && s.fuelConsumptionGalMin > 0) {
    s.fuelRemainingMins = s.fuelRemainingGals / s.fuelConsumptionGalMin;
  }

  // Calculate laps remaining
  if (lapTimePos == MAX_LAPS && s.fuelRemainingMins > 0) {
    byte count = 0;
    unsigned long totalLapTime = 0;
    for (int i = 0; i < MAX_LAPS; i++) {
      if (lapTimes[i] > 0) {
        totalLapTime += lapTimes[i];
        count++;
      }
    }
    if (count > 0) {
      float avgLapTimeMins = (totalLapTime / (float)count) / 60.0 / 1000.0;
      s.lapsRemaining = (int)(s.fuelRemainingMins / avgLapTimeMins);
      if (s.lapsRemaining < 0)
        s.lapsRemaining = 0;
    }
  }

  // Save current fuel used
  if ((now - lastFuelConsSave) > SAVE_FREQMS) {
    saveFuelCons(s.fuelUsedGals);
    lastFuelConsSave = now;
  }

  return s;
}

void saveFuelCons(float gals) {
  uint16_t g = (uint16_t)(gals * 1000);
  EEPROM.update(EEPROM_ADDR, (uint8_t)((g & 0xFF00) >> 8));
  EEPROM.update(EEPROM_ADDR + 1, (uint8_t)(g & 0x00FF));
}

float loadFuelCons() {
  uint16_t gup = EEPROM.read(EEPROM_ADDR);
  uint16_t glo = EEPROM.read(EEPROM_ADDR + 1);
  return ((float)((gup << 8) + glo)) / 1000.0;
}

// Interrupt is called once a millisecond, looks for any pulses from the sensor
SIGNAL(TIMER0_COMPA_vect) {
  uint8_t x = digitalRead(sensorPin);

  if (x == lastflowpinstate)
    return;

  if (x == LOW)
    fuelPulses++;

  lastflowpinstate = x;
}