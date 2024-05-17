#include <EEPROM.h>

void recieveSerial() {
  if (Serial.available()) {
    String serialrx = Serial.readString();
    if (serialrx) {
      serialrx.trim();
      serialrx.toLowerCase();
      Serial.print("Received: ");
      Serial.println(serialrx);

      if (serialrx.startsWith("getcanspeed")) {
        Serial.print("CAN Speed=");
        Serial.println(getCanSpeed());
      } else if (serialrx.startsWith("setcanspeed")) {
        serialrx.remove(0, 12);
        long sp = serialrx.toInt();
        if (sp > 0) {
          setCanSpeed(sp);
          Serial.print("Updated CAN Speed=");
          Serial.println(getCanSpeed());
          Serial.println("Restart to apply changes.");
        } else {
          Serial.print("Invalid speed found: ");
          Serial.println(serialrx);
        }
      } else if (serialrx.startsWith("getflowpin")) {
        Serial.print("Flow Sensor Pin=");
        Serial.println(getFlowPin());
      } else if (serialrx.startsWith("setflowpin")) {
        serialrx.remove(0, 10);
        long sp = serialrx.toInt();
        if (sp >= 0) {
          setFlowPin(sp);
          Serial.print("Flow Sensor Pin=");
          Serial.println(getFlowPin());
          Serial.println("Restart to apply changes.");
        } else {
          Serial.print("Invalid pin found: ");
          Serial.println(serialrx);
        }
      } else if (serialrx.startsWith("reset")) {
        Serial.print("Reseting...");
        resetFuel();
      } else if (serialrx.startsWith("getmetric")) {
        Serial.print("MetricUnits=");
        Serial.println(getMetric());
      } else if (serialrx.startsWith("setmetric")) {
        serialrx.remove(0, 9);
        long sp = serialrx.toInt();
        if (sp >= 0) {
          setMetric(sp);
          Serial.print("MetricUnits=");
          Serial.println(getMetric());
          Serial.println("Restart to apply changes.");
        } else {
          Serial.print("Invalid metric units: ");
          Serial.println(serialrx);
        }
      } else if (serialrx.startsWith("getk")) {
        Serial.print("K=");
        Serial.println(getK());
      } else if (serialrx.startsWith("setk")) {
        serialrx.remove(0, 4);
        long sp = serialrx.toInt();
        if (sp >= 0) {
          setK(sp);
          Serial.print("K=");
          Serial.println(getK());
          Serial.println("Restart to apply changes.");
        } else {
          Serial.print("Invalid K value found: ");
          Serial.println(serialrx);
        }
      }
    }
  }
}

void setCanSpeed(uint8_t speedEnum) {
  EEPROM.write(2, speedEnum);
}
uint8_t getCanSpeed() {
  return EEPROM.read(2);
}
void setFlowPin(uint8_t pin) {
  EEPROM.write(3, pin);
}
uint8_t getFlowPin() {
  return EEPROM.read(3);
}
void setMetric(uint8_t isMetric){
  EEPROM.write(4, isMetric);
}
uint8_t getMetric() {
  return EEPROM.read(4);
}
void setK(uint32_t k) {
  const uint8_t idx = 5; 
  EEPROM.write(idx, (k >> 24) & 0xFF);
  EEPROM.write(idx + 1, (k >> 16) & 0xFF);
  EEPROM.write(idx + 2, (k >> 8) & 0xFF);
  EEPROM.write(idx + 3, k & 0xFF);
}
uint32_t getK() {
  const uint8_t idx = 5; 
  return ((uint32_t)EEPROM.read(idx) << 24) + ((uint32_t)EEPROM.read(idx + 1) << 16) + ((uint32_t)EEPROM.read(idx + 2) << 8) + (uint32_t)EEPROM.read(idx + 3);
}

