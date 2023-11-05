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