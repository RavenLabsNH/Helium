#include "MICS6814.h"

#define PIN_CO  0
#define PIN_NO2 0
#define PIN_NH3 0

MICS6814 gas(PIN_CO, PIN_NO2, PIN_NH3);

void setup() {
  
  pinMode(PIN_CO, INPUT_PULLUP);
  pinMode(PIN_NO2, INPUT_PULLUP);
  pinMode(PIN_NH3, INPUT_PULLUP);
  
  Serial.begin(9600);

  Serial.println("MICS-6814 Sensor Sample");
  Serial.print("Calibrating Sensor");

  gas.calibrate();

  Serial.println("OK!");
}

void loop() {
  Serial.print("NH3: ");
  Serial.print(gas.getResistance(CH_NH3));
  Serial.print("/");
  Serial.print(gas.getBaseResistance(CH_NH3));
  Serial.print(" = ");
  Serial.print(gas.getCurrentRatio(CH_NH3));
  Serial.print(" => ");
  Serial.print(gas.measure(NH3));
  Serial.println("ppm");
  delay(50);

  Serial.print("CO: ");
  Serial.print(gas.getResistance(CH_RED));
  Serial.print("/");
  Serial.print(gas.getBaseResistance(CH_RED));
  Serial.print(" = ");
  Serial.print(gas.getCurrentRatio(CH_RED));
  Serial.print(" => ");
  Serial.print(gas.measure(CO));
  Serial.println("ppm");
  delay(50);

  Serial.print("NO2: ");
  Serial.print(gas.getResistance(CH_OX));
  Serial.print("/");
  Serial.print(gas.getBaseResistance(CH_OX));
  Serial.print(" = ");
  Serial.print(gas.getCurrentRatio(CH_OX));
  Serial.print(" => ");
  Serial.print(gas.measure(NO2));
  Serial.println("ppm");

  delay(1000);
}