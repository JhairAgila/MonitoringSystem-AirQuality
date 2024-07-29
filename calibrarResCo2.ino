#include <MQ135.h>

// Definir el pin analógico al que está conectado el AOUT del sensor MQ135
const int MQ135_PIN = 34;  // Puedes cambiar esto al pin que uses
const float RO_CLEAN_AIR_FACTOR = 9.83;  // Valor recomendado para MQ135

MQ135 gasSensor = MQ135(MQ135_PIN);

void setup() {
  Serial.begin(115200);

  // Calibrar el sensor en aire limpio
  float R0 = calibrateSensor();
  Serial.print("Valor calibrado de R0: ");
  Serial.println(R0);

  // Fijar el valor de R0 calibrado
  gasSensor.setR0(R0);
}

void loop() {
  double ppm = gasSensor.getPPM();

  Serial.print("CO2 (ppm): ");
  Serial.println(ppm);

  delay(2000);
}

float calibrateSensor() {
  const int numReadings = 50;  // Número de lecturas para el promedio
  float resistanceSum = 0.0;

  for (int i = 0; i < numReadings; i++) {
    resistanceSum += gasSensor.getResistance();
    delay(500);
  }

  float avgResistance = resistanceSum / numReadings;
  float R0 = avgResistance / RO_CLEAN_AIR_FACTOR;

  return R0;
}
