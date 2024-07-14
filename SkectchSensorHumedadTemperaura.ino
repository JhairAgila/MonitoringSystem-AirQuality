// #include <DHT.h>
#include "DHTesp.h"
DHTesp dht;
const int Dhtpin = 4; //Pin para el Sensor


float getTemperatura(){
  float temperatura = dht.getTemperature(); // Recuperar la temperatura del sensor
  
  Serial.print("Temperatura: ");
  Serial.print(temperatura);
  Serial.println(" °C");

  return temperatura;
}

float getHumedad(){
  float humedad = dht.getHumidity(); // Recuperar la humedad del sensor

  Serial.print("Humedad: ");
  Serial.print(humedad);
  Serial.println(" %");
  Serial.println();

  return humedad;
}

void setup() {
  // put your setup code here, to run once:
  // dht.begin();
  delay(3000);
  dht.setup(Dhtpin, DHTesp::DHT11);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  float temperatura = getTemperatura();
  float humedad = getHumedad();
  // float presion = getPresion();
  // float temperatura = dht.readTemperature();
  // float humedad = dht.readHumidity();
  Serial.print("Temp: ");
  Serial.print(temperatura);
  Serial.println(" C");

  Serial.print("Humedad: ");
  Serial.print(humedad);
  Serial.print("%");

  delay(3000);
}
