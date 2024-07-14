#include "MQ135.h"
#include <WiFi.h>
#include <Wire.h>
#include <ThingSpeak.h>
//Usado para macar diferencias entre estados
#define pinVerde 2
#define pinNaranja 4
#define pinRojo 5
//Rs/Ro = 2,34
//CO2 del ambiente = 400 ppm
float R0CO2 = 104883.01;
int air_quality;

const char* ssid = "AGILAC";
const char* password = "1104185507";
WiFiClient client;
unsigned long myChannelNumber = 1;
const char *myWriteAPIKey = "EKWSP6GJJKE345ZJ";
unsigned long lastime = 0;
unsigned long timeDelay = 20000;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  // pinMode(pinVerde, OUTPUT);
  // pinMode(pinNaranja, OUTPUT);
  // pinMode(pinRojo, OUTPUT);
  pinMode(34, INPUT);
  WiFi.mode(WIFI_STA);
  ThingSpeak.begin(client);
}

void loop() {
  //Conexion a internet
  if( (millis() - lastime) > timeDelay ){
    //Coneccion o reconextar WiFi
    if(WiFi.status() != WL_CONNECTED){
      Serial.print("Intentando conectar");
      while(WiFi.status() != WL_CONNECTED){
        WiFi.begin(ssid, password);
        delay(5000);
      }
      Serial.print("\Conectado: ");
    }
    //Valor dado por el sensor
    MQ135 gasSensor = MQ135(34); 

    float air_quality = gasSensor.getPPM(); //Leido de forma analogica
    //V_leido=5(1000/(Rs+1000))
    //Rs=1000((5-V)/V) 
    float voltaje = air_quality*(5.0/1023.0); // Parte indispensable
    Serial.println("Voltaje:  ");
    Serial.println(voltaje);
    float Rs = 1000*( (5-voltaje)/voltaje); // Resistencia del sensor
    double CO2 = 122.06*pow((Rs/R0CO2), -2.845);
    //400 = 122.06 * (Rs/R0CO2^-2,845)  
    Serial.print(air_quality);
    Serial.println(" PPM");
    Serial.println("-----Valor obtneido del CO2-----");
    Serial.print(CO2);
    int x = ThingSpeak.writeField(myChannelNumber, 1, air_quality, myWriteAPIKey);
    if(x == 200){
      Serial.println("Canal actualizado correctamente");
    }else{
      Serial.println("Existe un problema en la actualziacion del canal " + String(x));
    }
    lastime = millis();

    //Impresion de leds 
  //   if(CO2 >= 900){
  //     digitalWrite(pinRojo, HIGH);
  //   }
  //   if(CO2 >= 400 && CO2 < 900 ){ 
  //     digitalWrite(pinNaranja, HIGH);
  //   }
  //   if(CO2 > 0.4 && CO2 < 400){
  //     digitalWrite(pinVerde, HIGH);
  //   }
  }
  // put your main code here, to run repeatedly:

}
