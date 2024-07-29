#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
// #include <SocketIoClient.h>
#include "MQ135.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085.h>
#include "DHTesp.h"

// Inicializar el controlador del sensor co2
Adafruit_BMP085 bmp;
// Inicializamos el controlador del sensor hum/temp
DHTesp dht;

// Pines de LEDs y Buzzer
const int LED_QUALITY_NORMAL = 26;  // Pin D26
const int LED_QUALITY_BAD = 14;     // Pin D14
const int LED_QUALITY_DANGER = 15;  // Pin D15
// Pin para el Buzzer
const int BUZZER_PIN = 13;
// Pin para el sensor de Co2
const int PIN_CO2 = 34;
// Pin para el sensor DHT11
const int DHTPIN = 19;

// // Constantes para la lectura del voltaje y la resistencia del sensor
// const float VOLTAGE_SUPPLY = 3.3;      // Voltaje de alimentación del sensor
// const int ADC_RESOLUTION = 4095;       // Resolución del ADC (ESP32)
// const float LOAD_RESISTANCE = 1000.0;  // Resistencia de carga en ohmios

const char* NODE_CODE = "NP001";

// Variables para conexión a servidor envío de datos
WiFiClient client;
const char* serverEndpointSaveData = "http://172.212.120.151/ms3/climate-datas";
// const char* serverEndpointSaveData = "http://192.168.1.119:4000/ms3/climate-datas";
const String endpointGetNodeStatus = "http://172.212.120.151/ms4/alerts/node-status/" + String(NODE_CODE);
// const String endpointGetNodeStatus = "http://192.168.1.119:4000/ms4/alerts/node-status/" + String(NODE_CODE);
const char* serverEndpointGetNodeStatus = endpointGetNodeStatus.c_str();
// const char* ssid = "RED-ViVi";
// const char* password = "ladeantes";
// U
const char* ssid = "Internet_UNL";
const char* password = "UNL1859WiFi";
// Variables para conexión a servidor de sockets
// SocketIoClient socket;
// const char* socketsServerEndpoint = "192.168.1.119";
// const int socketsServerPort = 5005;

// Variables para definir tiempo de envío de datos a servidor
unsigned long previousMillis = 0;
const long interval = 40000;  // Intervalo de tiempo a realizar petición (1 minuto)

// Variables para definir tiempo de actualización de alerta
unsigned long previousMillisUpdateAlert = 0;
const long intervalUpdateAlert = 2000;  // Intervalo de tiempo a realizar petición (1 minuto)

// Valor de R0 obtenido después de la calibración
const float R0_CO2 = 50;
const float R0_CO21 = 80;
const float R0_CO22 = 150;
// const float R0_CO2 = 104883.01;

// MQ135 gasSensor = MQ135(PIN_CO2);

void setup() {
  // Configuración de la comunicación serial
  Serial.begin(115200);

  // Configuración del sensor DHT11
  dht.setup(DHTPIN, DHTesp::DHT11);

  // Configuración del sensor BMP180
  bmp.begin();

  // Esperar un poco antes de iniciar la conexión WiFi
  delay(1000);

  // Conexión a la red WiFi
  WiFi.begin(ssid, password);

  Serial.print("Conectando a WiFi...");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("Conectado a WiFi");
  Serial.println();

  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  Serial.print("Dirección MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.println();

  // Configuración de los pines de los LEDs y Buzzer
  pinMode(LED_QUALITY_NORMAL, OUTPUT);  // Led de calidad del aire saludable
  pinMode(LED_QUALITY_BAD, OUTPUT);     // Led de calidad del aire mala
  pinMode(LED_QUALITY_DANGER, OUTPUT);  // Led de calidad del aire muy mala
  pinMode(BUZZER_PIN, OUTPUT);          // Pin del Buzzer

  pinMode(PIN_CO2, INPUT);

  // Inicialización de los LEDs y Buzzer
  digitalWrite(LED_QUALITY_NORMAL, LOW);
  digitalWrite(LED_QUALITY_BAD, LOW);
  digitalWrite(LED_QUALITY_DANGER, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  // Conexión a Socket.IO
  // socket.on("connect", handleOnSocketConnect);
  // socket.on("disconnect", handleOnSocketDisconnect);

  // String alertEvent = "alertNode" + String(NODE_CODE);
  // socket.on(alertEvent.c_str(), handleSocketEvent);

  // socket.on("error", [](const char* payload, size_t length) {
  //   Serial.println("Error en la conexión de socket");
  // });

  // socket.begin(socketsServerEndpoint, socketsServerPort);
}

void loop() {
  unsigned long currentMillis = millis();
  unsigned long currentMillisUpdateAlert = millis();

  digitalWrite(LED_QUALITY_NORMAL, LOW);
  digitalWrite(LED_QUALITY_BAD, LOW);
  digitalWrite(LED_QUALITY_DANGER, LOW);

  double CO2 = getCo2();
  Serial.print("CO2: ");
  Serial.println(CO2);

  // Comprobar si ha pasado el intervalo de tiempo
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    sendData();
  }

  // Comprobar si ha pasado el intervalo de tiempo
  if (currentMillisUpdateAlert - previousMillisUpdateAlert >= intervalUpdateAlert) {
    previousMillisUpdateAlert = currentMillisUpdateAlert;

    updateNodeAlertStatus();
  }


  delay(1000);
  // Mantener la conexión de Socket.IO activa
  // socket.loop();
}

// Función para enviar datos al servidor
void sendData() {
  float temperature = getTemperatura();
  float humidity = getHumedad();
  double co2 = getCo2();

  // Crear JSON
  StaticJsonDocument<200> doc;
  doc["temp"] = temperature;
  doc["hum"] = humidity;
  doc["co2"] = co2;
  doc["nodeCode"] = NODE_CODE;

  // Serializar JSON a string
  String jsonStr;
  serializeJson(doc, jsonStr);

  // Envio JSON al servidor
  HTTPClient http;
  http.begin(serverEndpointSaveData);

  // Configuración de las cabeceras de la solicitud
  http.addHeader("Content-Type", "application/json");

  int httpResponseCode = http.POST(jsonStr);

  Serial.print("Código de respuesta HTTP: ");
  Serial.println(httpResponseCode);

  Serial.println("Datos enviados: ");
  Serial.println(jsonStr);

  Serial.print("Respuesta del servidor:");
  Serial.println(http.getString());
  Serial.println();

  if (httpResponseCode > 0) {
    Serial.println("Éxito en la solicitud HTTP");
  } else {
    Serial.println("Error en la solicitud HTTP");
  }

  http.end();
}

void updateNodeAlertStatus() {
  // Envio JSON al servidor
  HTTPClient http;
  http.begin(serverEndpointGetNodeStatus);

  // Configuración de las cabeceras de la solicitud
  http.addHeader("Content-Type", "application/json");

  int httpResponseCode = http.GET();

  Serial.print("Código de respuesta HTTP: ");
  Serial.println(httpResponseCode);

  Serial.print("Respuesta del servidor: ");
  String payload = http.getString();
  Serial.println(payload);

  if (httpResponseCode > 0) {
    Serial.println("Éxito en la solicitud GET HTTP");

    // Crear un objeto para analizar el JSON
    DynamicJsonDocument doc(1024);  // Tamaño del buffer, ajusta según sea necesario
    DeserializationError error = deserializeJson(doc, payload);

    if (error) {
      Serial.print("Error al analizar JSON: ");
      Serial.println(error.c_str());
      http.end();
      return;
    }

    // Extraer el valor de emitSound
    bool emitSound = doc["emitSound"];

    if (emitSound) {
      digitalWrite(LED_QUALITY_DANGER, HIGH);
      digitalWrite(BUZZER_PIN, HIGH);
    } else {
      digitalWrite(LED_QUALITY_DANGER, LOW);
      digitalWrite(BUZZER_PIN, LOW);
    }
  } else {
    Serial.println("No se pudo obtener el status del nodo");
  }

  http.end();
}

// Función para obtener la temperatura
float getTemperatura() {
  float temperatura = dht.getTemperature();

  Serial.print("Temperatura: ");
  Serial.print(temperatura);
  Serial.println("°C");

  return temperatura;
}

// Función para obtener la humedad
float getHumedad() {
  float humedad = dht.getHumidity();

  Serial.print("Humedad: ");
  Serial.print(humedad);
  Serial.println(" %");
  Serial.println();

  return humedad;
}

// Función para obtener el CO2
double getCo2() {
  //Valor dado por el sensor
  MQ135 gasSensor = MQ135(PIN_CO2, R0_CO2);
  MQ135 gasSensor1 = MQ135(PIN_CO2, R0_CO21);
  MQ135 gasSensor2 = MQ135(PIN_CO2, R0_CO22);

  // float air_quality = gasSensor.getPPM();  //Leido de forma analogica
  // //V_leido=5(1000/(Rs+1000))
  // //Rs=1000((5-V)/V)
  // float voltaje = air_quality * (5.0 / 1023.0);  // Parte indispensable
  // Serial.println("Voltaje:  ");
  // Serial.println(voltaje);
  // float Rs = 1000 * ((5 - voltaje) / voltaje);  // Resistencia del sensor
  // double CO2 = 122.06 * pow((Rs / R0_CO2), -2.845);
  // //400 = 122.06 * (Rs/R0_CO2^-2,845)
  // Serial.print(air_quality);
  // Serial.println(" PPM");
  // Serial.println("-----Valor obtenido del CO2-----");
  // Serial.print(CO2);
  double CO2 = gasSensor.getPPM();
  double CO21 = gasSensor1.getPPM();
  double CO22 = gasSensor2.getPPM();

  Serial.print("TRES VALORES CO2:");
  Serial.println(CO2);
  Serial.println(CO21);
  Serial.println(CO22);


  return CO2;
}

// // Función para manejar eventos de Socket.IO
// void handleSocketEvent(const char* payload, size_t length) {
//   Serial.println("----- En socket -----");
//   StaticJsonDocument<200> doc;
//   deserializeJson(doc, payload);

//   bool emitSound = doc["emitSound"];

//   Serial.println("----- Valor obtenido recibido socket -----");
//   Serial.print(emitSound);
//   Serial.println("------------------------------------------");

//   if (emitSound) {
//     digitalWrite(LED_QUALITY_DANGER, HIGH);
//     digitalWrite(BUZZER_PIN, HIGH);
//   } else {
//     digitalWrite(LED_QUALITY_DANGER, LOW);
//     digitalWrite(BUZZER_PIN, LOW);
//   }
// }

// void handleOnSocketConnect(const char* payload, size_t length) {
//   Serial.println("Conectado al servidor de sockets");
// }

// void handleOnSocketDisconnect(const char* payload, size_t length) {
//   Serial.println("Desconectado del servidor de sockets");
// }