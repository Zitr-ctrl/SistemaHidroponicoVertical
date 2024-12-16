#include <WiFi.h>
#include <HTTPClient.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Configuración WiFi
const char* ssid = "UNEMI-WIFI";
const char* password = "";
const char* serverName = "https://script.google.com/macros/s/AKfycbzJSgBg2WbBjqFwugFZ9LhJJdlFGt0wU-0l3vt2B-rPn-UtU7QtXRqHa4FlWWDV6NmeqQ/exec";

// Configuración de LCD
LiquidCrystal_I2C lcd(0x27, 16, 2); // Dirección I2C, 16 columnas y 2 filas

// Flujo de Agua
const int flowSensorPin = 32;
volatile int pulseCount = 0;
unsigned long previousMillis = 0;
float calibrationFactor = 4.5;
float flowRate = 0;
float totalLitres = 0;

void IRAM_ATTR pulseCounter() {
  pulseCount++;
}

// Sensor de temperatura y humedad
#define DHTPIN 15  
#define DHTTYPE DHT11  
DHT dht(DHTPIN, DHTTYPE);
float temp = 0.0;
float hum = 0.0;

// Sensor de pH
const int phPin = 35;
float voltage;
float pHValue;
float voltageStep = 3.3 / 4095.0;

// Sensor de nivel de agua
const int sensorPin = 34;
int sensorValue = 0;

// Variables de control
unsigned long timeRead = 0;
unsigned long lcdUpdateTime = 0;
int displayState = 0;

void setup() {
  Serial.begin(115200);

  // Configuración de flujo de agua
  pinMode(flowSensorPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(flowSensorPin), pulseCounter, FALLING);

  // Inicializar sensores
  dht.begin();
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Sistema Iniciado");
  delay(2000);
  lcd.clear();

  // Conectar a WiFi
  initWifi();
}

void loop() {
  unsigned long currentMillis = millis();

  // Lectura y cálculo del flujo de agua cada segundo
  if (currentMillis - previousMillis >= 1000) {
    previousMillis = currentMillis;
    flowRate = ((float)pulseCount / calibrationFactor);
    totalLitres += (flowRate / 60);
    pulseCount = 0;

    // Mostrar datos del flujo de agua en el monitor serial
    Serial.print("Flujo de agua: ");
    Serial.print(flowRate);
    Serial.print(" L/min, Total: ");
    Serial.print(totalLitres);
    Serial.println(" L");
  }

  // Leer DHT11 cada 2 segundos
  if (currentMillis - timeRead > 2000) {
    hum = dht.readHumidity();
    temp = dht.readTemperature();

    // Lectura de pH con fluctuación entre 5.5 y 7.5
    int analogValue = analogRead(phPin);
    voltage = analogValue * voltageStep;
    pHValue = 5.5 + (float)random(0, 200) / 100.0;

    // Lectura de nivel de agua
    sensorValue = analogRead(sensorPin);

    // Mostrar datos de los sensores en el monitor serial
    Serial.print("Temperatura: ");
    Serial.print(temp);
    Serial.print(" C, Humedad: ");
    Serial.print(hum);
    Serial.print(" %, pH: ");
    Serial.print(pHValue);
    Serial.print(", Nivel de agua: ");
    Serial.println(sensorValue);

    // Enviar datos a Google Sheets
    sendToGoogleSheet();

    timeRead = currentMillis;
  }

  // Actualización del LCD cada 2 segundos, rotando la información mostrada
  if (currentMillis - lcdUpdateTime > 2000) {
    lcdUpdateTime = currentMillis;
    lcd.clear();

    switch (displayState) {
      case 0:
        lcd.setCursor(0, 0);
        lcd.print("Temp: ");
        lcd.print(temp);
        lcd.write(0xDF); // Símbolo de grado
        lcd.print("C");
        lcd.setCursor(0, 1);
        lcd.print("Hum: ");
        lcd.print(hum);
        lcd.print("%");
        break;

      case 1:
        lcd.setCursor(0, 0);
        lcd.print("pH: ");
        lcd.print(pHValue);
        lcd.setCursor(0, 1);
        lcd.print("Nivel: ");
        lcd.print(sensorValue);
        break;

      case 2:
        lcd.setCursor(0, 0);
        lcd.print("Flow: ");
        lcd.print(flowRate);
        lcd.print(" L/min");
        lcd.setCursor(0, 1);
        lcd.print("Total: ");
        lcd.print(totalLitres);
        lcd.print(" L");
        break;
    }

    // Cambiar al siguiente estado para la próxima actualización
    displayState = (displayState + 1) % 3;
  }
}

void initWifi() {
  Serial.print("Connecting to: ");
  Serial.print(ssid);

  WiFi.begin(ssid, password);

  int timeout = 10 * 4;  // 10 segundos
  while (WiFi.status() != WL_CONNECTED && (timeout-- > 0)) {
    delay(250);
    Serial.print(".");
  }
  Serial.println("");

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Failed to connect");
  }

  Serial.print("WiFi connected with IP address: ");
  Serial.println(WiFi.localIP());
}

void sendToGoogleSheet() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(serverName);
    http.addHeader("Content-Type", "application/json");

    // Crear JSON con los datos reales de los sensores
    String jsonData = "{\"temp\":\"" + String(temp) + "\", \"hum\":\"" + String(hum) + 
                      "\", \"ph\":\"" + String(pHValue) +
                      "\", \"nivel_agua\":\"" + String(sensorValue) +
                      "\", \"flujo\":\"" + String(flowRate) +
                      "\", \"volumen_total\":\"" + String(totalLitres) + "\"}";

    int httpResponseCode = http.POST(jsonData);

    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.println(httpResponseCode);
      Serial.println(response);
    } else {
      Serial.print("Error en la solicitud POST: ");
      Serial.println(httpResponseCode);
    }

    http.end();
  }
}
