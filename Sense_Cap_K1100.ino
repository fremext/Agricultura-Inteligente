#include <Arduino.h>
#include <SensirionI2cSht4x.h>
#include <Wire.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <Adafruit_SGP30.h> // Biblioteca para el sensor SGP30

// Inicializar la pantalla TFT
TFT_eSPI tft = TFT_eSPI(); // Inicializa TFT_eSPI

// Definición de colores
#define TFT_BLACK       0x0000
#define TFT_WHITE       0xFFFF

// Definición de pines
#define FLOW_SENSOR_PIN BCM4  // Pin conectado al sensor de flujo
#define SOIL_SENSOR_PIN A0    // Pin analógico del sensor de humedad del suelo
#define PUMP_PIN BCM17        // Pin para controlar la mini bomba

// Inicializa el sensor SGP30
Adafruit_SGP30 sgp;

// Inicializa el sensor SHT40
SensirionI2cSht4x sht4x;

volatile int pulseCount = 0;  // Variable para contar los pulsos

// Función de interrupción para contar los pulsos del sensor de flujo
void pulseCounter() {
  pulseCount++;
}

void setup() {
  Serial.begin(115200); 
  Wire.begin();

  // Configuración de la pantalla TFT
  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(TFT_DARKGREEN); // Fondo verde oscuro
  
  // Configurar el pin del sensor de flujo y la interrupción
  pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), pulseCounter, FALLING);

  // Inicializar el sensor SHT40
  uint16_t error;
  char errorMessage[256];
  
  uint8_t sensorAddress = 0x44; // Dirección I2C del sensor SHT40
  sht4x.begin(Wire, sensorAddress); // Inicializa el sensor con la dirección I2C correcta

  // Inicializar el sensor SGP30
  if (!sgp.begin()) {
    Serial.println("No se pudo encontrar el sensor SGP30, revisa las conexiones.");
    while (1);
  }
  if (!sgp.IAQinit()) { // Inicializa el sensor SGP30 para medir VOC y eCO2
    Serial.println("Error en la inicialización de IAQ");
    while (1);
  }

  // Configurar el pin de la bomba
  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW); // Asegúrate de que la bomba esté apagada al inicio

delay(1000);

  // Mostrar las etiquetas estáticas en la pantalla TFT
  tft.fillRect(0, 0, 320, 50, TFT_ORANGE); // Encabezado
  tft.setTextColor(TFT_BLACK);
  tft.setTextSize(3);
  tft.setCursor(34, 15);
  tft.print("SenseCAP K1100");

  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(10, 60);
  tft.print("Flujo de Agua: ");
  
  tft.setCursor(10, 85);
  tft.print("Temp Ambiental: ");
  
  tft.setCursor(10, 110);
  tft.print("Hum Ambiental: ");
  
  tft.setCursor(10, 135);
  tft.print("Humedad del Suelo: ");
  
  tft.setCursor(10, 160);
  tft.print("Nivel de Luz Solar: ");
  
  tft.setCursor(10, 185);
  tft.print("CO2 Ambiental: ");
  
  tft.setCursor(10, 210);
  tft.print("VOC Ambiental: ");
}

void loop() {
  // La fórmula para calcular el flujo en litros por minuto
  float flowRate = (pulseCount / 7.5);  // Convertir pulsos a flujo (depende del sensor)
  pulseCount = 0;  // Reiniciar el contador de pulsos

  // Leer temperatura y humedad del sensor SHT40
  uint16_t error;
  char errorMessage[256];
  float temperature, humidity;
  error = sht4x.measureHighPrecision(temperature, humidity);
  if (error) {
    Serial.print("Error en measureHighPrecision(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  }

  // Leer el valor del sensor de humedad del suelo
  int soilMoisture = analogRead(SOIL_SENSOR_PIN);
  
  // Convertir el valor del sensor de humedad del suelo a porcentaje
  int soilMoisturePercent = map(soilMoisture, 0, 1023, 0, 100);

  // Leer el valor del sensor de luz (necesitas definir WIO_LIGHT)
  int lightValue = analogRead(WIO_LIGHT); // Asegúrate de definir WIO_LIGHT

  // Leer los valores del sensor SGP30
  if (sgp.IAQmeasure()) {
    uint16_t eCO2 = sgp.eCO2;
    uint16_t TVOC = sgp.TVOC;

    // Mostrar los datos en la pantalla TFT
    tft.setTextColor(TFT_BLACK);
    static String lastFlujo = "", lastTemperatura = "", lastHumedad = "", lastSuelo = "", lastLuz = "", lastECO2 = "", lastTVOC = "";

    if (String(flowRate) != lastFlujo) {
      tft.fillRect(180, 60, 150, 20, TFT_DARKGREEN);
      tft.setCursor(180, 60);
      tft.print(flowRate);
      tft.print(" L/min");
      lastFlujo = String(flowRate);
    }

    if (String(temperature) != lastTemperatura) {
      tft.fillRect(195, 85, 150, 20, TFT_DARKGREEN);
      tft.setCursor(195, 85);
      tft.print(temperature);
      tft.print(" C");
      lastTemperatura = String(temperature);
    }

    if (String(humidity) != lastHumedad) {
      tft.fillRect(185, 110, 150, 20, TFT_DARKGREEN);
      tft.setCursor(185, 110);
      tft.print(humidity);
      tft.print(" %");
      lastHumedad = String(humidity);
    }
    
    if (String(soilMoisturePercent) != lastSuelo) {
      tft.fillRect(230, 135, 150, 20, TFT_DARKGREEN);
      tft.setCursor(230, 135);
      tft.print(soilMoisturePercent);
      tft.print(" %");
      lastSuelo = String(soilMoisturePercent);
    }

    if (String(lightValue) != lastLuz) {
      tft.fillRect(240, 160, 150, 20, TFT_DARKGREEN);
      tft.setCursor(240, 160);
      tft.print(lightValue);
      tft.print(" %");
      lastLuz = String(lightValue);
    }

    if (String(eCO2) != lastECO2) {
      tft.fillRect(180, 185, 150, 20, TFT_DARKGREEN);
      tft.setCursor(180, 185);
      tft.print(eCO2);
      tft.print(" ppm");
      lastECO2 = String(eCO2);
    }

    if (String(TVOC) != lastTVOC) {
      tft.fillRect(180, 210, 150, 20, TFT_DARKGREEN);
      tft.setCursor(180, 210);
      tft.print(TVOC);
      tft.print(" ppb");
      lastTVOC = String(TVOC);
    }

     // Controlar la mini bomba
    if (soilMoisturePercent <= 30) {
      digitalWrite(PUMP_PIN, HIGH); // Encender la bomba
    } else {
      digitalWrite(PUMP_PIN, LOW);  // Apagar la bomba
    }
  }
  delay(500); // Esperar 0.5 segundos antes de actualizar los datos nuevamente
}
