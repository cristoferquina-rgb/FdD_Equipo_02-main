#include <Wire.h> 
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>

// Pines motor (UN SOLO GIRO)
const int IN1 = 32;   // dirección fija
const int ENA = 25;   // habilitar motor

// LEDs y buzzer
const int LED_ROJO = 4;
const int LED_VERDE = 2;
const int BUZZER = 13;

// Sensores
const int MQ2_PIN = 34;        
const int SENSOR_CORRIENTE = 33;

// Umbrales
const int MQ2_UMBRAL = 250;
const float TEMP_UMBRAL = 35.0;
const float STALL_A = 9.0; // Amperios para declarar atasco

// Filtrado / detección
const int MED_N = 7;
const int AVG_N = 5;
const float IIR_ALPHA = 0.3f;
const int REQUIRED_CONSEC = 7;
const int SAMPLE_MS = 150;

// Retraso inicial
int totalSeg = 90;

// BME280 + LCD
Adafruit_BME280 bme;
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Buffers filtrado
int medBuf[MED_N];
int medIdx = 0;
bool medFull = false;
float avgBuf[AVG_N];
int avgIdx = 0;
bool avgFull = false;
float iirValue = 0.0f;

int consecHigh = 0;
bool motor_atascado = false;
bool app_conectada = false;

// WiFi
const char* ssid = "UPCH_CENTRAL";
const char* password = "CAYETANO2022";

// Servidor web
AsyncWebServer server(80);


// --------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  // Pines motor
  pinMode(IN1, OUTPUT);
  pinMode(ENA, OUTPUT);

  // LEDs y buzzer
  pinMode(LED_ROJO, OUTPUT);
  pinMode(LED_VERDE, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  // Sensores
  pinMode(MQ2_PIN, INPUT);
  pinMode(SENSOR_CORRIENTE, INPUT);

  // BME280
  if (!bme.begin(0x76)) {
    Serial.println("No se encontró BME280");
    while (1);
  }

  // LCD
  lcd.init();
  lcd.backlight();

  // WiFi
  WiFi.begin(ssid, password);
  lcd.setCursor(0,0);
  lcd.print("Conectando WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    lcd.print(".");
  }

  lcd.clear();
  IPAddress ip = WiFi.localIP();

  // Timer de inicio (SE QUEDA)
  for (int i = totalSeg; i >= 0; i--) {
    lcd.setCursor(0,0);
    lcd.print("SmartVent :)      ");
    lcd.setCursor(0,1);
    lcd.print("IP: "); lcd.print(ip);
    lcd.setCursor(0,2);
    lcd.print("Iniciando en: ");
    lcd.print(i); lcd.print("s  ");

    delay(1000);
  }
  lcd.clear();

  // Inicial IIR
  iirValue = analogRead(SENSOR_CORRIENTE);

  // API
  server.on("/api/ultimo", HTTP_GET, [](AsyncWebServerRequest *request){
    bool alerta = (analogRead(MQ2_PIN) >= MQ2_UMBRAL ||
                   bme.readTemperature() >= TEMP_UMBRAL);

    String json = "{";
    json += "\"temperatura\":" + String(bme.readTemperature(),1) + ",";
    json += "\"mq2\":" + String(analogRead(MQ2_PIN)) + ",";
    json += "\"alerta\":" + String(alerta ? "true" : "false") + ",";
    json += "\"atascado\":" + String(motor_atascado ? "true" : "false");
    json += "}";

    request->send(200, "application/json", json);
    app_conectada = true;
  });

  server.begin();
}


// --------------------------------------------------------------------
void loop() {
  float temp = bme.readTemperature();
  int mq2 = analogRead(MQ2_PIN);
  float currentA = readFilteredCurrentA();

  bool alerta = (mq2 >= MQ2_UMBRAL || temp >= TEMP_UMBRAL);

  // LED rojo + buzzer (independiente del motor)
  if (alerta) {
    digitalWrite(LED_ROJO, HIGH);
    digitalWrite(BUZZER, HIGH);
  } else {
    digitalWrite(LED_ROJO, LOW);
    digitalWrite(BUZZER, LOW);
  }

  // Motor solo si hay alerta
  if (alerta && !motor_atascado) startMotor();
  else stopMotor();

  // Detección de atasco
  if (digitalRead(ENA) == HIGH) {
    if (currentA >= STALL_A) consecHigh++;
    else if (consecHigh > 0) consecHigh--;

    if (consecHigh >= REQUIRED_CONSEC) {
      motor_atascado = true;
      stopMotor();

      // Beep especial
      for (int i = 0; i < 3; i++) {
        digitalWrite(BUZZER, HIGH); delay(150);
        digitalWrite(BUZZER, LOW); delay(150);
      }
    }
  } else {
    consecHigh = 0;
  }

  // LED verde = normal
  digitalWrite(LED_VERDE, alerta ? LOW : HIGH);

  // LCD
  lcd.setCursor(0,0);
  lcd.print("SmartVent :)       ");

  lcd.setCursor(0,1);
  lcd.print("Temp: "); lcd.print(temp,1); lcd.print(" C   ");

  lcd.setCursor(0,2);
  lcd.print("MQ2: "); lcd.print(mq2); lcd.print("     ");

  lcd.setCursor(0,3);
  if (alerta) {
  lcd.print("ALARMA ACTIVA     ");
  } else {
  lcd.print("Todo normal       ");
}

delay(SAMPLE_MS);
}




// ---------------- Funciones ----------------
float adcToAmps_fromADCavg(float adc_avg) {
  const float ADC_REF = 3.3;
  const int ADC_MAX = 4095;

  float volts = adc_avg * (ADC_REF / ADC_MAX);
  float amps = fabs((volts - 1.65) / 0.100); // ACS712 20A = 100 mV/A
  return amps;
}

int medianOfBuf() {
  int len = medFull ? MED_N : medIdx;
  if (len == 0) return 0;

  int tmp[len];
  for (int i=0; i<len; i++) tmp[i] = medBuf[i];

  for (int i=0; i<len-1; i++)
    for (int j=i+1; j<len; j++)
      if (tmp[i] > tmp[j]) {
        int t = tmp[i];
        tmp[i] = tmp[j];
        tmp[j] = t;
      }

  return tmp[len/2];
}

float avgOfBuf() {
  int len = avgFull ? AVG_N : avgIdx;
  if (len == 0) return 0;

  float s = 0;
  for (int i=0; i<len; i++) s += avgBuf[i];

  return s / len;
}

float readFilteredCurrentA() {
  int raw = analogRead(SENSOR_CORRIENTE);

  medBuf[medIdx] = raw;
  medIdx = (medIdx+1) % MED_N;
  if (medIdx == 0) medFull = true;

  int med = medianOfBuf();

  avgBuf[avgIdx] = med;
  avgIdx = (avgIdx+1) % AVG_N;
  if (avgIdx == 0) avgFull = true;

  float adc_avg = avgOfBuf();

  iirValue = IIR_ALPHA * adc_avg + (1 - IIR_ALPHA) * iirValue;

  return adcToAmps_fromADCavg(iirValue);
}

void startMotor() {
  digitalWrite(IN1, HIGH);
  digitalWrite(ENA, HIGH);
}

void stopMotor() {
  digitalWrite(IN1, LOW);
  digitalWrite(ENA, LOW);
}
