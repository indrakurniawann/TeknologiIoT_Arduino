#include <Wire.h>
#include <Adafruit_INA219.h>
#include <Servo.h> // Mengimpor pustaka Servo
#include "DHT.h"

// Define pins for TRIG and ECHO
#define TRIG_PIN 4
#define ECHO_PIN 5
#define FAN 6
#define MOTORDC 7
#define GETARAN 8
#define RED_LED 10
#define GREEN_LED 11

Adafruit_INA219 ina219;

Servo myServo;

// Konfigurasi DHT22
#define DHTPIN 9// Pin data DHT22 terhubung ke GPIO 9
#define DHTTYPE DHT22  // Jenis sensor DHT
DHT dht(DHTPIN, DHTTYPE);

float humidity = 0.0;
float temperature = 0.0;
int valueGetaran = 0;

// Variabel timer 
unsigned long previousMillis = 0;
const long interval = 2000;  // Interval pembacaan data sensor (ms)

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  dht.begin();          // Inisialisasi sensor DHT22

  // Set pin modes
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(FAN, OUTPUT);
  pinMode(MOTORDC, OUTPUT);
  pinMode(GETARAN, INPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  // Optional: Test message
  Serial.println("HC-SR04 Distance Measurement");

  if (! ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) { delay(10); }
  }
  // To use a slightly lower 32V, 1A range (higher precision on amps):
  ina219.setCalibration_32V_1A(); 
  // Or to use a lower 16V, 40`0mA range (higher precision on volts and amps):
  // ina219.setCalibration_16V_400mA();
  Serial.println("Measuring voltage and current with INA219 ...");

  myServo.attach(3);
}

void loop() {
  // Send a 10us pulse to TRIG pin
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Read the duration of the echo signal
  long duration = pulseIn(ECHO_PIN, HIGH);

  // Calculate distance in centimeters
  float distance = duration * 0.034 / 2;

  // Print the distance
  // Serial.print("Distance: ");
  // Serial.print(distance);
  // Serial.println(" cm");

  if(distance < 30)
  {
    digitalWrite(MOTORDC, LOW);
  }else {
    digitalWrite(MOTORDC, HIGH);
  }

  // read temperature
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    // Membaca data dari sensor DHT22
    humidity = dht.readHumidity();
    temperature = dht.readTemperature();

    // Validasi pembacaan data
    if (isnan(humidity) || isnan(temperature)) {
      Serial.println("Failed to read from DHT sensor!");
      return;
    }
  }

  // Serial.print("Temperature:   "); Serial.print(temperature); Serial.println(" C");
  // Serial.print("Humidity: "); Serial.print(humidity); Serial.println(" g/kg");

  // Wait before next measurement
  delay(100);

  // read current, load voltage
  float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  float loadvoltage = 0;
  float power_mW = 0;

  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();
  loadvoltage = busvoltage + (shuntvoltage / 1000);
  
  // Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
  // Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
  // Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
  // Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
  // Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");
  // Serial.println("");
  // delay(1000);

    if(temperature > 50)
    {
      digitalWrite(FAN, HIGH);
      myServo.write(180);
    }else
    {
      digitalWrite(FAN, LOW);
      myServo.write(0);
    }

    int statusGetaran = digitalRead(GETARAN); // Membaca nilai dari sensor HW-072
    if (statusGetaran == HIGH) { 
      Serial.println("Bergetarrrrrr");
      valueGetaran = 1;
      digitalWrite(RED_LED, LOW);
      digitalWrite(GREEN_LED, HIGH);
    } else {
      Serial.println("Diamm");
      valueGetaran = 0;
      digitalWrite(RED_LED, HIGH);
      digitalWrite(GREEN_LED, LOW);
    }

  // Serial.print("IOTSENSOR");
  // Serial.print("#");
  // Serial.print(random(0, 10));
  // Serial.print("#");
  // Serial.print(random(10, 20));
  // Serial.print("#");
  // Serial.print(random(20, 30));
  // Serial.print("#");
  // Serial.print(random(30, 40));
  // Serial.print("#");
  // Serial.print(random(40, 50));
  // Serial.print("#");
  // Serial.print(random(50, 60));
  // Serial.print("#");
  // Serial.println(random(60, 70));

  
  Serial.print("IOTSENSOR");
  Serial.print("#");
  Serial.print(temperature);
  Serial.print("#");
  Serial.print(humidity);
  Serial.print("#");
  Serial.print(distance);
  Serial.print("#");
  Serial.print(valueGetaran);
  Serial.print("#");
  Serial.print(current_mA);
  Serial.print("#");
  Serial.println(loadvoltage);
  delay(1000);
}
