/********* Blynk + ESP8266 + DHT11 + MAX30100(sim) + MPU6050 + Hand-coded Random Forest *********/
#define BLYNK_TEMPLATE_ID "TMPL3gRsA7Ysc"
#define BLYNK_TEMPLATE_NAME "NODEMCU"
#define BLYNK_AUTH_TOKEN "xhEXMBfrDZMC45RvyGS45K6Yut1Rc8bG"

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// WiFi
char ssid[] = "NNNMD";
char pass[] = "9845931373";

// DHT11
#define DHTPIN  D4
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// MPU6050 (Adafruit)
Adafruit_MPU6050 mpu;

// NOTE: MAX30100 is treated as "simulated HR" here to keep code light:
// you can later replace hrCalc() with real MAX3010x library calls if needed.
float fakeHr = 80.0;

// Blynk timer
BlynkTimer timer;

/*************** SIMPLE HAND-CODED RANDOM FOREST ****************/
// Features: [temp, hum, hr, activity]
// Output: 0 = Healthy, 1 = Disease

// 10 simple "trees" – each tree is just one rule using one feature
// These thresholds roughly follow the earlier project logic. [conversation_history:34][conversation_history:37]
struct SimpleTree {
  uint8_t feature_idx;  // 0=temp,1=hum,2=hr,3=activity
  float   threshold;    // if feature > threshold → disease vote
};

SimpleTree forest[10] = {
  {0, 39.5},  // Tree1: high temp → disease
  {2, 120.0}, // Tree2: high HR   → disease
  {3, 0.30},  // Tree3: high activity (restless) → disease
  {0, 40.0},  // Tree4
  {2, 130.0}, // Tree5
  {3, 0.35},  // Tree6
  {0, 38.9},  // Tree7
  {2, 115.0}, // Tree8
  {3, 0.25},  // Tree9
  {0, 41.0}   // Tree10
};

// Predict using the tiny Random Forest
// returns: class_label (0 or 1)
// confidence: fraction of trees voting for disease
uint8_t rfPredict(float temp, float hum, float hr, float activity, float &confidence) {
  float features[4] = { temp, hum, hr, activity };
  uint8_t disease_votes = 0;

  for (int i = 0; i < 10; i++) {
    uint8_t f = forest[i].feature_idx;
    float   thr = forest[i].threshold;
    if (features[f] > thr) {
      disease_votes++;   // this tree says "disease"
    }
  }

  confidence = disease_votes / 10.0;
  return (disease_votes >= 5) ? 1 : 0;  // majority vote
}

/******************* SENSOR HELPERS ********************/

float readTemperature() {
  float t = dht.readTemperature();
  if (isnan(t)) t = 37.5;  // fallback
  return t;
}

float readHumidity() {
  float h = dht.readHumidity();
  if (isnan(h)) h = 60.0;  // fallback
  return h;
}

// Very simple fake heart rate; replace with MAX30100 readings later if you want. [conversation_history:32]
float readHeartRate() {
  // here just slowly vary around 85
  fakeHr += (random(-5, 6) * 0.5);
  if (fakeHr < 60) fakeHr = 60;
  if (fakeHr > 140) fakeHr = 140;
  return fakeHr;
}

// Compute simple "activity" from MPU6050 acceleration magnitude
float readActivity() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // magnitude of acceleration vector, roughly in g
  float ax = a.acceleration.x / 9.81;
  float ay = a.acceleration.y / 9.81;
  float az = a.acceleration.z / 9.81;
  float mag = sqrt(ax * ax + ay * ay + az * az);

  // normalize a bit (0 to ~1 for normal movement)
  if (mag < 0) mag = 0;
  if (mag > 2) mag = 2;
  return mag; // 0–2 range
}

/******************* MAIN TASK: READ + ML + BLYNK ********************/

void sendDataAndPredict() {
  float temp = readTemperature();
  float hum  = readHumidity();
  float hr   = readHeartRate();
  float act  = readActivity();
  Serial.println("Sent to Blynk");

  // ML prediction
  float confidence = 0.0;
  uint8_t label = rfPredict(temp, hum, hr, act, confidence);

  String status = (label == 1) ? "DISEASE ALERT" : "HEALTHY";

  // Send raw sensor data to Blynk
  Blynk.virtualWrite(V0, temp);      // Temperature
  Blynk.virtualWrite(V1, hum);       // Humidity
  Blynk.virtualWrite(V2, hr);        // Heart rate
  Blynk.virtualWrite(V3, act);       // Activity index

  // ML outputs
  Blynk.virtualWrite(V4, label);           // 0 or 1
  Blynk.virtualWrite(V5, status);          // Text
  Blynk.virtualWrite(V6, confidence * 100); // Confidence %

  // Debug
  Serial.print("Temp: "); Serial.print(temp);
  Serial.print("  Hum: "); Serial.print(hum);
  Serial.print("  HR: ");  Serial.print(hr);
  Serial.print("  Act: "); Serial.print(act);
  Serial.print("  --> Status: "); Serial.print(status);
  Serial.print("  Conf: "); Serial.print(confidence * 100.0);
  Serial.println("%");
}

/******************* SETUP & LOOP ********************/

void setup() {
  Serial.begin(115200);
  delay(1000);

  dht.begin();

  Wire.begin();
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found, using default activity=0.1");
  } else {
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  }

 Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);


  // run every 3 seconds
  timer.setInterval(3000L, sendDataAndPredict);
}

void loop() {
  Blynk.run();
  timer.run();
}