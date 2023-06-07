#include <WiFi.h>
#include "FirebaseESP32.h"
#include <DHT.h>
#include <ros.h>
#include <ArduinoJson.h>

#include <std_msgs/String.h>


ros::NodeHandle nh;

//Variables
int chk;
float hum;
float temp;
String node;

//Constantes
#define DHTPIN 4
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

//Credenciales WIFI
#define WIFI_SSID "WOMBAT_WIFI"
#define WIFI_PASSWORD "Wombat_001"

//Credenciales de Firebase
#define FIREBASE_HOST "https://wombat-db-default-rtdb.firebaseio.com/"
#define FIREBASE_AUTH "yOcQrKVx8H0u1DYz0VK6YACESVDfC8v3tjxW40b0"

//Firebase data object
FirebaseData firebaseData;

std_msgs::String back_msg;
ros::Publisher backmsg("backmsg", &back_msg);


void messageCb(const std_msgs::String& js) {
  digitalWrite(32, HIGH);
  StaticJsonDocument<200> doc;
  const char* json = js.data;
  DeserializationError error = deserializeJson(doc, json);
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }
  int x = doc["pos"][0];
  int y = doc["pos"][1];

  const char* product  = doc["obj"];
  const char* num = doc["ID"];

  char nodoh[60] = "/Product Categories/";
  strcat(nodoh, product);
  strcat(nodoh, "/");
  strcat(nodoh, num);
  strcat(nodoh, "/Hum");

  char nodot[60] = "/Product Categories/";
  strcat(nodot, product);
  strcat(nodot, "/");
  strcat(nodot, num);
  strcat(nodot, "/Temp");

  char nodox[60] = "/Product Categories/";
  strcat(nodox, product);
  strcat(nodox, "/");
  strcat(nodox, num);
  strcat(nodox, "/X");

  char nodoy[60] = "/Product Categories/";
  strcat(nodoy, product);
  strcat(nodoy, "/");
  strcat(nodoy, num);
  strcat(nodoy, "/Y");

  Firebase.setInt(firebaseData, nodoh, hum);
  Firebase.setInt(firebaseData, nodot, temp);
  Firebase.setInt(firebaseData, nodox, x);
  Firebase.setInt(firebaseData, nodoy, y);

  delay(200);

  char nodotmax[60] = "/Product Categories/";
  strcat(nodotmax, product);
  strcat(nodotmax, "/Temperature/Max");

  char nodotmin[60] = "/Product Categories/";
  strcat(nodotmin, product);
  strcat(nodotmin, "/Temperature/Min");

  char nodohmax[60] = "/Product Categories/";
  strcat(nodohmax, product);
  strcat(nodohmax, "/Humidity/Max");

  char nodohmin[60] = "/Product Categories/";
  strcat(nodohmin, product);
  strcat(nodohmin, "/Humidity/Min");

  int hmin;
  int hmax;
  int tmin;
  int tmax;

  Firebase.getInt(firebaseData, nodohmin);
  hmin = firebaseData.intData();
  Firebase.getInt(firebaseData, nodohmax);
  hmax = firebaseData.intData();
  Firebase.getInt(firebaseData, nodotmin);
  tmin = firebaseData.intData();
  Firebase.getInt(firebaseData, nodotmax);
  tmax = firebaseData.intData();

  if (hum < hmin || hum > hmax) {
    digitalWrite(26, HIGH);
  } else {
    digitalWrite(26, LOW);
  }
  if (temp < tmin || temp > tmax) {
    digitalWrite(26, HIGH);
  } else {
    digitalWrite(26, LOW);
  }

  char hmins[30];
  char hmaxs[30];
  char hs[30];
  char tmins[30];
  char tmaxs[30];
  char ts[30];

  sprintf(hmins, "%d", hmin);
  sprintf(hmaxs, "%d", hmax);
  gcvt(hum, 5, hs);
  sprintf(tmins, "%d", tmin);
  sprintf(tmaxs, "%d", tmax);
  gcvt(temp, 5, ts);

  char final_msg[300] = "{\"Temperature\":[";
  strcat(final_msg, tmins);
  strcat(final_msg, ",");
  strcat(final_msg, tmaxs);
  strcat(final_msg, ",");
  strcat(final_msg, ts);
  strcat(final_msg, "],\"Humidity\":[");
  strcat(final_msg, hmins);;
  strcat(final_msg, ",");
  strcat(final_msg, hmaxs);
  strcat(final_msg, ",");
  strcat(final_msg, hs);
  strcat(final_msg, "]}");
  
  back_msg.data = final_msg;
  backmsg.publish(&back_msg);

  delay(1000);
}

ros::Subscriber<std_msgs::String> qr("/WOMBAT/vision/qr", &messageCb);

void setup() {
  Serial.begin(57600);
  dht.begin();
  pinMode(32, OUTPUT); //Amarillo
  pinMode(33, OUTPUT); //Verde
  pinMode(25, OUTPUT); //Azul
  pinMode(26, OUTPUT); //Rojo
  Serial.println();

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Conectando al Wifi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    digitalWrite(25, LOW);
    delay(300);
  }
  digitalWrite(25, HIGH);
  Serial.println();
  Serial.println("Connected!");

  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);

  nh.initNode();
  nh.advertise(backmsg);
  nh.subscribe(qr);
}

void loop() {
  while (!dht.read()) {
    digitalWrite(33, LOW);
    Serial.println("No sensor detected");
  }
  digitalWrite(33, HIGH);
  digitalWrite(32, LOW);
  temp = dht.readTemperature();
  hum = dht.readHumidity();
  nh.spinOnce();
  delay(1);
}