#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <String>
#include <string.h>
#include <FirebaseArduino.h>
#include <ArduinoJson.h>

#include <NTPClient.h>
#include <WiFiUdp.h>
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

SoftwareSerial mySerial(D6, D5); // RX | TX
ESP8266WiFiMulti WiFiMulti;

const byte numChars = 32;
char receivedChars[numChars];
boolean newData = false;

#define FIREBASE_HOST "smarttrash-55130.firebaseio.com"
#define FIREBASE_KEY "dSyqDhjaYVj4nOEuJt3UaBnzijPO4A4AuGzaZ3Da"

void setup() {

  Serial.begin(115200);
  mySerial.begin(115200);

  pinMode(D6, INPUT);
  pinMode(D5, OUTPUT);
  Serial.println();
  delay(10);

  for (uint8_t t = 4; t > 0; t--) {
    Serial.printf("[SETUP] WAIT %d...\n", t);
    Serial.flush();
    delay(1000);
  }

  WiFi.mode(WIFI_STA);
  Serial.print("connecting");

  WiFi.begin("Bbone", "plmlplml");
  
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("connected: ");
  Serial.println(WiFi.localIP());
//  mySerial.println("fuckker");
  timeClient.begin();
  timeClient.setTimeOffset(25200);
  Firebase.begin(FIREBASE_HOST, FIREBASE_KEY);
//  Firebase.set("name", "Wio Link 001");
//  Firebase.pushString("time/close", T());
//  Firebase.pushString("time/open", T());
//  Firebase.setString("recentGarbage","0.25");
//  StaticJsonBuffer<200> jsonBuffer;
//  JsonObject& valueObject = jsonBuffer.createObject();
//  valueObject["quantity"] = "0.60";
//  valueObject["timestamp"] = T();
//  Firebase.push("garbage",valueObject);
}

void loop() {
  if(WiFi.status()== WL_CONNECTED){
    recvWithStartEndMarkers();
    showNewData();
//    if(mySerial.available())    //Checks is there any data in buffer 
//    {
//      // Serial.println("We got:");
//      Serial.println(char(mySerial.read()));  //Read serial data byte and send back to serial monitor
//    }
////    else
////    {
////      mySerial.println("kuy");
//////      Serial.println("."); //Print Hello word every one second
////      delay(200);                      // Wait for a second
////    }
    
//    T();
    delay(1000);
  }
}

String T(){
  while(!timeClient.update()) {
    timeClient.forceUpdate();
  }
  return timeClient.getFormattedDate();
}

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (mySerial.available() > 0 && newData == false) {
        rc = mySerial.read();
        Serial.println(rc);
        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

void showNewData() {
    if (newData == true) {
        Serial.println("This arrived: ");
        Serial.println(receivedChars);
        if(receivedChars[0] == 'O') {
          String timestamp = T();
          Firebase.pushString("time/open", timestamp);
        }else if (receivedChars[0] == 'Q'){
          String timestamp = T();
          Serial.println("QUANTITY for q : " + getQuantity(receivedChars));
          Firebase.setString("recentGarbage",getQuantity(receivedChars));
          Firebase.pushString("time/close", timestamp);
          StaticJsonBuffer<200> jsonBuffer;
          JsonObject& valueObject = jsonBuffer.createObject();
          valueObject["quantity"] = getQuantity(receivedChars);
          valueObject["timestamp"] = T();
          Firebase.push("garbageHistory",valueObject);
        }        
        newData = false;
    }
}

String getQuantity(String s){
  // check integer
  String result = s.substring(2, s.length());
  Serial.println(result);
  return result;
}

String getValue(String data, char separator, int index)
{
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
