/*
 * 
 * Code for SLRC 2019 (Sri lanka Robotic Chalenge - Conducted by Department of Electroni and telecommuniction Engineering, university of Moratuwa)
 * 
 * (Code for Secondary robot)
 * 
 * Developed by Ragavender.S and Rajan.S.F
 * 
 * Team name:- Ro_Mi (Faculty of Engineering, university Of Jaffna)
 * 
 * 
 * Team Members:- 
 *        1.Ragavender S (Team Leader)          (Department of computer Engineering)
 *        2.Rajan S.F                           (Department of computer Engineering)
 *        3.SomeshShehan Y                      (Department of Mechanical Engineering)
 *        4.Sharugan M                          (Department of Mechanical Engineering)
 *        5.Prasanna A                          (Department of Electrical and Electronic Engineering)
 * 
 * 
 */

#include <Servo.h>        //Library for Servo motor
#include <SPI.h>
#include <WiFi.h>

Servo base;// declare servo name type servo

unsigned long tempLastConnectionTime = 0; 

String writeAPIKey = "FQMRPEN7KZCX55MT";
char ssid[] = "vivo 1806";
char pass[] = "rajah1234";
int keyIndex = 0;
int status = WL_IDLE_STATUS;
WiFiClient client1;


IPAddress server1(192, 168, 43,234);


unsigned long lastConnectionTime = 0;
const unsigned long postingInterval = 7L * 1000L;


void setup() {
  Serial.begin(9600);              
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv != "1.1.0") {
    Serial.println("Please upgrade the firmware");
  }
  while (status != WL_CONNECTED) {
    //    Serial.print("Attempting to connect to SSID: ");
    //   Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
    delay(10000);
  }
  printWifiStatus();


  base.attach(8);// attach your servo
  base.writeMicroseconds(1500);
  base.write(0); 
  delay(1000);


  
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
   Serial.print("SSID: ");
    Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
   Serial.print("IP Address: ");
    Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  //  Serial.print("signal strength (RSSI):");
  //  Serial.print(rssi);
  // Serial.println(" dBm");
}

void loop() {

  while (client1.available()) {
    char c1 = client1.read();
    
    Serial.println(c1);
      }




      

      

  if (millis() - lastConnectionTime > postingInterval) {
   
      httpRequest1();
     
      
  }
}



void httpRequest1() {


  client1.stop();

  if (client1.connect(server1, 80)) {
    //      Serial.println("connecting...");
    // send the HTTP PUT request:
//    client1.println("GET /latest.txt HTTP/1.1");
//    client1.println("Host: www.arduino.cc");
//    client1.println("User-Agent: ArduinoWiFi/1.1");
    
    client1.println("Connection: close");
    client1.println();
    lastConnectionTime = millis();
  } else {

  }
}


void moveToGreen(){ 
    base.write(90);   
}

void moveToRed(){ 
    base.write(180); 
}

void moveToBlue(){ 
    base.write(0);  
}


void setPosition(){
  
}
