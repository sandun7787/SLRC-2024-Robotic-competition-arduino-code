/*
 * 
 * Code for SLRC 2019 (Sri lanka Robotic Chalenge - Conducted by Department of Electroni and telecommuniction Engineering, university of Moratuwa)
 * 
 * (Code for primary robot)
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

  /*
   * Line follower basic code taken from http://42bots.com
   * Details attched below
   */
   
/*
** Line Follower Basic
** Version 0.6
** Last Update: 2013-05-24
** By Stan, http://42bots.com
*/

#include <Servo.h>        //Library for Servo motor
#include <SPI.h>          //Library for WiFi Sheild
#include <WiFi.h>         //Library for WiFi Sheild


/* Define motor controll inputs */
const int motorRPin1 = 2; // signal pin 1 for the right motor, connect to IN1               
const int motorRPin2 = 3;  // signal pin 2 for the right motor, connect to IN2
const int motorREnable = 4; // enable pin for the right motor (PWM enabled)

const int motorLPin1 = 5; // signal pin 1 for the left motor, connect to IN3           
const int motorLPin2 = 6; // signal pin 2 for the left motor, connect to IN4
const int motorLEnable = 7; // enable pin for the left motor (PWM enabled)

/* Define the pins for the IR receivers */
const int irPins[6] = {A0, A1, A2, A3, A4, A5};

/* Define values for the IR Sensor readings */

// an array to hold values from analogRead on the ir sensor (0-1023)
int irSensorAnalog[6] = {0,0,0,0,0,0}; 

// an array to hold boolean values (1/0) for the ir sensors
int irSensorDigital[6] = {0,0,0,0,0,0};

int treashold = 700; // IR sensor treashold value for line detection

// binary representation of the sensor reading 
//from left to right when facing the same direction as the robot
int irSensors = B000000; 
int irLast = B000000;

int count = 0; // number of sensors detecting the line

// A score to determine deviation from the line [-180 ; +180]. 
// Negative means the robot is left of the line.
int error = 0;  

int errorLast = 0;  //  store the last value of error

// A correction value, based on the error from target. 
// It is used to change the relative motor speed with PWM.
int correction = 0; 

int lap = 0; // keep track of the laps

int zone = 0;

/* Set up maximum speed and speed for turning (to be used with PWM) */
int maxSpeed = 255; // used for PWM to control motor speed [0 - 255]

/* variables to keep track of current speed of motors */
int motorLSpeed = 0;
int motorRSpeed = 0;


int row = 1;                    //row number of Grids. Initialized to 1;
int colomn = 0;                 //colomn number of Grids. Initialized to 1;

int rowP=0;                     //Prvios row of current position of robot
int colomnP=0;                  //Prvios colomn of current position of robot



//for color detection

//pins of Colour sensor TCS 230
const int S0=16;
const int S1=17;
const int S2=18;
const int S3=19;
const int sensorOut=20;

int frequency = 0;

// Pins for LEDs for showing colours of boxes
int ledR = 13;
int ledG = 14;
int ledB = 15;

// Pins for LEDs for showing side of pillers
int ledLW = 21;
int ledRW = 22;


int dataR=0;
int dataG=0;
int dataB=0;

int green[3];
int blue[3];
int red[3];

int gCount=0;
int rCount=0;
int bCount=0;

char colourN[9][9];

int colour[9][9];
int number = 0;
int increment=1;


int gT=0;
int rT=0;
int bT=0;

char sort[3]={'r','g','b'};           //sortd order of colours

int rC=0;
int gC=0;
int bC=0;

int Rr=0;
int Cr=0;
int Rg=0;
int Cg=0;
int Rb=0;
int Cb=0;


int bV=0;
int gV=0;


//for Ultra sonic sensors
const int trigPinR = 9;
const int echoPinR = 10;

const int trigPinL = 11;
const int echoPinL = 12;


long durationR;         //Duration for sound recieved by reciver of right side ultra sonic sensor
long durationL;         //Duration for sound recieved by reciver of left side ultra sonic sensor

int distanceR;          //Distance of right side piller
int distanceL;          //Distance of left side piller

int lineValue=0;    //could be 10,20,30,40 according to positions specified in task

int obsPoints=0;      //can be 1 or 2 according to the side specified in task


boolean rTrue=false;    
boolean bTrue=false;
boolean gTrue=false;


int transData=0;            //red 1, green 2, blue 3, dropped secondary 4, again connect 5

Servo lock;               // servo for lock which was attached the primarry and secondary robot
Servo redP;               // Servo for push red coloured coin into the correct box
Servo greenP;             // Servo for push green and blue coloured coin into the correct box






//for wifi sheild

char ssid[] = "vivo 1806";      
char pass[] = "rajah1234";   
int keyIndex = 0;                
int status = WL_IDLE_STATUS;

WiFiServer server(80);



void setup() {

  /* Set up motor controll pins as output */
  pinMode(motorLPin1,OUTPUT);        
  pinMode(motorLPin2,OUTPUT);
  pinMode(motorLEnable,OUTPUT);
  
  pinMode(motorRPin1,OUTPUT);        
  pinMode(motorRPin2,OUTPUT);
  pinMode(motorREnable,OUTPUT);
   
  /* Set-up IR sensor pins as input */
  for (int i = 0; i < 6; i++) {
    pinMode(irPins[i], INPUT);
  }
  
  
  TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM01) | _BV(WGM00); 
  TCCR0B = _BV(CS00); 
    
  /* Set-up console output for debugging.  */
  Serial.begin(115200);

  //for color detection

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);

  pinMode(ledR, OUTPUT);
  pinMode(ledG, OUTPUT);
  pinMode(ledB, OUTPUT);
  

  //setting frequency scale
  digitalWrite(S0,HIGH);
  digitalWrite(S1,LOW);

//  Serial.begin(9600);
  for(int i=0;i<9;i++){
    for(int j=0;j<9;j++){
      colourN[i][j]='w';
    }
  }


  pinMode(trigPinR,OUTPUT);
  pinMode(echoPinR,INPUT);
  pinMode(trigPinL,OUTPUT);
  pinMode(echoPinL,INPUT);


  lock.attach(25);// attach your servo
  lock.writeMicroseconds(1500);
  lock.write(0); 
  delay(1000);

  redP.attach(26);// attach your servo
  redP.writeMicroseconds(1500);
  redP.write(90); 
  delay(1000);

  greenP.attach(26);// attach your servo
  greenP.writeMicroseconds(1500);
  greenP.write(90); 
  delay(1000);


//  //for wifi sheild
//   Serial.begin(9600);
  while (!Serial) {
    
  }

 if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
  
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv != "1.1.0") {
    Serial.println("Please upgrade the firmware");
  }

  
  while (status != WL_CONNECTED) {
 Serial.println("Attempting to connect to SSID . If red bulb blinks then check your hotspot whether its On or Off");
 Serial.println(ssid);
   
    status = WiFi.begin(ssid, pass);
    printWifiStatus();
     Serial.println(server.available());
     if(server.available())
     {Serial.println("Connected");}
  }
  server.begin();
  
//
//  
}
//
//


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
  if (zone == 0) {                //in Stating box
    Scan();
    while (irSensors == B111111){
      motorRSpeed = maxSpeed;
      motorLSpeed = maxSpeed;
      Drive();
      Scan();
      }
  motorRSpeed = 0;
  motorLSpeed = 0;
  Drive();
 
  zone = 1; 
  }

   if (zone == 1) {      //Line following until 90 degree
    Scan();
    UpdateError();
    UpdateCorrection();
    Drive();
    if(error==150){
      zone=2;
    }
  }

  if(zone==2){          //after 90 degree untill go to the dropping box
    Scan();
    UpdateError();
    UpdateCorrection();
    if(irSensors==B111111){
      motorRSpeed=0;
      motorLSpeed=0;
      zone=3;
    }
    Drive();
  }

  if(zone==3){          //Turning 180 degree on dropping box for droping the secondary robot
    while(irSensors != B001100){
    Scan();
    motorRSpeed = -(maxSpeed);
    motorLSpeed = maxSpeed;
    Drive();
      
    }
    motorRSpeed =0;
    motorLSpeed = 0;
    Drive();
    zone=4;
    removeSecond();
  }

  if(zone==4){          //After turning 
    Scan();
    UpdateError();
    if(irSensors==B111111 || irSensors==B000000){
      error==150;
      zone=5;
    }
    UpdateCorrection();
    Drive();
  }

  if(zone==5){            //moving middle of pillers
    UltraScan();
    calcObsPoints();
    Scan();
    UpdateError();
    if(error==150 || error==-150){
      error=0;
    }
    if((irLast==B001111 || irLast==B000111) && (irSensors!=B001111 || irSensors!=B000111)){
      zone=6;
    }
    UpdateCorrection();
    Drive();
  }

  if(zone==6){            //calculating dropping places
    whichBlue();
    whichGreen();
    whichRed();
    zone=10;
  }

  if(zone==10){           // entered to the grid
    Scan();
    UpdateError();
    if(irSensors==B001111 && irLast!=B001111){
      colomn++;
      number=number+increment;
      colour[row][colomn]=number;
//      Serial.println(colomn);           //for debugging purpose
    }
    if(irSensors==B000000){
      zone=11;              //finished 1st row and turning to the next row
      row++;
      colomn++;
//      Serial.println(row);            //fordebugging purpose
    }
    if(error==150){
      error=0;
    }
    UpdateCorrection();
    ScanColour();
    blinkLED();
    Drive();
  }

  if(zone==11){           //Turning from 1st row
    Scan();
    UpdateError();
    if(irSensors==B000000){
      error=150;
    }
    if(irSensors!=B000000){
      zone=12;      //on second row
    }
    UpdateCorrection();
    Drive();
  }

  if(zone==12){       //Start to move on second row.
    Scan();
    UpdateError();
    if(errorLast==0 && irSensors==B111111 && irLast!=B111111){
      colomn--;
      number=number+increment;
      colour[row][colomn]=number;
//      Serial.println(colomn);
    }
    if(irLast==B111111 && irSensors==B000000){
      zone=13;      //turning to 3rd row;
      row++;
      colomn=0;
      error=-150;
//      Serial.println(row)
    }
    UpdateCorrection();
    ScanColour();
    blinkLED();
    Drive();

    if(row>8){                  //may need to change
      zone=14;
    }
  }

   if(zone==13){       //Start to move on 3rd row.
    Scan();
    UpdateError();
    if(irSensors==B111111 && irLast!=B111111){
      colomn++;
      number=number+increment;
      colour[row][colomn]=number;
//      Serial.println(colomn);
    }
    if(irLast==B111111 && irSensors==B000000){
      zone=11;      //turning to 3rd row;
      error=150;
      row++;
//      Serial.println(row);
    }
    UpdateCorrection();
    ScanColour();
    blinkLED();
    Drive();
  }

  if(zone=14){        //coin dropping zone
    findRedColomn();
    findRedRow();
    goTo1Row();
    goTo1Colomn();
    
    findBlueColomn();
    findBlueRow();
    goTo2Row();
    goTo2Colomn();
    
    findGreenColomn();
    findGreenRow();
    goTo3Row();
    goTo3Colomn();
    
    findEndRow();
    findEndColomn();
    goToEndRow();
    goToEndColomn();
    
    zone=15;
    
  }

  if(zone==15){           //moving out from grid
    Scan();
    UpdateError();
    if(error==150 || -150){
      error=0;
    }
    if((irSensors!=B001111 || irSensors!=B000111 || irSensors!=B111111) && (irLast==B001111 || irLast==B000111 || irLast==B111111)){
      zone=16;
    }
    UpdateCorrection();
    Drive();
    
  }

  if(zone==16){                 //move to ending box
    Scan();
    UpdateError();
    UpdateCorrection();
    Drive();

    if(irSensors==B111111){
      zone=17;
  }

  if (zone ==17){           //on ending box
  int len = 150;
  int i = 0;
  while(i< len){
    motorRSpeed = maxSpeed;
    motorLSpeed = maxSpeed;
    Drive();
    }
  motorRSpeed = 0;
  motorLSpeed = 0;
  Drive();
  }
}
  
  
}

void Scan() {
  // Initialize the sensor counter and binary value
  count = 0;
  irLast=irSensors;
  irSensors = B000000;
    
  for (int i = 0; i < 6; i++) {
    irSensorAnalog[i] = analogRead(irPins[i]);

    if (irSensorAnalog[i] >= treashold) {
        irSensorDigital[i] = 1;
    }
    else {irSensorDigital[i] = 0;}
    //Serial.print(irSensorAnalog[i]);
    //Serial.print("|");
    count = count + irSensorDigital[i];
    int b = 5-i;
    irSensors = irSensors + (irSensorDigital[i]<<b);
    }    
}

void UpdateError() {
  
  errorLast = error;  
  
  switch (irSensors) {
     
    case B000000:
       if (errorLast < 0) { error = -180;}
       else if (errorLast > 0) {error = 180;}
       break;
     
     case B100000: // leftmost sensor on the line
       error = -150;
       break;
      
     case B010000:
       error = -90;
       break;

     case B001000: 
       error = -30;
       break;

     case B000100:  
       error = 30;
       break;
       
     case B000010: 
       error = 90;
       break;           

     case B000001: // rightmost sensor on the line
       error = 150;
       break;
       
/* 2 Sensors on the line */         
     
     case B110000:
       error = -120;
       break;
      
     case B011000:
       error = -60;
       break;

     case B001100: 
       error = 0;
       break;

     case B000110: 
       error = 60;
       break;           

     case B000011:
       error = 120;
       break;

/* 3 Sensors on the line */    
       
     case B111000:
     case B011100:
       error = -150;
       break;
      
     case B000111:
     case B001110:
       error = 150;
       break;

 /* 4 Sensors on the line */       
     case B111100:
       error = -150;
       break;
       
     case B111010:
       error = -150;
       break;
      
     case B001111:
       error = 150;
       break;
       
     case B010111:
       error = 150;
       break;

/* 5 Sensors on the line */      
     case B111110:
       error = -150;
       break;
      
     case B011111:
       error = +150;
       break;
      
     case B111111:
       lap = lap + 1;
       error = 0;
       break;
   
     default:
       error = errorLast;
//      Serial.print("Unhandled case: ");
//      Serial.println(count);
//      Serial.print("| ");
//      Serial.println(irSensors);    
  }
}

void UpdateCorrection() {

  if (error >= 0 && error < 30) {
    correction = 0;
  }
  
  else if (error >=30 && error < 60) {
    correction = 15;
  }
  
  else if (error >=60 && error < 90) {
    correction = 40;
  }
  
  else if (error >=90 && error < 120) {
    correction = 55;
  }  
  
  else if (error >=120 && error < 150) {
    correction = 75;
  } 
  
  else if (error >=150 && error < 180) {
    correction = 255;
  }   

  else if (error >=180) {
    correction = 305;
  }

  if (error <= 0 && error > -30) {
    correction = 0;
  }
  
  else if (error <= -30 && error > -60) {
    correction = -15;
  }
  
  else if (error <= -60 && error > -90) {
    correction = -40;
  }
  
  else if (error <= -90 && error > -120) {
    correction = -55;
  }  
  
  else if (error <= -120 && error > -150) {
    correction = -75;
  } 
  
  else if (error <= -150 && error > -180) {
    correction = -255;
  }   

  else if (error <= -180) {
    correction = -305;
  }
  
  /* Adjust the correction value if maxSpeed is less than 255 */
  correction = (int) (correction * maxSpeed / 255 + 0.5);
  
  if (correction >= 0) {
    motorRSpeed = maxSpeed - correction;
    motorLSpeed = maxSpeed;
  }
  
  else if (correction < 0) {
    motorRSpeed = maxSpeed;
    motorLSpeed = maxSpeed + correction;
  }
}

void Drive() {
  if (motorRSpeed > 255) {motorRSpeed = 255;}
  else if (motorRSpeed < -255) {motorRSpeed = -255;}
  
  if (motorLSpeed > 255) {motorLSpeed = 255;}
  else if (motorLSpeed < -255) {motorLSpeed = -255;}
  
  if (motorRSpeed > 0) { // right motor forward (using PWM)
     analogWrite(motorREnable, motorRSpeed);
     digitalWrite(motorRPin1, HIGH);
     digitalWrite(motorRPin2, LOW);
  } 
  
  else if (motorRSpeed < 0) {// right motor reverse (using PWM)
     analogWrite(motorREnable, abs(motorRSpeed));
     digitalWrite(motorRPin1, LOW);
     digitalWrite(motorRPin2, HIGH);
  } 
  
  else if (motorRSpeed == 0) { // right motor fast stop
     digitalWrite(motorREnable, HIGH);
     digitalWrite(motorRPin1, LOW);
     digitalWrite(motorRPin2, LOW);
  }
  
  if (motorLSpeed > 0) { // left motor forward (using PWM)
     analogWrite(motorLEnable, motorLSpeed);
     digitalWrite(motorLPin1, HIGH);
     digitalWrite(motorLPin2, LOW);
  } 
  
  else if (motorLSpeed < 0) { // right motor reverse (using PWM)
     analogWrite(motorLEnable, abs(motorLSpeed));
     digitalWrite(motorLPin1, LOW);
     digitalWrite(motorLPin2, HIGH);
  } 
  
    else if (motorLSpeed == 0) { // left motor fast stop
     digitalWrite(motorLEnable, HIGH);
     digitalWrite(motorLPin1, LOW);
     digitalWrite(motorLPin2, LOW);
  }
}




//for color detection
bool isRed()          //find is it red box?
{
  if((dataR<80  && dataR>50 ) && (dataG<90 && dataG>60 ) && (dataB<120 && dataB>90 )&& ((irSensors==B111111 && irLast!=B111111) || (irSensors==B001111 && irLast!=B001111) || (irSensors==B111100 && irLast!=B111100)) )
  {
  return true;
  }
  else
  {
    return false;
  }
  }
  
bool isGreen()        //find is it green box?
{
  if((dataR<230  && dataR>160 ) && (dataG<115 && dataG>80 ) && (dataB<140 && dataB>115 ) && ((irSensors==B111111 && irLast!=B111111) || (irSensors==B001111 && irLast!=B001111) || (irSensors==B111100 && irLast!=B111100)) )
  {
  return true;
  }
  else
  {
    return false;
  }
  }

  bool isBlue()         //find is it blue box?
{
  if((dataR<170  && dataR>130 ) && (dataG<70 && dataG>60 ) && (dataB<120 && dataB>100 ) && ((irSensors==B111111 && irLast!=B111111) || (irSensors==B001111 && irLast!=B001111) || (irSensors==B111100 && irLast!=B111100)) )
  {
  return true;
  }
  else
  {
    return false;
  }
  }



  void ScanColour(){              //Getting readings from colour sensors
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  dataR=pulseIn(sensorOut,LOW);
  delay(20);

  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  dataG=pulseIn(sensorOut,LOW);
  delay(20);

  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  dataB=pulseIn(sensorOut,LOW);
  delay(20);
   
  }
  

  void blinkLED(){                //showing box colour using colour LED
    if(isRed() && colourN[row-1][colomn]!='r'){
      colourN[row][colomn]='r';
      increment=2;
      red[rCount]=number;
      rCount++;
      digitalWrite(ledR,HIGH);
      delay(500);
      digitalWrite(ledR,LOW);
    }

    if(isGreen() && colourN[row-1][colomn]!='g'){
      colourN[row][colomn]='g';
      increment=3;
      green[gCount]=number;
      gCount++;
      digitalWrite(ledG,HIGH);
      delay(500);
      digitalWrite(ledG,LOW);
    }

    if(isBlue() && colourN[row-1][colomn]!='b'){
      colourN[row][colomn]='b';
      increment=4;
      blue[bCount]=number;
      bCount++;
      digitalWrite(ledB,HIGH);
      delay(500);
      digitalWrite(ledB,LOW);
    }
  }

  void isFirst(){                   //finding sorted order of coin dropping
    for(int i=0;i<3;i++){
      gT=gT+pow(green[i],(3-i));
    }

    for(int i=0;i<3;i++){
      rT=rT+pow(red[i],(3-i));
    }

    for(int i=0;i<3;i++){
      bT=bT+pow(blue[i],(3-i));
    }

    if(gT<=rT && gT<=bT && bT<=rT){      //gT<bT<rT
      sort[0]='g';
      sort[1]='b';
      sort[2]='r';
    }else if(gT<=rT && gT<=bT && bT<=rT){   //gT<rT<bT
      sort[0]='g';
      sort[1]='r';
      sort[2]='b';
    }else if(rT<=gT && rT<=bT && gT<=bT){   //rT<gT<bT
      sort[0]='r';
      sort[1]='g';
      sort[2]='b';
    }else if(rT<=gT && rT<=bT && bT<=gT){       //rT<bT<gT
      sort[0]='r';
      sort[1]='b';
      sort[2]='g';
    }else if(bT<=gT && bT<=rT && gT<=rT){   //bT<gT<rT
      sort[0]='b';
      sort[1]='g';
      sort[2]='r';
    }else{                //bT<rT<gT
      sort[0]='b';
      sort[1]='r';
      sort[2]='g';
    }    
  }

  

void searchRed(){           //finding red coloured box's row and colomn number
  for(int i=0;i<8;i++){
    for(int j=0;j<8;j++){
      if(colourN[i][j] == 'r'){
        if(rC==1){
        Rr=i;
        Cr=j;
        break;
      }else{
        rC--;
      }
    }
  }
}
}
  

void searchGreen(){         //finding green coloured box's row and colomn number
  for(int i=0;i<8;i++){
    for(int j=0;j<8;j++){
      if(colourN[i][j] == 'g'){
        if(gC==1){
         Rg=i;
        Cg=j;
        break; 
        }
      }else{
        gC--;
      }
    }
  }
}



void searchBlue(){                   //finding blue coloured box's row and colomn number
  for(int i=0;i<8;i++){
    for(int j=0;j<8;j++){
      if(colourN[i][j] == 'b'){
        if(bC==1){
          Rb=i;
          Cb=j;
          break;
        }
       
      }else{
        bC--;
      }
    }
  }
}


void findRedRow(){            //
  Scan();
  UpdateError();
  if(Rr>row && colomnP<colomn){
   error=150;
  }else if(Rr>row && colomnP>colomn){
    error=-150;
  }else if(Rr<row && colomnP<colomn){
    error=-150;
  }else if(Rr<row && rowP>row && colomnP>colomn){
    error=150;
  }
  UpdateCorrection;
  Drive();
}

void findRedColomn(){
  Scan();
  UpdateError();
  if(Cr>colomn && rowP<row){
    error=-150;
  }else if(Cr>colomn && rowP>row){
    error=150;
  }else if(Cr<colomn && rowP<row){
    error=150;
  }else if(Cr<colomn && rowP>row){
    error=-150;
  }
  UpdateCorrection();
  Drive();
}


void findGreenRow(){
  Scan();
  UpdateError();
  if(Rg>row && colomnP<colomn){
   error=150;
  }else if(Rg>row && colomnP>colomn){
    error=-150;
  }else if(Rg<row && colomnP<colomn){
    error=-150;
  }else if(Rg<row && rowP>row && colomnP>colomn){
    error=150;
  }
  UpdateCorrection();
  Drive();
  
}

void findGreenColomn(){
  Scan();
  UpdateError();
  if(Cg>colomn && rowP<row){
    error=-150;
  }else if(Cg>colomn && rowP>row){
    error=150;
  }else if(Cg<colomn && rowP<row){
    error=150;
  }else if(Cg<colomn && rowP>row){
    error=-150;
  }
  UpdateCorrection();
  Drive();
  
}

void findBlueRow(){
  Scan();
  UpdateError();
  if(Rb>row && colomnP<colomn){
   error=150;
  }else if(Rb>row && colomnP>colomn){
    error=-150;
  }else if(Rb<row && colomnP<colomn){
    error=-150;
  }else if(Rb<row && rowP>row && colomnP>colomn){
    error=150;
  }
  UpdateCorrection();
  Drive();
}

void findBlueColomn(){
  Scan();
  UpdateError();
  if(Cb>colomn && rowP<row){
    error=-150;
  }else if(Cb>colomn && rowP>row){
    error=150;
  }else if(Cb<colomn && rowP<row){
    error=150;
  }else if(Cb<colomn && rowP>row){
    error=-150;
  }
  UpdateCorrection();
  Drive();
}



void goTo1Row(){
  if(sort[0]=='r'){
    while(Rr!=row){
    Scan();
    UpdateError();
    if(errorLast==0 && ((irSensors==B111111 && irLast!=B111111)||(irSensors==B001111 && irLast!=B001111)||(irSensors==B000111 && irLast!=B000111)) && row>Rr){
      row--;
    }
    if(errorLast==0 && ((irSensors==B111111 && irLast!=B111111)||(irSensors==B001111 && irLast!=B001111)||(irSensors==B000111 && irLast!=B000111)) && row<Rr){
      row++;
    }
    if(error==150|| error==-150){
      error=0;
    }
    UpdateCorrection();
    Drive();
    }
  }else if(sort[0]=='g'){
    while(Rg!=row){
    Scan();
    UpdateError();
    if(errorLast==0 && ((irSensors==B111111 && irLast!=B111111)||(irSensors==B001111 && irLast!=B001111)||(irSensors==B000111 && irLast!=B000111)) && row>Rg){
      row--;
    }else if(errorLast==0 && ((irSensors==B111111 && irLast!=B111111)||(irSensors==B001111 && irLast!=B001111)||(irSensors==B000111 && irLast!=B000111)) && row<Rg){
      row++;
    }
    if(error==150|| error==-150){
      error=0;
    }
    UpdateCorrection();
    Drive();
    }
    
  }else if(sort[0]=='b'){
    while(Rb!=row){
    Scan();
    UpdateError();
    if(errorLast==0 && ((irSensors==B111111 && irLast!=B111111)||(irSensors==B001111 && irLast!=B001111)||(irSensors==B000111 && irLast!=B000111)) && row>Rb){
      row--;
    }else if(errorLast==0 && ((irSensors==B111111 && irLast!=B111111)||(irSensors==B001111 && irLast!=B001111)||(irSensors==B000111 && irLast!=B000111)) && row<Rb){
      row++;
    }
    if(error==150|| error==-150){
      error=0;
    }
    UpdateCorrection();
    Drive();
    }
    
  }

}




void goTo1Colomn(){
  if(sort[0]=='r'){
    while(Cr!=colomn){
    Scan();
    UpdateError();
    if(errorLast==0 && ((irSensors==B111111 && irLast!=B111111)||(irSensors==B001111 && irLast!=B001111)||(irSensors==B000111 && irLast!=B000111)) && colomn<Cr){
      colomn++;
    }else if(errorLast==0 && ((irSensors==B111111 && irLast!=B111111)||(irSensors==B001111 && irLast!=B001111)||(irSensors==B000111 && irLast!=B000111)) && colomn>Cr){
      colomn--;
    }
    if(error==150|| error==-150){
      error=0;
    }
    UpdateCorrection();
    Drive();
    }

    if(rTrue==true){
      turningForDropR();
    }

    transData=1;
    pushRed();
    WiFiTrans();
    
  }else if(sort[0]=='g'){
    while(Cg!=colomn){
    Scan();
    UpdateError();
    if(errorLast==0 && ((irSensors==B111111 && irLast!=B111111)||(irSensors==B001111 && irLast!=B001111)||(irSensors==B000111 && irLast!=B000111)) && colomn<Cg){
      colomn++;
    }else if(errorLast==0 && ((irSensors==B111111 && irLast!=B111111)||(irSensors==B001111 && irLast!=B001111)||(irSensors==B000111 && irLast!=B000111)) && colomn>Cg){
      colomn--;
    }
    if(error==150|| error==-150){
      error=0;
    }
    UpdateCorrection();
    Drive();
    }if(gTrue==true){
      turningForDropG();
    }

    transData=2;
    pushGreen();
    WiFiTrans();
    
  }else if(sort[0]=='b'){
    while(Cb!=colomn){
    Scan();
    UpdateError();
    if(errorLast==0 && ((irSensors==B111111 && irLast!=B111111)||(irSensors==B001111 && irLast!=B001111)||(irSensors==B000111 && irLast!=B000111)) && colomn<Cb){
     colomn++;
    }else if(errorLast==0 && ((irSensors==B111111 && irLast!=B111111)||(irSensors==B001111 && irLast!=B001111)||(irSensors==B000111 && irLast!=B000111)) && colomn>Cb){
     colomn--;
    }
    if(error==150|| error==-150){
      error=0;
    }
    UpdateCorrection();
    Drive();
    }if(bTrue==true){
      turningForDropB();
    }
    
  }

  transData=3;
  pushBlue();
  WiFiTrans();

}

void goTo2Row(){
  if(sort[1]=='r'){
    while(Rr!=row){
    Scan();
    UpdateError();
    if(errorLast==0 && ((irSensors==B111111 && irLast!=B111111)||(irSensors==B001111 && irLast!=B001111)||(irSensors==B000111 && irLast!=B000111)) && row>Rr){
      row--;
    }
    if(errorLast==0 && ((irSensors==B111111 && irLast!=B111111)||(irSensors==B001111 && irLast!=B001111)||(irSensors==B000111 && irLast!=B000111)) && row<Rr){
      row++;
    }
    if(error==150|| error==-150){
      error=0;
    }
    UpdateCorrection();
    Drive();
    }
  }else if(sort[1]=='g'){
    while(Rg!=row){
    Scan();
    UpdateError();
    if(errorLast==0 && ((irSensors==B111111 && irLast!=B111111)||(irSensors==B001111 && irLast!=B001111)||(irSensors==B000111 && irLast!=B000111)) && row>Rg){
      row--;
    }else if(errorLast==0 && ((irSensors==B111111 && irLast!=B111111)||(irSensors==B001111 && irLast!=B001111)||(irSensors==B000111 && irLast!=B000111)) && row<Rg){
      row++;
    }
    if(error==150|| error==-150){
      error=0;
    }
    UpdateCorrection();
    Drive();
    }
    
  }else if(sort[1]=='b'){
    while(Rb!=row){
    Scan();
    UpdateError();
    if(errorLast==0 && ((irSensors==B111111 && irLast!=B111111)||(irSensors==B001111 && irLast!=B001111)||(irSensors==B000111 && irLast!=B000111)) && row>Rb){
      row--;
    }else if(errorLast==0 && ((irSensors==B111111 && irLast!=B111111)||(irSensors==B001111 && irLast!=B001111)||(irSensors==B000111 && irLast!=B000111)) && row<Rb){
      row++;
    }
    if(error==150|| error==-150){
      error=0;
    }
    UpdateCorrection();
    Drive();
    }
    
  }

}

void goTo2Colomn(){
  if(sort[1]=='r'){
    while(Cr!=colomn){
    Scan();
    UpdateError();
    if(errorLast==0 && ((irSensors==B111111 && irLast!=B111111)||(irSensors==B001111 && irLast!=B001111)||(irSensors==B000111 && irLast!=B000111)) && colomn<Cr){
      colomn++;
    }else if(errorLast==0 && ((irSensors==B111111 && irLast!=B111111)||(irSensors==B001111 && irLast!=B001111)||(irSensors==B000111 && irLast!=B000111)) && colomn>Cr){
      colomn--;
    }
    if(error==150|| error==-150){
      error=0;
    }
    UpdateCorrection();
    Drive();
    }
    if(rTrue==true){
      turningForDropR();
    }

    transData=1;
    pushRed();
    WiFiTrans();
    
  }else if(sort[1]=='g'){
    while(Cg!=colomn){
    Scan();
    UpdateError();
    if(errorLast==0 && ((irSensors==B111111 && irLast!=B111111)||(irSensors==B001111 && irLast!=B001111)||(irSensors==B000111 && irLast!=B000111)) && colomn<Cg){
      colomn++;
    }else if(errorLast==0 && ((irSensors==B111111 && irLast!=B111111)||(irSensors==B001111 && irLast!=B001111)||(irSensors==B000111 && irLast!=B000111)) && colomn>Cg){
      colomn--;
    }
    if(error==150|| error==-150){
      error=0;
    }
    UpdateCorrection();
    Drive();
    }
    if(gTrue==true){
      turningForDropG();
    }

    transData=2;
    pushGreen();
    WiFiTrans();
    
  }else if(sort[1]=='b'){
    while(Cb!=colomn){
    Scan();
    UpdateError();
    if(errorLast==0 && ((irSensors==B111111 && irLast!=B111111)||(irSensors==B001111 && irLast!=B001111)||(irSensors==B000111 && irLast!=B000111)) && colomn<Cb){
     colomn++;
    }else if(errorLast==0 && ((irSensors==B111111 && irLast!=B111111)||(irSensors==B001111 && irLast!=B001111)||(irSensors==B000111 && irLast!=B000111)) && colomn>Cb){
     colomn--;
    }
    if(error==150|| error==-150){
      error=0;
    }
    UpdateCorrection();
    Drive();
    }
    if(bTrue==true){
      turningForDropB();
    }

    transData=3;
    pushBlue();
    WiFiTrans();
  }

}


void goTo3Row(){
  if(sort[2]=='r'){
    while(Rr!=row){
    Scan();
    UpdateError();
    if(errorLast==0 && ((irSensors==B111111 && irLast!=B111111)||(irSensors==B001111 && irLast!=B001111)||(irSensors==B000111 && irLast!=B000111)) && row>Rr){
      row--;
    }
    if(errorLast==0 && ((irSensors==B111111 && irLast!=B111111)||(irSensors==B001111 && irLast!=B001111)||(irSensors==B000111 && irLast!=B000111)) && row<Rr){
      row++;
    }
    if(error==150|| error==-150){
      error=0;
    }
    UpdateCorrection();
    Drive();
    }
  }else if(sort[2]=='g'){
    while(Rg!=row){
    Scan();
    UpdateError();
    if(errorLast==0 && ((irSensors==B111111 && irLast!=B111111)||(irSensors==B001111 && irLast!=B001111)||(irSensors==B000111 && irLast!=B000111)) && row>Rg){
      row--;
    }else if(errorLast==0 && ((irSensors==B111111 && irLast!=B111111)||(irSensors==B001111 && irLast!=B001111)||(irSensors==B000111 && irLast!=B000111)) && row<Rg){
      row++;
    }
    if(error==150|| error==-150){
      error=0;
    }
    UpdateCorrection();
    Drive();
    }
    
  }else if(sort[2]=='b'){
    while(Rb!=row){
    Scan();
    UpdateError();
    if(errorLast==0 && ((irSensors==B111111 && irLast!=B111111)||(irSensors==B001111 && irLast!=B001111)||(irSensors==B000111 && irLast!=B000111)) && row>Rb){
      row--;
    }else if(errorLast==0 && ((irSensors==B111111 && irLast!=B111111)||(irSensors==B001111 && irLast!=B001111)||(irSensors==B000111 && irLast!=B000111)) && row<Rb){
      row++;
    }
    if(error==150|| error==-150){
      error=0;
    }
    UpdateCorrection();
    Drive();
    }
    
  }

}




void goTo3Colomn(){
  if(sort[2]=='r'){
    while(Cr!=colomn){
    Scan();
    UpdateError();
    if(errorLast==0 && ((irSensors==B111111 && irLast!=B111111)||(irSensors==B001111 && irLast!=B001111)||(irSensors==B000111 && irLast!=B000111)) && colomn<Cr){
      colomn++;
    }else if(errorLast==0 && ((irSensors==B111111 && irLast!=B111111)||(irSensors==B001111 && irLast!=B001111)||(irSensors==B000111 && irLast!=B000111)) && colomn>Cr){
      colomn--;
    }
    if(error==150|| error==-150){
      error=0;
    }
    UpdateCorrection();
    Drive();
    }
    if(rTrue==true){
      turningForDropR();
    }

    transData=1;
    pushRed();
    WiFiTrans();
    
  }else if(sort[2]=='g'){
    while(Cg!=colomn){
    Scan();
    UpdateError();
    if(errorLast==0 && ((irSensors==B111111 && irLast!=B111111)||(irSensors==B001111 && irLast!=B001111)||(irSensors==B000111 && irLast!=B000111)) && colomn<Cg){
      colomn++;
    }else if(errorLast==0 && ((irSensors==B111111 && irLast!=B111111)||(irSensors==B001111 && irLast!=B001111)||(irSensors==B000111 && irLast!=B000111)) && colomn>Cg){
      colomn--;
    }
    if(error==150|| error==-150){
      error=0;
    }
    UpdateCorrection();
    Drive();
    }
    if(gTrue==true){
      turningForDropG();
    }

    transData=2;
    pushGreen();
    WiFiTrans();
    
  }else if(sort[2]=='b'){
    while(Cb!=colomn){
    Scan();
    UpdateError();
    if(errorLast==0 && ((irSensors==B111111 && irLast!=B111111)||(irSensors==B111111 && irLast!=B111111)||(irSensors==B111111 && irLast!=B111111)) && colomn<Cb){
     colomn++;
    }else if(errorLast==0 && ((irSensors==B111111 && irLast!=B111111)||(irSensors==B001111 && irLast!=B001111)||(irSensors==B000111 && irLast!=B000111)) && colomn>Cb){
     colomn--;
    }
    if(error==150|| error==-150){
      error=0;
    }
    UpdateCorrection();
    Drive();
    }

    if(bTrue==true){
      turningForDropB();
    }


    transData=3;
    pushBlue();
    WiFiTrans();
    
  }

  

}


void findEndRow(){
  Rb=9;
  Scan();
  UpdateError();
  if(Rb>row && colomnP<colomn){
   error=150;
  }else if(Rb>row && colomnP>colomn){
    error=-150;
  }else if(Rb<row && colomnP<colomn){
    error=-150;
  }else if(Rb<row && rowP>row && colomnP>colomn){
    error=150;
  }
  UpdateCorrection();
  Drive();
}

void findEndColomn(){
  Cb=0;
  Scan();
  UpdateError();
  if(Cb>colomn && rowP<row){
    error=-150;
  }else if(Cb>colomn && rowP>row){
    error=150;
  }else if(Cb<colomn && rowP<row){
    error=150;
  }else if(Cb<colomn && rowP>row){
    error=-150;
  }
  UpdateCorrection();
  Drive();
}

void goToEndColomn(){
  while(Cb!=colomn){
    Scan();
    UpdateError();
    if(errorLast==0 && ((irSensors==B111111 && irLast!=B111111)||(irSensors==B001111 && irLast!=B001111)||(irSensors==B000111 && irLast!=B000111))){
      colomn++;
    }
    if(error==150|| error==-150){
      error=0;
    }
    UpdateCorrection();
    Drive();
    }
}

void goToEndRow(){
  while(Rb!=row){
    Scan();
    UpdateError();
    if(errorLast==0 && ((irSensors==B111111 && irLast!=B111111)||(irSensors==B001111 && irLast!=B001111)||(irSensors==B000111 && irLast!=B000111))){
      row--;
    }
    UpdateCorrection();
    Drive();
    }
}



void UltraScan(){             //for finding distance

  digitalWrite(trigPinR,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinR,LOW);

  durationR=pulseIn(echoPinR,HIGH);
  distanceR=durationR*0.034/2;      //velocity 0.034cm/micro Second
/*      for debugging
  Serial.print("Duration");
  Serial.print(duration);
  Serial.print("      Distance: ");
  Serial.println(distance);
  */

  digitalWrite(trigPinL,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinL,LOW);

  durationL=pulseIn(echoPinL,HIGH);
  distanceL=durationL*0.034/2;      //velocity 0.034cm/micro Second
  
}


void calcObsPoints(){               //Calculating points by positions of pillers
  
  irLast = irSensors;

  if(irLast!=B111111 && irSensors==B111111){
    lineValue=lineValue+10;
  }
  
  //checking boxes and calculating points
  if(distanceR<=13 && distanceR>=5 && irLast==B111111 && irSensors!=B111111){        
    obsPoints=obsPoints+lineValue*2;
//    digitalWrite(ledLW,LOW);
    digitalWrite(ledRW,HIGH);
    delay(500);
//    digitalWrite(ledLW,LOW);
    digitalWrite(ledRW,LOW);
  }

  if(distanceL<=13 && distanceL>=5 && irLast==B111111 && irSensors!=B111111){       
    obsPoints=obsPoints+lineValue*1;
    digitalWrite(ledLW,HIGH);
//    digitalWrite(ledRW,LOW);
    delay(500);
    digitalWrite(ledLW,LOW);
//    digitalWrite(ledRW,LOW);
  }
}



void whichRed(){            //finding red dropping box number
  if(obsPoints>=100 && obsPoints<=166){
    rC=1;
  }else if(obsPoints>=167 && obsPoints<=233){
    rC=2;
  }else if(obsPoints>=234 && obsPoints<=300){
    rC=3;
  }
}



/*
 * 
 * fibinocci number can calulate using recursive function. Because of process time is higher in Arduino for calculating bigger fibinocci number, We have hard coded here.
 * 
 */

void whichBlue(){     //finding blue dropping box number
  
if(obsPoints==103||obsPoints==122||obsPoints==135||obsPoints==139||obsPoints==140||obsPoints==148||obsPoints==152||obsPoints==155||obsPoints==164||obsPoints==166||obsPoints==170||obsPoints==176||obsPoints==181||obsPoints==189||obsPoints==200||obsPoints==217||obsPoints==224||obsPoints==225||obsPoints==244||obsPoints==258||obsPoints==259||obsPoints==284||obsPoints==288||obsPoints==290||obsPoints==295)
    bV=0;
else if(obsPoints==116||obsPoints==119||obsPoints==126||obsPoints==128||obsPoints==134||obsPoints==137||obsPoints==141||obsPoints==151||obsPoints==162||obsPoints==188||obsPoints==205||obsPoints==208||obsPoints==222||obsPoints==226||obsPoints==233||obsPoints==237||obsPoints==243||obsPoints==248||obsPoints==254||obsPoints==256||obsPoints==260||obsPoints==263||obsPoints==267||obsPoints==278||obsPoints==289)
     bV=1;
else if(obsPoints==104||obsPoints==105||obsPoints==107||obsPoints==132||obsPoints==142||obsPoints==153||obsPoints==160||obsPoints==169||obsPoints==186||obsPoints==195||obsPoints==197||obsPoints==204||obsPoints==215||obsPoints==218||obsPoints==219||obsPoints==229||obsPoints==230||obsPoints==240||obsPoints==245||obsPoints==272||obsPoints==275||obsPoints==283||obsPoints==285||obsPoints==291||obsPoints==300)
     bV=2;
else if(obsPoints==101||obsPoints==115||obsPoints==143||obsPoints==154||obsPoints==174||obsPoints==199||obsPoints==201||obsPoints==206||obsPoints==209||obsPoints==212||obsPoints==227||obsPoints==241||obsPoints==271||obsPoints==286||obsPoints==292||obsPoints==294)
     bV=3;
else if(obsPoints==100||obsPoints==111||obsPoints==112||obsPoints==113||obsPoints==118||obsPoints==125||obsPoints==171||obsPoints==172||obsPoints==179||obsPoints==202||obsPoints==207||obsPoints==220||obsPoints==231||obsPoints==253||obsPoints==262||obsPoints==266||obsPoints==276||obsPoints==284||obsPoints==296||obsPoints==297)
     bV=4;
else if(obsPoints==106||obsPoints==110||obsPoints==120||obsPoints==127||obsPoints==144||obsPoints==146||obsPoints==147||obsPoints==165||obsPoints==180||obsPoints==182||obsPoints==183||obsPoints==185||obsPoints==210||obsPoints==238||obsPoints==246||obsPoints==255||obsPoints==264||obsPoints==273||obsPoints==293)
     bV=5;
else if(obsPoints==108||obsPoints==117||obsPoints==123||obsPoints==131||obsPoints==149||obsPoints==150||obsPoints==167||obsPoints==168||obsPoints==193||obsPoints==194||obsPoints==213||obsPoints==214||obsPoints==221||obsPoints==232||obsPoints==236||obsPoints==239||obsPoints==247||obsPoints==252||obsPoints==268||obsPoints==277||obsPoints==280||obsPoints==287)
     bV=6;
else if(obsPoints==102||obsPoints==121||obsPoints==124||obsPoints==129||obsPoints==159||obsPoints==175||obsPoints==177||obsPoints==184||obsPoints==192||obsPoints==198||obsPoints==234||obsPoints==251||obsPoints==261||obsPoints==265||obsPoints==279||obsPoints==281||obsPoints==299)
     bV=7;
else if(obsPoints==114||obsPoints==136||obsPoints==145||obsPoints==156||obsPoints==158||obsPoints==161||obsPoints==173||obsPoints==178||obsPoints==187||obsPoints==190||obsPoints==203||obsPoints==211||obsPoints==235||obsPoints==242||obsPoints==249||obsPoints==269||obsPoints==274||obsPoints==298)
     bV=8;
else if(obsPoints==109||obsPoints==130||obsPoints==133||obsPoints==138||obsPoints==157||obsPoints==163||obsPoints==191||obsPoints==196||obsPoints==216||obsPoints==223||obsPoints==228||obsPoints==250||obsPoints==257||obsPoints==270)
     bV=9;

if(bV>=0 && bV<=2){
  bC=1;
}else if(bV>=3 && bV<=6){
  bC=2;
}else if(bV>=7 && bV<=9){
  bC=3;
}
    
}

void whichGreen(){            //finding green dropping box number
  if (obsPoints== 100||obsPoints== 108 ||obsPoints== 109    ||obsPoints== 119    ||obsPoints== 147    ||obsPoints==  148   ||obsPoints==  157   ||obsPoints== 159    ||obsPoints==  161   ||obsPoints== 162    ||obsPoints== 197    ||obsPoints== 213||obsPoints== 221||obsPoints== 263||obsPoints==  269||obsPoints== 274 ||obsPoints== 286)
    gV=0;
else if (obsPoints== 101||obsPoints== 102||obsPoints== 106||obsPoints== 110||obsPoints== 111||obsPoints== 125||obsPoints== 131||obsPoints== 145||obsPoints== 149||obsPoints== 163||obsPoints== 164||obsPoints== 170||obsPoints== 171||obsPoints== 176||obsPoints== 179||obsPoints== 184||obsPoints== 196||obsPoints== 203||obsPoints== 208||obsPoints== 209||obsPoints== 219||obsPoints== 231||obsPoints== 234||obsPoints== 239||obsPoints== 246||obsPoints== 279 )
    gV=1;
else if (obsPoints== 103||obsPoints== 112||obsPoints== 128||obsPoints== 130||obsPoints== 139||obsPoints== 150||obsPoints== 165||obsPoints== 168||obsPoints== 190||obsPoints== 191||obsPoints== 194||obsPoints== 198||obsPoints== 206||obsPoints== 210||obsPoints== 217||obsPoints== 230||obsPoints== 245||obsPoints== 251||obsPoints== 257||obsPoints== 262||obsPoints== 264||obsPoints== 265||obsPoints== 268||obsPoints== 273||obsPoints== 275||obsPoints== 276||obsPoints== 291||obsPoints== 295||obsPoints== 299 )
    gV=2;
else if (obsPoints== 113||obsPoints== 117||obsPoints== 132||obsPoints== 138||obsPoints== 143||obsPoints== 151||obsPoints== 155||obsPoints== 172||obsPoints== 175||obsPoints== 188||obsPoints== 199||obsPoints== 202||obsPoints== 211||obsPoints== 228||obsPoints== 243||obsPoints== 247||obsPoints== 250||obsPoints== 255||obsPoints== 259||obsPoints== 270||obsPoints== 290||obsPoints== 294||obsPoints== 298 )
    gV=3;
else if (obsPoints== 104||obsPoints== 116||obsPoints== 124||obsPoints== 135||obsPoints== 136||obsPoints== 142||obsPoints== 154||obsPoints== 166||obsPoints==173||obsPoints==177||obsPoints==232||obsPoints==254||obsPoints==260||obsPoints==271||obsPoints==277|285||obsPoints==287||obsPoints==288)
    gV=4;
else if (obsPoints==114||obsPoints==126||obsPoints==133||obsPoints==152||obsPoints==183||obsPoints==187||obsPoints==192||obsPoints==200||obsPoints==204||obsPoints==216||obsPoints==227||obsPoints==238||obsPoints==242||obsPoints==248||obsPoints==266||obsPoints==292||obsPoints==296)
    gV=5;
else if (obsPoints==123||obsPoints==127||obsPoints==140||obsPoints==178||obsPoints==182||obsPoints==185||obsPoints==212||obsPoints==233||obsPoints==237||obsPoints==240||obsPoints==252||obsPoints==278||obsPoints==284||obsPoints==300)
    gV=6;
else if(obsPoints==105||obsPoints==118||obsPoints==122||obsPoints==156||obsPoints==167||obsPoints==180||obsPoints==193||obsPoints==205||obsPoints==214||obsPoints==215||obsPoints==226||obsPoints==235||obsPoints==244||obsPoints==261||obsPoints==267||obsPoints==272||obsPoints==283)
    gV=7;
else if(obsPoints==115||obsPoints==120||obsPoints==141||obsPoints==144||obsPoints==153||obsPoints==174||obsPoints==181||obsPoints==186||obsPoints==201||obsPoints==218||obsPoints==224||obsPoints==225||obsPoints==236||obsPoints==241||obsPoints==249||obsPoints==253||obsPoints==256||obsPoints==258||obsPoints==280||obsPoints==282||obsPoints==289||obsPoints==293||obsPoints==297)
    gV=8;
else if(obsPoints==107||obsPoints==121||obsPoints==129||obsPoints==134||obsPoints==137||obsPoints==146||obsPoints==158||obsPoints==160||obsPoints==169||obsPoints==189||obsPoints==195||obsPoints==207||obsPoints==220||obsPoints==222||obsPoints==223||obsPoints==229||obsPoints==281)
    gV=9;



if(gV>=0 && gV<=2){
  bC=1;
}else if(gV>=3 && gV<=6){
  bC=2;
}else if(gV>=7 && gV<=9){
  bC=3;
}


}



void turningForDropR(){
  while(irSensors != B001100){
    Scan();
    motorRSpeed = -(maxSpeed);
    motorLSpeed = maxSpeed;
    Drive();
      
    }
    motorRSpeed =0;
    motorLSpeed = 0;
    Drive();

    if(rTrue==true){
      while(irSensors != B001100){
    Scan();
    motorRSpeed = maxSpeed;
    motorLSpeed = -(maxSpeed);
    Drive();
      
    }
    motorRSpeed =0;
    motorLSpeed = 0;
    Drive();
    }
}

void turningForDropG(){
  while(irSensors != B001100){
    Scan();
    motorRSpeed = -(maxSpeed);
    motorLSpeed = maxSpeed;
    Drive();
      
    }
    motorRSpeed =0;
    motorLSpeed = 0;
    Drive();

    if(rTrue==true){
      while(irSensors != B001100){
    Scan();
    motorRSpeed = maxSpeed;
    motorLSpeed = -(maxSpeed);
    Drive();
      
    }
    motorRSpeed =0;
    motorLSpeed = 0;
    Drive();
    }
}

void turningForDropB(){
  while(irSensors != B001100){
    Scan();
    motorRSpeed = -(maxSpeed);
    motorLSpeed = maxSpeed;
    Drive();
      
    }
    motorRSpeed =0;
    motorLSpeed = 0;
    Drive();

    if(rTrue==true){
      while(irSensors != B001100){
    Scan();
    motorRSpeed = maxSpeed;
    motorLSpeed = -(maxSpeed);
    Drive();
      
    }
    motorRSpeed =0;
    motorLSpeed = 0;
    Drive();
    }
}



void WiFiTrans(){
  WiFiClient client = server.available();

  if (client) {
char f;
switch(transData)
{
  case 1:
  f='A'; break;
 
  case 2:
  f='B'; break;
  case 3:
  f='C'; break;
case 4:
  f='D'; break;
case 5:
  f='E'; break;
case 6:
  f='F'; break;
case 7:
  f='G'; break;
case 8:
  f='H'; break;
case 9:
  f='I'; break;
case 10:
  f='J'; break;
case 11:
  f='K'; break;
case 12:
  f='L'; break;
case 13:
  f='M'; break;
case 14:
  f='N'; break;
case 15:
  f='O'; break;
case 16:
  f='P'; break;
case 17:
  f='Q'; break;
case 18:
  f='R'; break;
case 19:
  f='S'; break;
 case 20:
  f='T'; break;
  case 21:
  f='U'; break;
  case 22:
  f='V'; break;
  case 23:
  f='W'; break;
  case 24:
  f='X'; break;
  case 25:
  f='Y'; break;
  case 26:
  f='Z'; break;
  case 27:
  f='a'; break;
  case 28:
  f='b'; break;
  case 29:
  f='c'; break;
  case 30:
  f='d'; break;
  case 31:
  f='e'; break;
  case 32:
  f='f'; break;
  case 33:
  f='g'; break;
  case 34:
  f='h'; break;
  case 35:
  f='i'; break;
  case 36:
  f='j'; break;
  case 37:
  f='k'; break;
  case 38:
  f='l'; break;
  case 39:
  f='m'; break;
  case 40:
  f='z'; break;


}
          client.print(f);

  }
}






void removeSecond(){      
  lock.write(120);
}

void pushRed(){
  redP.write(0);
}

void pushGreen(){
  greenP.write(180);
}

void pushBlue(){
  greenP.write(0);
}
