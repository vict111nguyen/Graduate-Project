#include<math.h>
#include <TimerOne.h>
#include "Ublox.h"

unsigned long previousMillis = 0;  // will store last time LED was updated
unsigned long previousMillis1 = 0;
unsigned long previousMillis2 = 0;
// constants won't change:
const long interval = 1000;  // interval at which to blink (milliseconds)
const long interval1 = 1000;
const long interval2 = 3000;  // interval at which to blink (milliseconds)

#define N_FLOATS 3

Ublox M8_Gps;
float gpsArray[N_FLOATS] = {0, 0, 0};

#include <Wire.h>

//Address map for registers
#define configA 0x00
#define configB 0x01
#define mode 0x02
#define dataOutX_U 0x03
#define dataOutX_L 0x04
#define dataOutZ_L 0x05
#define dataOutZ_L 0x06
#define dataOutY_L 0x07
#define dataOutY_L 0x08
#define statusReg 0x09
#define denta_phi -0.03228859116
//Operating modes sent to Mode register (0x02)
#define continuous 0x00
#define single 0x01
#define idle 0x02

#define i2c_addr 0x1E
#define gain 1090

int16_t x = 0;
int16_t y = 0;
int16_t z = 0;
float heading;
float gaussX;
float gaussY;
float gaussZ;
float offsetX=60;
float offsetY=45.5;

float t;
float u;
uint8_t c;

//giao dien
uint32_t lat_rec;
uint32_t long_rec;
uint16_t angle;
uint8_t buff[5];
uint8_t buff1[3];
uint8_t bytes[9];
uint8_t a=255;
bool b_newDT = false;
uint8_t i = 0;
uint16_t distance1;
uint16_t targetHeading1;

// dieu khien xe
#define WAYPOINT_DIST_TOLERANE 3
#define EARTH_RADIUS 6372795 
float currentLat=t;
float currentLong=u;

double targetLat;
double targetLong;

int headingError;
int err_output;
int targetHeading;
int currentHeading;
int distance=0;

// PID variable
float count=0,count1 = 0,count2 = 0,count3 = 0,count4 = 0;
float T;
float denta_speed,speed_target,speed;
float E,E1,E2;
float alpha, beta, gamma;
float Output, LastOutput;

// define chan
#define en1 31   
#define en2 33
#define en3 35
#define en4 37
#define Dir1 7
#define Dir2 6
#define Dir3 5
#define Dir4 4
#define pwm 11

//define encoder
#define encoderPinA1 2
#define encoderPinA2 3
#define encoderPinA3 18
#define encoderPinA4 19
#define encoderPinB1 22
#define encoderPinB2 24
#define encoderPinB3 26
#define encoderPinB4 28

//define PID parameter
#define Kp 125
#define Kd 0.001
#define Ki 0.00001



void setup()
{ Wire.begin();
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial2.begin(9600);
  pinMode(en1,OUTPUT);
  pinMode(en2,OUTPUT);
  pinMode(en3,OUTPUT);
  pinMode(en4,OUTPUT);
  pinMode(Dir1,OUTPUT);
  pinMode(Dir2,OUTPUT);
  pinMode(Dir3,OUTPUT);
  pinMode(Dir4,OUTPUT);
  pinMode(pwm,OUTPUT);
  speed_target=40; speed=0;
  E=0; E1=0; E2=0;
  Output = 0; LastOutput = 0;
  T=0.01;
  pinMode(encoderPinA1, INPUT_PULLUP);
  pinMode(encoderPinB1, INPUT_PULLUP);
  pinMode(encoderPinA2, INPUT_PULLUP);
  pinMode(encoderPinB2, INPUT_PULLUP);
  pinMode(encoderPinA3, INPUT_PULLUP);
  pinMode(encoderPinB3, INPUT_PULLUP);
  pinMode(encoderPinA4, INPUT_PULLUP);
  pinMode(encoderPinB4, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinA1), countPulse1, HIGH);
  attachInterrupt(digitalPinToInterrupt(encoderPinA2), countPulse2, HIGH);
  attachInterrupt(digitalPinToInterrupt(encoderPinA3), countPulse3, HIGH);
  attachInterrupt(digitalPinToInterrupt(encoderPinA4), countPulse4, HIGH);
  setOperatingMode(continuous);
  setSamples();
  Timer1.initialize(10000);  //don vi us
  Timer1.attachInterrupt(PID);
}

uint8_t devide_data(float latt,float logg,int heading){
  unsigned long h=latt*1000000;
  unsigned long k=logg*1000000;
  bytes[0] = a & 0xFF;
  bytes[1] = h >> 24 & 0xFF;
  bytes[2] = h >> 16  & 0xFF;
  bytes[3] = h >> 8 & 0xFF;
  bytes[4] = h >> 0 & 0xFF;
  bytes[5] = k >> 24 & 0xFF;
  bytes[6] = k >> 16  & 0xFF;
  bytes[7] = k >> 8 & 0xFF;
  bytes[8] = k >> 0 & 0xFF;
  // bytes[9] = k >> 8 & 0xFF;
  // bytes[10] = k >> 0 & 0xFF;
  return bytes[9];
}
void sent_data(uint8_t arr[]){
unsigned long currentMillis = millis();
if (currentMillis - previousMillis >= interval) {
//     // save the last time you blinked the LED
  previousMillis = currentMillis;
  for (int i = 0; i < sizeof(bytes); i = i + 1) {
  Serial1.write(bytes[i]);
  }
}
}


int calculateDistance(double current_Lat, double current_Long, double target_Lat, double target_Long) {
  double dLat = (target_Lat - current_Lat) * PI / 180.0; // convert to radians
  double dLon = (target_Long - current_Long) * PI / 180.0; // convert to radians
  double a = sin(dLat / 2) * sin(dLat / 2) + cos(current_Lat * PI / 180.0) * cos(target_Lat * PI / 180.0) * sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  int distance = (int)(EARTH_RADIUS * c); // convert to integer and round to the nearest meter
  return distance;
}
int calculateAngle(double current_Lat1, double current_Long1, double target_Lat1, double target_Long1) {
  double dLat1 = (target_Lat1 - current_Lat1) * PI / 180.0; // convert to radians
  double dLon1 = (target_Long1 - current_Long1) * PI / 180.0; // convert to radians
  double a = sin(dLat1 / 2) * sin(dLat1 / 2) + cos(current_Lat1 * PI / 180.0) * cos(target_Lat1 * PI / 180.0) * sin(dLon1 / 2) * sin(dLon1 / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  int angle=degrees(c);
  return angle;
}

int courseToWaypoint(double current_Lat, double current_Long, double target_Lat, double target_Long)
{
float dlon = radians(target_Long - current_Long);
float cLat = radians(current_Lat);
float tLat = radians(target_Lat);
float a1 = sin(dlon) * cos(tLat);
float a2 = sin(cLat) * cos(tLat) * cos(dlon);
a2 = cos(cLat) * sin(tLat) - a2;
a2 = atan2(a1, a2);
if (a2 < 0.0)
{
a2 += TWO_PI;
}
targetHeading = degrees(a2);
return targetHeading;
}

void calErtomove(int targetHeading1, int currentHeading1 )
{
  headingError = targetHeading1 - currentHeading1;  // calculate where we need to turn to head to destination
  if(headingError >= -20 && headingError <= 20 || headingError > 350 || headingError <-350)
  {
    err_output = 2;
  }
  else{
    if(headingError > 10 && headingError < 180)
    {
      err_output = 10;
    }
    if(headingError > 180)
    {
      err_output = 11;
    }
    if(headingError > -180 && headingError < 10)
    {
      err_output = 11;
    }
    if(headingError < -180)
    {
      err_output = 10;
    }

  }
}


void Car_control(int st,int speed){
  int pwmOutput = speed;
  analogWrite(pwm, pwmOutput);
  switch (st){
    case(1): //forward
      digitalWrite(en1,HIGH);
      digitalWrite(Dir1,HIGH);
      digitalWrite(en2,HIGH);
      digitalWrite(Dir2,HIGH);
      digitalWrite(en4,HIGH);
      digitalWrite(Dir4,HIGH);
      digitalWrite(en3,HIGH);
      digitalWrite(Dir3,HIGH);
      break;
    case(2): // back
      digitalWrite(en1,HIGH);
      digitalWrite(Dir1,LOW);
      digitalWrite(en2,HIGH);
      digitalWrite(Dir2,LOW);
      digitalWrite(en4,HIGH);
      digitalWrite(Dir4,LOW);
      digitalWrite(en3,HIGH);
      digitalWrite(Dir3,LOW);
      break;
      case(3)://for-right
      digitalWrite(en1,LOW);
      digitalWrite(Dir1,HIGH);
      digitalWrite(en2,HIGH);
      digitalWrite(Dir2,LOW);
      digitalWrite(en4,LOW);
      digitalWrite(Dir4,LOW);
      digitalWrite(en3,HIGH);
      digitalWrite(Dir3,LOW);
      break;
      case(4): // for-left
      digitalWrite(en1,HIGH);
      digitalWrite(Dir1,LOW);
      digitalWrite(en2,LOW);
      digitalWrite(Dir2,LOW);
      digitalWrite(en4,HIGH);
      digitalWrite(Dir4,LOW);
      digitalWrite(en3,LOW);
      digitalWrite(Dir3,LOW);
      break;
      case(5):  //back-right
      digitalWrite(en1,HIGH);
      digitalWrite(Dir1,HIGH);
      digitalWrite(en2,LOW);
      digitalWrite(Dir2,HIGH);
      digitalWrite(en4,HIGH);
      digitalWrite(Dir4,HIGH);
      digitalWrite(en3,LOW);
      digitalWrite(Dir3,HIGH);
      break;
      case(6): // back-left
      digitalWrite(en1,LOW);
      digitalWrite(Dir1,HIGH);
      digitalWrite(en2,HIGH);
      digitalWrite(Dir2,HIGH);
      digitalWrite(en4,LOW);
      digitalWrite(Dir4,HIGH);
      digitalWrite(en3,HIGH);
      digitalWrite(Dir3,HIGH);
      break;
      case(7): // right
      
      digitalWrite(en1,HIGH);
      digitalWrite(en2,HIGH);
      digitalWrite(Dir1,HIGH);

      
      digitalWrite(en4,HIGH);
      digitalWrite(en3,HIGH);
      digitalWrite(Dir3,HIGH);
      break;
      case(8): // left
      
      digitalWrite(en1,HIGH);
      digitalWrite(en2,HIGH);
      digitalWrite(Dir2,HIGH);

      digitalWrite(en3,HIGH);
      digitalWrite(en4,HIGH);
      digitalWrite(Dir4,HIGH);
      break;
      case(9): //stop
      digitalWrite(en1,LOW);
      digitalWrite(Dir1,LOW);
      digitalWrite(en2,LOW);
      digitalWrite(Dir2,LOW);
      digitalWrite(en4,LOW);
      digitalWrite(Dir4,LOW);
      digitalWrite(en3,LOW);
      digitalWrite(Dir3,LOW);
      break;
      case(10): //round_right
      digitalWrite(en1,HIGH);
      digitalWrite(Dir1,HIGH);
      digitalWrite(en2,HIGH);
      digitalWrite(Dir2,LOW);
      digitalWrite(en3,HIGH);
      digitalWrite(Dir3,LOW);
      digitalWrite(en4,HIGH);
      digitalWrite(Dir4,HIGH);
      break;
      case(11): //round_left
      digitalWrite(en1,HIGH);
      digitalWrite(Dir1,LOW);
      digitalWrite(en2,HIGH);
      digitalWrite(Dir2,HIGH);
      digitalWrite(en3,HIGH);
      digitalWrite(Dir3,HIGH);
      digitalWrite(en4,HIGH);
      digitalWrite(Dir4,LOW);
      break;
    default:
      break;
  }
}

void loop()
{ 
  uint8_t *p;
  uint8_t status = 0;
  while (Serial3.available()>0)
  { 
    char first = Serial3.read();
    if (M8_Gps.encode(first))
    { 
      gpsArray[0] = M8_Gps.latitude;
      gpsArray[1] = M8_Gps.longitude;
      t=gpsArray[0];
      u=gpsArray[1];
    }
  }
  uint8_t temp;
  while (Serial1.available()>0)
  { temp =  Serial1.read();
    if(temp == 255) { b_newDT = true; i = 0;}
    if(b_newDT) { buff[i] = temp; i ++;}
    if(i>=5) { b_newDT = false; }

    // if(temp == 254) { b_newDT = true; i = 0;}
    // if(b_newDT) { buff1[i] = temp; i ++;}
    // if(i>=3) { b_newDT = false; }
  }
  getXYZ();
  convert(x,y,z);
  sent_data(bytes);
  currentHeading=getHeading(gaussX,gaussY,gaussZ);
  Serial.print("Goc hien tai:");
  Serial.println(currentHeading);
  // Serial.println(currentHeading);
  bytes[9]=devide_data(t,u,currentHeading);
  // lat_rec=buff[1]<<24|buff[2]<<16|buff[3]<<8|buff[4];
  // long_rec=buff[5]<<24|buff[6]<<16|buff[7]<<8|buff[8];
  uint16_t distance1=buff[1]<<8|buff[2];
  uint16_t targetHeading1=buff[3]<<8|buff[4];
  int distance =int(distance1);
  int targetHeading=(int)(targetHeading1);
  Serial.println(distance);
  Serial.print("Goc den:");
  Serial.println(targetHeading);
  // long_rec=buff[5]<<24|buff[6]<<16|buff[7]<<8|buff[8];
  // targetLat=(double)(lat_rec/1000000.0);
  // targetLong=(double)(long_rec/1000000.0);
  // Serial.println(targetLat);
  // Serial.println(targetLong);
  // distance=calculateDistance(t,u,targetLat , targetLong);
  // Serial.println(distance);
  // targetHeading= calculateAngle(t,u,targetLat , targetLong);  
  // targetHeading=courseToWaypoint(t,u,targetLat , targetLong);
  // Serial.println(targetHeading);
  if(distance <=3){
    err_output=9;
  }
  else{
    calErtomove(targetHeading,currentHeading);
  }
  Car_control(err_output,abs(Output));
}

//Convert the raw X, Y, Z counts to Gauss
void convert(int16_t rawX, int16_t rawY, int16_t rawZ){
  gaussX = (float)(rawX+offsetX)/gain;
  gaussY = (float)(rawY+offsetY)/gain;
  gaussZ = (float)rawZ/gain;
}

//accounts for declination (error in magnetic field which is dependent on location)
uint16_t getHeading(float X, float Y, float Z){
  heading = (atan2(Y,X)+ denta_phi ) * 180 / PI +90 ; // sai lech do dat cam bien khac huong
  if (heading < 0) heading += 360;
  if (heading > 360) heading -= 360;
  return heading;
}

void setSamples(void){
  Wire.beginTransmission(i2c_addr);
  Wire.write(configA);        //write to config A register
  Wire.write(0x70);           //8 samples averaged, 15Hz output rate, normal measurement
  Wire.endTransmission();
  delay(10);
}


void setOperatingMode(uint8_t addr){
  Wire.beginTransmission(i2c_addr);
  Wire.write(mode);           //write to mode register
  Wire.write(addr);           //set measurement mode
  Wire.endTransmission();
  delay(10);
}

//get the raw counts of X, Y, Z from registers 0x03 to 0x08
void getXYZ(void){
  Wire.beginTransmission(i2c_addr);     // lay du lieu
  Wire.write(0x03);                     // dua du lieu vao thanh ghi 03
  Wire.endTransmission();               // dung viec lay

  Wire.requestFrom(i2c_addr, 6);        // lay du 6 byte
  if (Wire.available() >= 6){
    int16_t temp = Wire.read();     //read upper byte of X
    x = temp << 8;
    temp = Wire.read();             //read lower byte of X
    x = x | temp;
    temp = Wire.read();             //read upper byte of Z
    z = temp << 8;
    temp = Wire.read();             //read lower byte of Z
    z = z | temp;
    temp = Wire.read();             //read upper byte of Y
    y = temp << 8;
    temp = Wire.read();             //read lower byte of Y
    y = y | temp;
  }
}

//PID function
void PID(){
  count=(count1+count2+count3+count4)/4;
  speed=((count/304)*(1/T)*60);
  count1=0;
  count2=0;
  count3=0;
  count4=0;
  Serial.print(speed);
  Serial.print("\t");
  int a=0;
  Serial.println(a);
  // Serial.println(count);
  count=0;
  E=speed_target-speed;
  alpha = 2*T*Kp + Ki*T*T+ 2*Kd;
  beta = T*T*Ki - 4*Kd - 2*T*Kp;
  gamma = 2*Kd;
  Output = (alpha*E + beta*E1 + gamma*E2 + 2*T*LastOutput)/(2*T);
  LastOutput = Output;
  E2 = E1;
  E1 = E;
  Car_control(1,abs(Output));
}

void countPulse1() {
  if(digitalRead(encoderPinB1)==LOW){
    count1 ++;
  }
}
void countPulse2() {
  if(digitalRead(encoderPinB2)==LOW){
    count2 ++;
  }
}
void countPulse3() {
  if(digitalRead(encoderPinB3)==LOW){
    count3 ++;
  }
}
void countPulse4() {
  if(digitalRead(encoderPinB4)==LOW){
    count4 ++;
  }
}
