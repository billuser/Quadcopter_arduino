#include <EasyScheduler.h>


#include <Servo.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <helper_3dmath.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <PID_v1.h>
#include <PinChangeInt.h>
#include <PinChangeIntConfig.h>


#define ESC_A 9
#define ESC_B 6
#define ESC_C 5
#define ESC_D 3

#define RC_1 13
#define RC_2 12 
#define RC_3 11
#define RC_4 10
#define RC_5 8
#define RC_PWR A0

#define ESC_MIN 22
#define ESC_MAX 115
#define ESC_TAKEOFF_OFFSET 20
#define ESC_ARM_DELAY 5000

#define RC_HIGH_CH1 1000
#define RC_LOW_CH1 2000
#define RC_HIGH_CH2 1000
#define RC_LOW_CH2 2000
#define RC_HIGH_CH3 1000
#define RC_LOW_CH3 2000
#define RC_HIGH_CH4 1000
#define RC_LOW_CH4 2000
#define RC_HIGH_CH5 1000
#define RC_LOW_CH5 2000
#define RC_ROUNDING_BASE 50

//#define PITCH_P_VAL 0.4
//#define PITCH_I_VAL 0
//#define PITCH_D_VAL 0
//#define ROLL_P_VAL 0.4
//#define ROLL_I_VAL 0
//#define ROLL_D_VAL 0
//#define YAW_P_VAL 0
//#define YAW_I_VAL 0
//#define YAW_D_VAL 0
//
//#define PITCH_MIN -20
//#define PITCH_MAX 20
//#define ROLL_MIN -20
//#define ROLL_MAX 20
//#define YAW_MIN -180
//#define YAW_MAX 180
//#define PID_PITCH_INFLUENCE 20  
//#define PID_ROLL_INFLUENCE 20
//#define PID_YAW_INFLUENCE 20

#define PITCH_P_VAL 3
#define PITCH_I_VAL 0
#define PITCH_D_VAL 1
#define ROLL_P_VAL 3
#define ROLL_I_VAL 0
#define ROLL_D_VAL 0.5
#define YAW_P_VAL 0
#define YAW_I_VAL 0
#define YAW_D_VAL 0

#define PITCH_MIN -100
#define PITCH_MAX 100
#define ROLL_MIN -100
#define ROLL_MAX 100
#define YAW_MIN -180
#define YAW_MAX 180
#define PID_PITCH_INFLUENCE 100 
#define PID_ROLL_INFLUENCE 100
#define PID_YAW_INFLUENCE 100



MPU6050 mpu;
uint8_t mpuIntStatus;                 
uint8_t devStatus;                    
uint16_t packetSize;                    
uint16_t fifoCount;                    
uint8_t fifoBuffer[64];     

Quaternion q;                          // quaternion for mpu output
VectorFloat gravity;                   // gravity vector for ypr
float ypr[3] = {0.0f,0.0f,0.0f};       // yaw pitch roll values
float yprLast[3] = {0.0f, 0.0f, 0.0f};
volatile bool mpuInterrupt = false;   
boolean interruptLock = false;
float ch1, ch2, ch3, ch4, ch5;  
unsigned long rcLastChange1 = micros();
unsigned long rcLastChange2 = micros();
unsigned long rcLastChange3 = micros();
unsigned long rcLastChange4 = micros();
unsigned long rcLastChange5 = micros();
int velocity;                          
float bal_ac , bal_bd;                 
float bal_axes;                       
int va, vb, vc, vd;                   
int v_ac, v_bd;                      
Servo a,b,c,d;
PID pitchReg(&ypr[1], &bal_bd, &ch2, PITCH_P_VAL, PITCH_I_VAL, PITCH_D_VAL, REVERSE);
PID rollReg(&ypr[2], &bal_ac, &ch1, ROLL_P_VAL, ROLL_I_VAL, ROLL_D_VAL, REVERSE);
PID yawReg(&ypr[0], &bal_axes, &ch4, YAW_P_VAL, YAW_I_VAL, YAW_D_VAL, DIRECT);
float ch1Last, ch2Last, ch4Last, velocityLast;
int currentValue = 150;
const int maxValue = 255;
#define DEBUG true
                             
byte index = 0;
const int q1 = 3;
const int q2 = 10;
const int q3 = 11;
const int q4 = 9;
const int mpuInt = 2;
Schedular Task1;
Schedular Task2;
long previousMillis = 0;
int on = 0;
int ac_tag = 1;
int init_bd = 0;

void setup()
{
  Serial.begin(115200);
  pinMode(13,OUTPUT);
  digitalWrite(13,LOW);
  pidSteup();
  pinMode(q1, OUTPUT);
  pinMode(q2, OUTPUT);
  pinMode(q3, OUTPUT);
  pinMode(q4, OUTPUT);
  pinMode(mpuInt, OUTPUT);
  digitalWrite(mpuInt,HIGH);
  sendData("AT+RST\r\n",2000,DEBUG); // reset module
  sendData("AT+CWMODE=2\r\n",1000,DEBUG); // configure as access point
  sendData("AT+CIFSR\r\n",1000,true); // get ip address
  sendData("AT+CIPMUX=1\r\n",1000,DEBUG); // configure for multiple connections
  sendData("AT+CIPSERVER=1,80\r\n",1000,DEBUG); // turn on server on port 80
  
  digitalWrite(13,HIGH);
  //Scheduler.startLoop(loop);
  
  Task1.start();
  Task2.start();
  
   
  
}
 
void loop()
{
    digitalWrite(13,LOW);
//    pidLoop();
//    receverDataLoop();
    Task1.check(pidLoop,0); 
    Task2.check(receverDataLoop,0);
    
}

void getDateValue(){
     String data = "";
      while (Serial.available() > 0) {
        char a = Serial.read();
        data += a;
      }
     String data_split = getValue(data, ' ', 1);
     //String getData = getValue(data_split, ' ', 0);
     Serial.println(data_split);
}

void receverDataLoop(){
  if(Serial.available()){
    if(Serial.find("+IPD,")){
     delay(100);
    // int connectionId = esp8266.read()-48; // subtract 48 because the read() function returns 
     String data = "";
      while (Serial.available() > 0) {
        char a = Serial.read();
        data += a;
      }
     String data_split_a = getValue(data, '?', 1);
     String data_split_b = getValue(data_split_a, ' ', 0);
     String getData = getValue(data_split_b, '=', 1);
     String key = getValue(data_split_b, '=', 0);
//     Serial.println(data_split);
//     Serial.println(key);
     if(key == "left"){
       Serial.println(getData);
       currentValue = getData.toInt();
     }else if(key == "right"){
       Serial.println(getData);
     }else if(key == "up"){
       on = 1;
       currentValue = getData.toInt();
       if(currentValue < 150){
         on = 0;
       }else{
         on = 1;
         if(init_bd == 0){
            init_bd = bal_bd;
          }
       }
       analogWrite(q1, getData.toInt());
       analogWrite(q2, getData.toInt());
       analogWrite(q3, getData.toInt());
       analogWrite(q4, getData.toInt());
     }
     String closeCommand = "AT+CIPCLOSE"; 
     Serial.print(closeCommand);
   
    }
  }

}

String sendData(String command, const int timeout, boolean debug)
{
    String response = "";
    
    Serial.print(command); // send the read character to the esp8266
    
    long int time = millis();
    
    while( (time+timeout) > millis())
    {
      while(Serial.available())
      {
        
        // The esp has data so display its output to the serial window 
        char c = Serial.read(); // read the next character.
        response+=c;
      }  
    }
    
    if(debug)
    {
      Serial.print(response);
    }
    
    return response;
}

String getValue(String data, char separator, int index)
{
 int found = 0;
  int strIndex[] = {
0, -1  };
  int maxIndex = data.length()-1;
  for(int i=0; i<=maxIndex && found<=index; i++){
  if(data.charAt(i)==separator || i==maxIndex){
  found++;
  strIndex[0] = strIndex[1]+1;
  strIndex[1] = (i == maxIndex) ? i+1 : i;
  }
 }
  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void pidSteup(){
  //initRC();                            // Self explaining
  initMPU();
  //initESCs();
//  initBalancing();
  initRegulators();
}

void pidLoop(){
  getYPR();                          
  computePID();
  calculateVelocities();
  //updateMotors();
}

void computePID(){
  acquireLock();
  ch1 = floor(ch1/RC_ROUNDING_BASE)*RC_ROUNDING_BASE;
  ch2 = floor(ch2/RC_ROUNDING_BASE)*RC_ROUNDING_BASE;
  ch4 = floor(ch4/RC_ROUNDING_BASE)*RC_ROUNDING_BASE;
  ch2 = map(ch2, RC_LOW_CH2, RC_HIGH_CH2, PITCH_MIN, PITCH_MAX);
  ch1 = map(ch1, RC_LOW_CH1, RC_HIGH_CH1, ROLL_MIN, ROLL_MAX);
  ch4 = map(ch4, RC_LOW_CH4, RC_HIGH_CH4, YAW_MIN, YAW_MAX);
  if((ch2 < PITCH_MIN) || (ch2 > PITCH_MAX)) ch2 = ch2Last;
  if((ch1 < ROLL_MIN) || (ch1 > ROLL_MAX)) ch1 = ch1Last;
  if((ch4 < YAW_MIN) || (ch4 > YAW_MAX)) ch4 = ch4Last;
  ch1Last = ch1;
  ch2Last = ch2;
  ch4Last = ch4;
  ypr[0] = ypr[0] * 180/M_PI;
  ypr[1] = ypr[1] * 180/M_PI;
  ypr[2] = ypr[2] * 180/M_PI;
  if(abs(ypr[0]-yprLast[0])>30) ypr[0] = yprLast[0];
  if(abs(ypr[1]-yprLast[1])>30) ypr[1] = yprLast[1];
  if(abs(ypr[2]-yprLast[2])>30) ypr[2] = yprLast[2];
  yprLast[0] = ypr[0];
  yprLast[1] = ypr[1];
  yprLast[2] = ypr[2];
  pitchReg.Compute();
  rollReg.Compute();
  yawReg.Compute();
  releaseLock();
}

void getYPR(){
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    if((mpuIntStatus & 0x10) || fifoCount >= 1024){ 
      mpu.resetFIFO(); 
    }else if(mpuIntStatus & 0x02){
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    }
}

void calculateVelocities(){
//  (202 - currentValue)/20*20+

//  if(on == 1){
    float bd_a = currentValue;
    float bal_bd_a = bal_bd;
    if(bal_bd_a >= -139 && bal_bd_a <= 0){
        bd_a = (maxValue - currentValue) / 100 * abs(bal_bd) + currentValue;
        if(on == 1){
          analogWrite(q2, bd_a);
        }
        
    }
    float bd_b = currentValue;
    if(bal_bd_a >= 0 && bal_bd_a <=139){
      bd_b = (maxValue - currentValue) / 100 * abs(bal_bd) + currentValue;
      if(on == 1){
        analogWrite(q1, bd_b);
      }
    }
    float ac_a = currentValue;
    if(abs(bal_ac) > 1 && ac_tag == 1){
        ac_tag =  1;
    }else if(abs(bal_ac) < 1 && abs(bal_ac) > 0){
      digitalWrite(13,HIGH);
      ac_tag =  0;
    }
    float bal_ac_a;
    if(ac_tag == 1){
       bal_ac_a = 0;
    }else{
       bal_ac_a = bal_ac + 1;
    }
//    float bal_ac_a = bal_ac;
    if(bal_ac_a >= -101 && bal_ac_a <= 0){
      ac_a = (maxValue - currentValue) / 100 * abs(bal_ac_a) + currentValue;
       if(on == 1){
         analogWrite(q3, ac_a);
       }
      
    }
    float ac_b = currentValue;
    if(bal_ac_a >= 0 && bal_ac_a <=101){
      ac_b = (maxValue - currentValue) / 100 * abs(bal_ac_a) + currentValue;
      if(on == 1){
        analogWrite(q4, ac_b);
      }
    }
    
//    Serial.print("  currentValue   ");
//   Serial.print(currentValue);
//   Serial.print("  bd_a   ");
//   Serial.print((int)bd_a);
//   Serial.print("  bd_b   ");
//   Serial.print((int)bd_b);
//   Serial.print("  ac_a   ");
//   Serial.print((int)ac_a);
//   Serial.print("  ac_b   ");
//   Serial.print((int)ac_b);

//   Serial.print("  bal_bd_a   ");
//   Serial.print(bal_bd_a);
   Serial.print("  bal_bd   ");
   Serial.print(bal_bd);
   Serial.print("  bal_ac   ");
   Serial.print(bal_ac);
//   Serial.print("  bal_ac_a   ");
//   Serial.print(bal_ac_a);
//   Serial.print("  ac_tag   ");
//   Serial.print(ac_tag);

//   bal_ac_a
   Serial.println(" ");
    
//  }
  
   

   
//  Serial.println(" ");
}

inline void updateMotors(){

  a.write(va);
  c.write(vc);
  b.write(vb);
  d.write(vd);
  

}

inline void arm(){

  a.write(ESC_MIN);
  b.write(ESC_MIN);
  c.write(ESC_MIN);
  d.write(ESC_MIN);
  //delay(ESC_ARM_DELAY);
  //delay(10000);

}

inline void dmpDataReady() {
    mpuInterrupt = true;
}

inline void initRC(){
  pinMode(RC_PWR, OUTPUT);
  digitalWrite(RC_PWR, HIGH);
  
  // FIVE FUCKING INTERRUPTS !!!
  PCintPort::attachInterrupt(RC_1, rcInterrupt1, CHANGE);
  PCintPort::attachInterrupt(RC_2, rcInterrupt2, CHANGE);
  PCintPort::attachInterrupt(RC_3, rcInterrupt3, CHANGE);
  PCintPort::attachInterrupt(RC_4, rcInterrupt4, CHANGE);
  PCintPort::attachInterrupt(RC_5, rcInterrupt5, CHANGE);
  
}

void initMPU(){
  
  Wire.begin();
  mpu.initialize();
  mpu.setMasterClockSpeed(13);
  devStatus = mpu.dmpInitialize();
  if(devStatus == 0){
  
    mpu.setDMPEnabled(true);
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    packetSize = mpu.dmpGetFIFOPacketSize();
    
  }
}

inline void initESCs(){
  a.attach(ESC_A);
  b.attach(ESC_B);
  c.attach(ESC_C);
  d.attach(ESC_D);
  delay(100);
  arm();
}

inline void initBalancing(){
  bal_axes = 0;
  bal_ac = 0;
  bal_bd = 0;
}

inline void initRegulators(){

  pitchReg.SetMode(AUTOMATIC);
  pitchReg.SetOutputLimits(-PID_PITCH_INFLUENCE, PID_PITCH_INFLUENCE);
  
  rollReg.SetMode(AUTOMATIC);
  rollReg.SetOutputLimits(-PID_ROLL_INFLUENCE, PID_ROLL_INFLUENCE);
  
  yawReg.SetMode(AUTOMATIC);
  yawReg.SetOutputLimits(-PID_YAW_INFLUENCE, PID_YAW_INFLUENCE);

}

inline void rcInterrupt1(){
   if(!interruptLock) ch1 = micros() - rcLastChange1;
   rcLastChange1 = micros(); 
}

inline void rcInterrupt2(){
  if(!interruptLock) ch2 = micros() - rcLastChange2;
  rcLastChange2 = micros();
}

inline void rcInterrupt3(){
  if(!interruptLock) ch3 = micros() - rcLastChange3;
  rcLastChange3 = micros();
}

inline void rcInterrupt4(){
  if(!interruptLock) ch4 = micros() - rcLastChange4;
  rcLastChange4 = micros();
}

inline void rcInterrupt5(){
  if(!interruptLock) ch5 = micros() - rcLastChange5;
  rcLastChange5 = micros();
}

inline void acquireLock(){
  interruptLock = true; 
}

inline void releaseLock(){
  interruptLock = false;
}
