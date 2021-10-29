/////////////////////////////////////////
/// HUMI PID/////////////////////////////
///REV 2 FINAL///////////////////////////
/////////////////////////////////////////

#include  <TimerOne.h>
#include <avr/wdt.h>
#include "TM1637.h"
#define CLK 12//pins definitions for the module and can be changed to other ports       
#define DIO 11
TM1637 disp(CLK,DIO);
//float count = 25;
#define probe_pin A1
/*
////////////////////
//mode 1 (bawah)
#define lev1 A3
#define lev2 A2
#define lev3 A5
#define lev4 A4

*/


////////////////////
//mode 2 (atas)
#define lev1 A4
#define lev2 A5
#define lev3 A2
#define lev4 A3



#define led_lev1 6
#define led_lev2 5
#define led_lev3 4
#define led_lev4 13
#define temp_sensor A7
#define AC_pin 3
#define buzzer A0



#define led_heater 7
#define led_low_water 9
#define led_probe 8
#define led_high_temp 10

//threshold
int delta_temp;
int delta_temp_max = 20;
int delta_temp_min = 8;
int high_temp_limit = 44;
int temp_probe_time = 300;
int low_temp_state;



unsigned long low_water_time;
unsigned long low_water_time_prev;

//alarm state
int high_temp_state;
int temp_probe_state;
int low_water_state;
int buzzer_state;
//buffer
volatile int i=0;               
volatile boolean zero_cross=0;  

//variable dimmer              
int dim;                    
int inc=1;                      

int freqStep = 75;    

float internal_temp;

//pid
int setpoint;
float sensor;
float sensor_raw;
float sum;
float error;
float P_control;
float kp = 3.7;// 1) 6.5 2) 3         //kp= 3.7
float I_control;
float ki = 0.011;//     0.015; // 1) 0.023 2)0.01        //ki=0.011
float PI_control;
int i_windup;
int saturation = 118;
int power;
int power_min = 20;
int level;
//timer
float Time;
float elapsedTime;
float timePrev;


//variable thermal switch
int thermal_switch = A6;
int thermal_switch_state;

//zc condition
int zc_condition;

bool power_lock;

String status_heater;

unsigned long temp_timer;
unsigned long temp_timer_prev;
void setup() {                                      // Begin setup
  // initialize the LCD
  pinMode(thermal_switch, INPUT_PULLUP);
  pinMode(temp_sensor, INPUT);
  pinMode(lev1,INPUT_PULLUP);
  pinMode(lev2,INPUT_PULLUP);
  pinMode(lev3,INPUT_PULLUP);
  pinMode(lev4,INPUT_PULLUP);
  pinMode(led_lev1, OUTPUT);
  pinMode(led_lev2, OUTPUT);
  pinMode(led_lev3, OUTPUT);
  pinMode(led_lev4, OUTPUT);
  pinMode(led_heater, OUTPUT);
  pinMode(led_low_water, OUTPUT);
  pinMode(led_probe, OUTPUT);
  pinMode(led_high_temp, OUTPUT);
  pinMode(AC_pin, OUTPUT);                          // Set the Triac pin as output
  digitalWrite(AC_pin, LOW);
  pinMode(buzzer, OUTPUT);
  attachInterrupt(0, zero_cross_detect, RISING);    // Attach an Interupt to Pin 2 (interupt 0) for Zero Cross Detection
  Timer1.initialize(freqStep);                      // Initialize TimerOne library for the freq we need
  Timer1.attachInterrupt(dim_check, freqStep);      
  Serial.begin(9600);
  disp.set(7);//BRIGHT_TYPICAL = 2,BRIGHT_DARKEST = 0,BRIGHTEST = 7;
  disp.init(D4056A);//D4056A is the type of the module
  delay(2000);  
  
  wdt_enable(WDTO_500MS);                                 
}

void zero_cross_detect() {    
  zero_cross = true;               // set the boolean to true to tell our dimming function that a zero cross has occured
  i=0;
  digitalWrite(AC_pin, LOW);       // turn off TRIAC (and AC)
}                                 

// Turn on the TRIAC at the appropriate time
void dim_check() {                   
  if(zero_cross == true && power_lock == false) {              
    if(i>=dim) {                     
      digitalWrite(AC_pin, HIGH);        
      i=0;                       
      zero_cross = false; 
    } 
    else {
      i++;                      
    }                                
  }

  else {
    digitalWrite(AC_pin, LOW); //debug                                  
    i++;
  }

   if (i > saturation){
        zc_condition = 0;
        i = saturation;             
      } else{
        zc_condition = 1;
      }                                  
}                                   

void loop() {
 if (level == 0){
  digitalWrite(led_low_water, LOW);
  low_water_state = 0;
 }
 
 if(analogRead(lev1) < 200){
  level = 1;
  setpoint = 33;
  disp.display(1111);
  digitalWrite(led_lev1,HIGH); 
  digitalWrite(led_lev2,LOW);
  digitalWrite(led_lev3,LOW);
  digitalWrite(led_lev4,LOW);
  
  delta_temp_max = 18;
 }
 else if(analogRead(lev2) < 200){
  level = 2;
  setpoint = 35;
  disp.display(2222);
  digitalWrite(led_lev2,HIGH);
  digitalWrite(led_lev1,LOW);
  digitalWrite(led_lev3,LOW);
  digitalWrite(led_lev4,LOW);

  delta_temp_max = 20;
 }
 else if(analogRead(lev3) < 200){
  level = 3;
  setpoint = 37;
  disp.display(3333);
  digitalWrite(led_lev3,HIGH);
  digitalWrite(led_lev1,LOW);
  digitalWrite(led_lev2,LOW);
  digitalWrite(led_lev4,LOW);

  delta_temp_max = 20;
 }
  else if(analogRead(lev4) < 200){
  level = 4;
  setpoint = 39;
  disp.display(4444);
  digitalWrite(led_lev4,HIGH);
  digitalWrite(led_lev1,LOW);
  digitalWrite(led_lev2,LOW);
  digitalWrite(led_lev3,LOW);

  delta_temp_max = 23;
 }
 else{
 // temp_timer = millis() - temp_timer_prev;
 // if (temp_timer > 3000){
  disp.display(sensor);
 // } 
  
 // else {
 // disp.display(-1*internal_temp);
 // }
  
  //if (temp_timer > 6000){
  //  temp_timer_prev = millis();
  //}
  
 }
 //debug
 // digitalWrite(led_chamber,HIGH);
 // digitalWrite(led_probe,HIGH);
 // digitalWrite(led_high_temp,HIGH);
 
   //read sensor
  //internal_temp = (0.1 * (0.1391*analogRead(A1)-46.893)) + (0.9 * internal_temp);
  internal_temp = 45;

   
  for (int i=1; i<=50; i++){
//       sensor_raw = ((0.1155*analogRead(temp_sensor))-16.755);
       sensor_raw = ((0.1391*analogRead(temp_sensor))-46.893);
        sum = sum + sensor_raw;
      }
 sensor = sum/50;
 sum =0;

 delta_temp = internal_temp - sensor;
Serial.println(delta_temp);
 if ((sensor < 33 && internal_temp > 50) || (sensor  < 0)){
  digitalWrite(led_probe, HIGH);
  temp_probe_state = 0;
 } else {
  digitalWrite(led_probe, LOW);
  temp_probe_state = 1;
 }
   //error
 error = setpoint - sensor;
 //low_temp_time = (millis() - low_temp_time_prev)/1000;
  Serial.println(error);
  /*
   low_water_time = (millis() - low_water_time_prev) / 1000;
  if (abs(error) < 2 && power < min_power){
  
    if (low_water_time < 60){
      digitalWrite(led_low_water, LOW);
      low_water_state = 0;
    } else {
      digitalWrite(led_low_water, HIGH); 
      low_water_state = 1;
    }
  } else {
    low_water_time_prev = millis();
    digitalWrite(led_low_water, LOW);
    low_water_state = 0;
  }
*/
   
   if (internal_temp > 65 || sensor > high_temp_limit || thermal_switch_state >= 100){
    digitalWrite(led_high_temp, HIGH);
    high_temp_state = 1;
  } else {
     digitalWrite(led_high_temp, LOW);
     high_temp_state = 0;
  }
 
 /*
 if (low_temp_time > temp_probe_time){
  digitalWrite(led_low_temp, HIGH);
  low_temp_state = 1;
 } else {
  digitalWrite(led_low_temp, LOW);
  low_temp_state = 0;
 }
 */
  



  thermal_switch_state = analogRead(thermal_switch);
  //temp_probe_state = analogRead(probe_pin);
  
 // Serial.println(low_temp_time);

/*  
  if (temp_probe_state >= 100){
    digitalWrite(led_probe, HIGH);
    
  } else {
    digitalWrite(led_probe, LOW);
  }
  */
//  Serial.print("\t");
//  Serial.println(zc_condition);
 // thermal_switch_state = 0;
  if(high_temp_state == 1 || zc_condition == 0 || temp_probe_state == 0 ||low_water_state == 1){
    //low_temp_time_prev = millis();
    digitalWrite(led_heater, LOW);
    buzzer_state = 1;
    dim = 128;
    P_control = 0;
    I_control = I_control;
    PI_control = 0;
  }

  else {
 buzzer_state = 0;
   //proportional control
 P_control = kp * error; 
  if (P_control < 10){
    digitalWrite(led_heater, LOW);
  } else {
    digitalWrite(led_heater, HIGH);
  }

   //integral control
 I_control = ki*error*elapsedTime + I_control;
  
   //integral windup
  if(I_control > i_windup){
    I_control = i_windup;
  } else if (I_control < 0){
    I_control = 0; 
  }
  else {
    I_control = I_control;
  }
  //p+i control
 PI_control = P_control + I_control;
  
   //saturation
  if (PI_control > saturation){
    PI_control = saturation;
    i_windup = saturation-PI_control;
  } else if(PI_control < 0) {
     PI_control = 0;
  }
  else{
    i_windup = saturation;
  }    
  
  //dim = 0; //debug  
  dim = (saturation - PI_control)+10;
  }
  power = map(dim, 128, 10, 0, 100);
  
  //power lock
  if (power < 5){
    power_lock = true;
    status_heater = "OFF"; 
     
  } else {
    power_lock = false;
    status_heater = "ON "; 
    
  }
  //display


  if (buzzer_state == 1){
    digitalWrite (buzzer, HIGH);
  } else {
    digitalWrite (buzzer, LOW);
  }
  //display 2
  
  
    //TIMER
  timePrev = Time;                            // the previous time is stored before the actual time read
  Time = millis();                            // actual time read
  elapsedTime = (Time - timePrev) / 1000; 

  //serial debug
/*
  Serial.print("setpoint : ");
  Serial.print(setpoint);
  Serial.print(" sensor : ");
  Serial.print(sensor);
  Serial.print(" error : ");
  Serial.print(error);
  Serial.print(" P : ");
  Serial.print(P_control);
  Serial.print(" I : ");
  Serial.print(I_control);
  Serial.print(" PI : ");
  Serial.print(PI_control);
  Serial.print(" power : ");
  Serial.print(power);
  Serial.print(" delta t : ");
  Serial.println(elapsedTime);
  */
  delay(100);  
  wdt_reset(); 
}
