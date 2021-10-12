
#include <Servo.h>
Servo air_servo;
Servo tube_servo;
Servo o2_servo;
//serial parse
String myString;
char c;
int Index1,Index2,Index3,Index4,Index5,Index6, Index7;
String secondValue, thirdValue, fourthValue, fifthValue, firstValue, sixthValue;
float volume, volume_set, inspiration, expiration;
float air_servo_deg;
float tube_servo_deg;
int adc;
int zero_pin = 12;
int adc_null;

//o2 var
float o2;
float o2_raw;
float o2_filtered;
float o2_lpm;
float o2_lpm_filtered;
float o2_lpm_desired;

//o2 PID
float o2_set;
float p_control_o2;
float i_control_o2;
float pid_control_o2;
float kp_o2 = 6;
float ki_o2 = 0.2;
float pid_o2;
float o2_error;
float o2_deg;
float o2_valve_pin = 7;

unsigned long time_now;
unsigned long dt;
float dt_second;

unsigned long time_prev;
int air_servo_pin = 9;
int tube_servo_pin = 11;

float p_control;
float kp = 0.23;//best 0.2
float ki = 0.0015; //0.002
float i_control;
float integral_windup;
float pid_control;

float error;

unsigned long breath_time;
unsigned long breath_time_prev;
float breath_freq;
float breath_period;
float breath_in;
float breath_out;

char breath_stat;

#include <ADS1X15.h>

ADS1115 ADS(0x48);

void setup() {
  // initialize serial communication at 9600 bits per second:
  air_servo.attach(air_servo_pin);
  tube_servo.attach(tube_servo_pin);
  o2_servo.attach(o2_valve_pin);
  air_servo_deg = 90;
  air_servo.write(110);
  Serial.begin(9600);
  pinMode(zero_pin, INPUT_PULLUP);
  adc_null = analogRead(A3);
  o2_set = 20;
  ADS.begin();
  ADS.setGain(0);
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input on analog pin 0:
  time_now = millis();
// *volume set | inspiration | expiration | air servo | breath freq |o2 set
//default packet data *0.6|1|1|90|10|30

while (Serial.available()>0)
{
  delay(10);
  c = Serial.read();
  myString += c;
}
if (myString.length()>0)
{
Index1 = myString.indexOf('*');
Index2 = myString.indexOf('|', Index1+1);
Index3 = myString.indexOf('|', Index2+1);
Index4 = myString.indexOf('|', Index3+1);
Index5 = myString.indexOf('|', Index4+1);
Index6 = myString.indexOf('|', Index5+1);
Index7 = myString.indexOf('|', Index6+1);
secondValue = myString.substring(Index1+1, Index2);
thirdValue = myString.substring(Index2+1, Index3);
fourthValue = myString.substring(Index3+1, Index4);
fifthValue = myString.substring(Index4+1, Index5);
sixthValue = myString.substring(Index5+1, Index6);
firstValue = myString.substring(Index6+1, Index7);
//Serial.println(Index1);
volume_set = secondValue.toFloat();
Serial.print("volume:");Serial.println(volume_set);
inspiration = thirdValue.toFloat();
Serial.print("inspiration:");Serial.println(inspiration);
expiration = fourthValue.toFloat();
Serial.print("expiration:");Serial.println(expiration);
air_servo_deg = fifthValue.toFloat();
Serial.print("air servo deg:");Serial.println(air_servo_deg);
breath_freq = sixthValue.toFloat();
breath_period = 60/breath_freq * 1000;
//tube_servo_deg = firstValue.toFloat();
//Serial.print("tube servo deg:");Serial.println(tube_servo_deg);
Serial.print("breath freq:");Serial.println(breath_freq);
o2_set = firstValue.toFloat();
Serial.print("o2 set:");Serial.println(o2_set);
myString="";
}
  breath_in = breath_period*(inspiration/(inspiration+expiration));
  breath_out = breath_period*(expiration/(inspiration+expiration));
  breath_time = (millis() - breath_time_prev);


   adc = analogRead(A3)-adc_null;
  if (digitalRead(zero_pin) == 0){
    adc_null = analogRead(A3);
  }
  o2_lpm = ((float(adc)/776)*150)/0.93;
  // print out the value you read:
  if (o2_lpm < 2){
     o2_lpm = 0;
  }
  
  o2_lpm_filtered = (0.4*o2_lpm) + (0.6*o2_lpm_filtered);
 // if (o2_lpm_filtered < 2){
 //    o2_lpm_filtered = 0;
 // }

  
  
  if (breath_time < breath_in){
    //Serial.print(" in ");
 
  
  //volume base control
  //error = volume_set - volume;

  //flow base control
  
  o2_lpm_desired = volume_set / (breath_in/1000) *60;
 // error = volume_set - o2_lpm;
  error = o2_lpm_desired - o2_lpm_filtered;
  p_control = (kp*error);
  i_control = i_control+ (ki*error)*dt;
  
  if (i_control > 150){
    i_control = 150;
  }
  pid_control = p_control + i_control;
  if (pid_control > 150){
    pid_control = 150;
  }
  
  if (pid_control < 0){
    pid_control = 0;
  }
  
  //exp filter w = 0.6
  tube_servo_deg = pid_control;//(1*pid_control) + (1*tube_servo_deg);
  //tube_servo_deg = 150;
  } else {
  //  Serial.print(" out ");
 //   volume_set = 0;
//    error = 0;
//    p_control = 0;
  //  i_control = 0;
//    pid_control = 0;
    
    tube_servo_deg = 0;
  
  }
  if (breath_time > breath_period){
    volume = 0;
    breath_time_prev = millis();
  }
  /*
  Serial.print(breath_time);
  Serial.print(" ");
  */
  Serial.print(" Ti : ");
  Serial.print(breath_in);

  Serial.print(" e : ");
  Serial.print(error);
  
  Serial.print(" x_dot : ");
  Serial.print(o2_lpm);

  Serial.print(" x_dot_filter : ");
  Serial.print(o2_lpm_filtered);
  
  Serial.print(" x_set : ");
  Serial.print(volume_set);
  volume = volume+ (o2_lpm_filtered/60) *dt_second;
  Serial.print(" x : ");
  Serial.print(volume);
  //Serial.print(" dt_second : ");
 
  dt = (time_now - time_prev);
  dt_second = float(dt)/1000;
  //Serial.print(dt_second);
 // Serial.print(" air servo : ");
 // Serial.print(air_servo_deg);

 //Serial.print(" a servo : ");
  //Serial.print(air_servo_deg);
  air_servo.write(air_servo_deg);
  Serial.print(" Tdeg : ");
  Serial.print(tube_servo_deg);
  tube_servo.write(tube_servo_deg);
 // Serial.print(breath_stat);
  
  
  
  Serial.print(" o2 set : ");
  Serial.print(o2_set);
  o2_read();
  Serial.print(" o2 % : ");
  Serial.print(o2_filtered);
  
  o2_error = o2_set - o2_filtered;
  p_control_o2 = kp_o2 * o2_error;
  i_control_o2 = i_control_o2+ (ki_o2 * o2_error *dt_second);
  pid_control_o2 = p_control_o2 + i_control_o2;
  if(pid_control_o2 > 150){
    pid_control_o2 = 150;
  }

  if(pid_control_o2 < 0){
    pid_control_o2 = 0;
  }
  o2_deg = pid_control_o2;
  Serial.print(" o2 deg:");
  Serial.print(o2_deg);
 // o2_deg = o2_set; //debug
  o2_servo.write(o2_deg);

  Serial.print(" ");
   Serial.print(dt_second);
 Serial.println(""); 
  time_prev = time_now;  
}


void o2_read(){
  int16_t val_01 = ADS.readADC_Differential_0_1();  
  o2_raw = (0.9*val_01) + (0.1*o2_raw);
  o2_filtered = abs(map(o2_raw, 68, 313, 21, 90));
}
