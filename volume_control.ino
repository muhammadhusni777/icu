
#include <Servo.h>
Servo air_servo;
Servo tube_servo;

//serial parse
String myString;
char c;
int Index1,Index2,Index3,Index4,Index5,Index6;
String secondValue, thirdValue, fourthValue, fifthValue, firstValue;
float volume, volume_set, inspiration, expiration;
float air_servo_deg;
float tube_servo_deg;
int adc;
int zero_pin = 12;
int adc_null;

// the setup routine runs once when you press reset:
float o2_lpm;
float o2_lpm_filtered;

unsigned long time_now;
unsigned long dt;
float dt_second;

unsigned long time_prev;
int air_servo_pin = 9;
int tube_servo_pin = 11;

float p_control;
float kp = 120;
float error;

unsigned long breath_time;
unsigned long breath_time_prev;
float breath_freq;
float breath_period;
float breath_in;
float breath_out;

char breath_stat;
void setup() {
  // initialize serial communication at 9600 bits per second:
  air_servo.attach(air_servo_pin);
  tube_servo.attach(tube_servo_pin);
  air_servo_deg = 90;
  air_servo.write(110);
  Serial.begin(9600);
  pinMode(zero_pin, INPUT_PULLUP);
  adc_null = analogRead(A3);

}

// the loop routine runs over and over again forever:
void loop() {
  // read the input on analog pin 0:
  time_now = millis();
// *volume set | inspiration | expiration | air servo | tube servo
//packet data *1|1|90|50|50

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
secondValue = myString.substring(Index1+1, Index2);
thirdValue = myString.substring(Index2+1, Index3);
fourthValue = myString.substring(Index3+1, Index4);
fifthValue = myString.substring(Index4+1, Index5);
firstValue = myString.substring(Index5+1, Index6);
//Serial.println(Index1);
volume_set = secondValue.toFloat();
Serial.print("volume:");Serial.println(volume);
inspiration = thirdValue.toFloat();
Serial.print("inspiration:");Serial.println(inspiration);
expiration = fourthValue.toFloat();
Serial.print("expiration:");Serial.println(expiration);
air_servo_deg = fifthValue.toFloat();
Serial.print("air servo deg:");Serial.println(air_servo_deg);
breath_freq = firstValue.toFloat();
breath_period = 60/breath_freq * 1000;
//tube_servo_deg = firstValue.toFloat();
Serial.print("tube servo deg:");Serial.println(tube_servo_deg);
myString="";
}
  breath_in = breath_period*(inspiration/(inspiration+expiration));
  breath_out = breath_period*(expiration/(inspiration+expiration));
  breath_time = (millis() - breath_time_prev);
  if (breath_time < breath_in){
    Serial.print(" breath in ");
  } else {
    Serial.print(" breath out ");
  }
  if (breath_time > breath_period){
    breath_time_prev = millis();
  }
  Serial.print(breath_time);
  Serial.print(" ");
  adc = analogRead(A3)-adc_null;
  if (digitalRead(zero_pin) == 0){
    adc_null = analogRead(A3);
  }
  o2_lpm = ((float(adc)/776)*150)/0.93;
  // print out the value you read:
  o2_lpm_filtered = (0.1*o2_lpm) + (0.9*o2_lpm_filtered);
  if (o2_lpm_filtered < 3){
     o2_lpm_filtered = 0;
  }

  
  





  
  volume = volume+ (o2_lpm_filtered/60) *dt_second;
  error = volume_set - volume;
  p_control = kp*error;
  if (p_control > 180){
    p_control = 180;
  }
  
  if (p_control < 0){
    p_control = 0;
  }
  tube_servo_deg = p_control;

  Serial.print(breath_period);
  /*
  Serial.print(" x_dot : ");
  Serial.print(o2_lpm_filtered);
  Serial.print(" x : ");
  Serial.print(volume);
  //Serial.print(" dt_second : ");
  */
  dt = (time_now - time_prev);
  dt_second = float(dt)/1000;
  //Serial.print(dt_second);
  Serial.print(" air servo : ");
  Serial.print(air_servo_deg);
  air_servo.write(air_servo_deg);
  Serial.print(" tube servo : ");
  Serial.print(tube_servo_deg);
  tube_servo.write(tube_servo_deg);
 // Serial.print(breath_stat);
  
  Serial.println(" ");
  
  time_prev = time_now;  
}
