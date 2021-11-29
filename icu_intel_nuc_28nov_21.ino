#include <MapFloat.h>

#include <Servo.h>

#include <IWatchdog.h>

#include <ADS1X15.h>
ADS1115 ADS(0x48);

#include <LwIP.h>
#include <STM32Ethernet.h>



Servo air_servo;
Servo tube_servo;
Servo o2_servo;
Servo test_servo;

/*
mosquitto_pub -h 123.45.0.10 -p 1883 -t hoho -m *sudut servo*
*/

//#include <Wire.h>
//#include <MechaQMC5883.h>
//#include <SPI.h>
//#include <Ethernet.h>
#include <PubSubClient.h>
//serial parse
String myString;
char c;
int Index1,Index2,Index3,Index4,Index5,Index6, Index7;
String secondValue, thirdValue, fourthValue, fifthValue, firstValue, sixthValue;
float volume, volume_set = 0.3, inspiration = 1, expiration = 1;
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
float o2_lpm_filtered_raw;
float o2_lpm_desired;
float o2_lpm_sum;
//o2 PID
float o2_set = 17;
float p_control_o2;
float i_control_o2;
float pid_control_o2;
float kp_o2 = 4;
float ki_o2 = 0.09;
float pid_o2;
float o2_error;
float o2_deg;
unsigned long o2_time;
unsigned long o2_time_prev;

unsigned long time_now;
unsigned long dt;
float dt_second;

unsigned long time_prev;
float o2_valve_pin = D7;
int air_servo_pin = D5;
int tube_servo_pin = D4;
int relay_pin = D8;

//tube valve
float p_control;
float kp = 0.003;//best 0.23
float ki = 0.002; //0.0005
float i_control;
float integral_windup;
float pid_control;
float tube_valve_val = 150;


float error;

unsigned long breath_time;
unsigned long breath_time_prev;
float breath_freq = 10;
float breath_period;
float breath_in;
float breath_out;

float hfot_set_lpm;
float tube_valve_manual = 150; //cpap debug
float o2_valve_manual;
float air_valve_manual;
char breath_stat;


//peep valve
int peep_pin = 3;
float peep_pwm;
float peep_zero;
float peep_value;

//pressure control
float pressure_adc;
float pressure;
float pressure_filtered;
float prev_pressure;
float sp_pressure;
float sp_pressure_prev;
float error_pressure;
float error_pressure_prev;
float kp_pressure = 1;
float ki_pressure = 0.5;
float kd_pressure = 0.05;
float p_control_pressure;
float i_control_pressure;
float d_control_pressure;
float pi_control_pressure;
float pid_control_pressure;
int val_01;
int azimuth =0;
// Update these with values suitable for your network.
byte mac[]    = {  0xDE, 0xED, 0xBA, 0xFE, 0xFE, 0xE6 }; //0x00, 0x80, 0xE1, 0x01, 0x01, 0x01
//byte mac[]    = {  0x00, 0x80, 0xE1, 0x01, 0x01, 0x01 };
float val;

unsigned long mqtt_time;
unsigned long mqtt_time_prev;

unsigned long derivative_time;
unsigned long derivative_time_prev;

String pac_status = "in";

//String mode = "standby";
String mode = "p/ac";
IPAddress ip(123, 45, 0, 106);
IPAddress server(123, 45, 0, 10);




void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.println("] ");
  
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    //Serial.print((char)message[i]);
    messageTemp += (char)message[i];
    
  }
  //val = messageTemp.toFloat();
 
  //Serial.println();
   if (String(topic) == "test_topic") {
    val = messageTemp.toFloat(); 
   }
  
  if (String(topic) == "mode_set") {
    mode = messageTemp; 
   }
   if (String(topic) == "volume_set") {
    volume_set = messageTemp.toFloat(); 
   }

  if (String(topic) == "pressure_set") {
    sp_pressure = messageTemp.toFloat(); 
   }
  
    if (String(topic) == "inspiration") {
    inspiration = messageTemp.toFloat(); 
   }

    if (String(topic) == "expiration") {
    expiration = messageTemp.toFloat(); 
   }

    if (String(topic) == "breath_freq") {
    breath_freq = messageTemp.toFloat(); 
   }

   if (String(topic) == "hfot_set_lpm") {
    hfot_set_lpm = messageTemp.toFloat(); 
   }
   
   if (String(topic) == "o2_set") {
    o2_set = messageTemp.toFloat(); 
   }

    if (String(topic) == "tube_valve") {
    tube_valve_manual = messageTemp.toFloat(); 
   }
   if (String(topic) == "o2_valve") {
    o2_valve_manual = messageTemp.toFloat(); 
   }
   if (String(topic) == "air_valve") {
    air_valve_manual = messageTemp.toFloat(); 
   }

   if (String(topic) == "peep") {
    peep_value = messageTemp.toFloat(); 
   }
   
  if (String(topic) == "hoho") {
    Serial.print("Changing output to ");  
    if(messageTemp == "1on"){
      Serial.println("on");
      digitalWrite(LED_RED, HIGH);
    }
    else if(messageTemp == "1off"){
      Serial.println("off");
      digitalWrite(LED_RED, LOW);
    }
    else if(messageTemp == "2on"){    
      Serial.println("off");  
      digitalWrite(LED_GREEN, HIGH);
    }
     else if(messageTemp == "2off"){
      Serial.println("off");
      digitalWrite(LED_GREEN, LOW);
    }
     else if(messageTemp == "3on"){    
      Serial.println("off");  
      digitalWrite(LED_BLUE, HIGH);
    }
     else if(messageTemp == "3off"){
      Serial.println("off");
      digitalWrite(LED_BLUE, LOW);
    }
    
  }
 messageTemp ="";
}

EthernetClient ethClient;
PubSubClient client(ethClient);
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("STM32-ICU")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      //client.publish("outTopic","hello world");
      // ... and resubscribe
      client.subscribe("hoho");
      client.subscribe("test_topic");
      client.subscribe("volume_set");
      client.subscribe("inspiration");
      client.subscribe("expiration");
      client.subscribe("breath_freq");
      client.subscribe("o2_set");
      client.subscribe("mode_set");
      client.subscribe("hfot_set_lpm");
      client.subscribe("air_valve");
      client.subscribe("tube_valve");
      client.subscribe("o2_valve");
      client.subscribe("pressure_set");
      client.subscribe("peep");
      client.subscribe("debug");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  } 
}

void setup() {
 
  air_servo.attach(air_servo_pin);
  tube_servo.attach(tube_servo_pin);
  o2_servo.attach(o2_valve_pin);
  //test_servo.attach(2);
  air_servo_deg = 90;
  air_servo.write(110);
   
   Serial.begin(115200);
  Serial.println("booting....");
  pinMode(zero_pin, INPUT_PULLUP);
  adc_null = analogRead(A0);
  o2_set = 17;
  
  client.setServer(server, 1883);
  client.setCallback(callback);

  pinMode (LED_RED, OUTPUT);
  pinMode (LED_BLUE, OUTPUT);
  pinMode (LED_GREEN, OUTPUT);
  pinMode(peep_pin, OUTPUT);
  pinMode(relay_pin, OUTPUT);
  
  Ethernet.begin(mac, ip);
  // Allow the hardware to sort itself out
  delay(1500);
  //qmc.setMode(Mode_Continuous,ODR_200Hz,RNG_2G,OSR_256);
  IWatchdog.begin(4000000);
  ADS.begin();
}


static char volume_send[15];
static char o2_send[15];
static char flow_send[15];
static char flow_sp_send[15];
static char pressure_send[15];
static char peep_pwm_send[15];

// the loop routine runs over and over again forever:
void loop() {
  // read the input on analog pin 0:

  time_now = millis();
if (!client.connected()) {
    reconnect();
  }
   adc = analogRead(A0)-adc_null;
  if (digitalRead(zero_pin) == 0){
    adc_null = analogRead(A0);
  }
  

 for (int i=1; i<=150; i++){
  o2_lpm = ((float(adc)/776)*150)/0.93;
  o2_lpm_sum = o2_lpm_sum + o2_lpm;
 }
  o2_lpm_filtered_raw = o2_lpm_sum/150;
  if (o2_lpm_filtered_raw < 2){
    o2_lpm_filtered_raw = 0;
  }
  o2_lpm_sum = 0;
  o2_lpm_filtered = (0.1*o2_lpm_filtered_raw) + (0.9*o2_lpm_filtered);
  
  
  if(mode == "standby"){
    volume_set = 0;
    o2_lpm_desired = 0;
    client.publish("status", "do nothin");
    tube_servo_deg = 20;
  }


  if (mode == "debug"){
  analogWrite(peep_pin, peep_pwm);
  tube_servo_deg = tube_valve_manual;
  o2_deg = o2_valve_manual;
  air_servo_deg = air_valve_manual;
  peep_pwm = peep_value;
  client.publish("status", "debug");
  
  }
  
  if(mode == "hfot"){
  o2_lpm_desired = hfot_set_lpm;
  error = o2_lpm_desired - o2_lpm_filtered;
  p_control = (kp*error);
  i_control = i_control+ (ki*error)*dt;
  if (i_control > 150){
    i_control = 150;
  }
  if (i_control < 0){
    i_control = 0;
  }
  pid_control = p_control + i_control + 0;
  if (pid_control > 150){
    pid_control = 150;
  }
  
  if (pid_control < 0){
    pid_control = 0;
  }
  client.publish("status", "high flow oxygen");
  tube_servo_deg = pid_control;
  o2_control();
  }

  if(mode == "p/ac"){
    tube_servo_deg = 150;
    
    kp_pressure = 1;
    ki_pressure = 1;
    kd_pressure = 0;
    breath_period = 60/breath_freq * 1000;
    breath_in = breath_period*(inspiration/(inspiration+expiration));
    breath_out = breath_period*(expiration/(inspiration+expiration));
    breath_time = (millis() - breath_time_prev);
    if (breath_time < breath_in){
      
    digitalWrite(relay_pin,HIGH);
    pac_status = "in";
    pressure_control();
    peep_pwm = pid_control_pressure;
    } else {
    pressure_control();
    digitalWrite(relay_pin,LOW);
    i_control_pressure = 0;
    pac_status = "out";
    peep_pwm = 0;
    }
   
    if (breath_time > breath_period){
    breath_time_prev = millis();
    }
    client.publish("status", "pressure assist");
    }

  if(mode == "cpap"){
    kp_pressure = 1;
    ki_pressure = 0.5;
    kd_pressure = 0.01;
    tube_servo_deg = tube_valve_manual;
    o2_deg = o2_valve_manual;
    air_servo_deg = air_valve_manual;
    peep_pwm = peep_value;
    client.publish("status", "constant pressure");
    pressure_control();
    digitalWrite(relay_pin,HIGH);
    peep_pwm = pid_control_pressure;
  }
  
  if (mode == "v/ac"){
  breath_period = 60/breath_freq * 1000;
  breath_in = breath_period*(inspiration/(inspiration+expiration));
  breath_out = breath_period*(expiration/(inspiration+expiration));
  breath_time = (millis() - breath_time_prev);
    
  if (breath_time < breath_in){
  o2_lpm_desired = volume_set / (breath_in/1000) *60 + 2;
  error = o2_lpm_desired - o2_lpm_filtered;
  p_control = (kp*error);
  i_control = i_control+ (ki*error)*dt;
  
  if (i_control > 150){
    i_control = 150;
  }
  pid_control = p_control + i_control + 40;
  if (pid_control > 150){
    pid_control = 150;
  }
  
  if (pid_control < 0){
    pid_control = 0;
  }
  pressure_control();
  peep_pwm = pi_control_pressure;
  digitalWrite(relay_pin, HIGH);
  
  } else {
  volume = 0;
  i_control = i_control;
  pid_control = pid_control;    
    //tube_servo_deg = 30;
  peep_pwm = 0;
  digitalWrite(relay_pin, LOW);
  }
  if (breath_time > breath_period){
    volume = 0;
    breath_time_prev = millis();
  }
  
  client.publish("status","volume assist");
  o2_control();
  tube_servo_deg = pid_control;
  }
/*
  Serial.println(mode);
  
  Serial.print(" adc : ");
  Serial.print(adc);
  Serial.print(" e : ");
  Serial.print(error);
  Serial.print(" x_dot : ");
  Serial.print(o2_lpm_filtered);
  Serial.print(" x_set : ");
  Serial.print(volume_set);
  volume = volume+ (o2_lpm_filtered/60) *dt_second;
  Serial.print(" x : ");
  Serial.print(volume);
  //Serial.print(" dt_second : ");
 */
  dt = (time_now - time_prev);
  dt_second = float(dt)/1000;

  /*--------actuator code------------*/
  //air servo
  air_servo.write(air_servo_deg);
  //Serial.print(" Adeg : ");
  //Serial.print(air_servo_deg);
  //tube servo
  //Serial.print(" Tdeg : ");
  //Serial.print(tube_servo_deg);
  tube_servo.write(tube_servo_deg);
  //o2 valve
  //Serial.print(" o2 deg : ");
  //Serial.print(o2_deg);
  o2_servo.write(o2_deg);
  
  //peep valve
  analogWrite(peep_pin, peep_pwm); 
  //Serial.print("peep : ");
  //Serial.println(peep_pwm);
  
 // Serial.print(breath_stat);  
 // Serial.print(" o2 set : ");
 // Serial.print(o2_set);
  
   
  //o2_read() / i2c read;
  o2_time = time_now - o2_time_prev;
  if (o2_time > 500){
   o2_filtered =(0.9 * abs(map(ADS.readADC_Differential_0_1(), 68, 313, 21, 90))) + (0.1*o2_filtered); 
   o2_time_prev = time_now;
   pressure_adc = ADS.readADC_Differential_2_3();
   pressure = mapFloat(pressure_adc,171, 183, 0, 7);
  
   
   if (pressure < 0){
    pressure = 0;
   }
   
  }

//  Serial.print(" o2 % : ");  
//  Serial.print(o2_filtered);
  mqtt_time = (millis() - mqtt_time_prev);
  if (mqtt_time > 200){
  Serial.print(pac_status); 
  Serial.print(" sp: ");
  Serial.print(sp_pressure);
  Serial.print(" e: ");
  Serial.print(error_pressure);
  Serial.print(" pv: ");
  Serial.print(pressure_filtered);
  Serial.print(" pv(Tn-2) : ");
  Serial.print(prev_pressure);
  Serial.print(" p: ");
  Serial.print(p_control_pressure);
  Serial.print(" i: ");
  Serial.print(i_control_pressure);
  Serial.print(" d: ");
  Serial.print(d_control_pressure);
  //Serial.print(" pi: ");
  //Serial.print(pi_control_pressure);
  Serial.print(" pid: ");
  Serial.println(pid_control_pressure);
  client.publish("volume",dtostrf(volume,6,3,volume_send));
  client.publish("oxygen", dtostrf(o2_filtered, 6, 3, o2_send));
  client.publish("flow", dtostrf(o2_lpm_filtered, 6, 3, flow_send));
  client.publish("flow_sp", dtostrf(o2_lpm_desired, 6, 3, flow_sp_send));
  client.publish("pressure", dtostrf(pressure, 6, 3, pressure_send));
  mqtt_time_prev = millis();
  //Serial.println("mqtt send");
  }
  client.loop();
  time_prev = time_now;  
  
  IWatchdog.reload();


}

void o2_control(){
  if (o2_set < 80){
    air_servo_deg = 120; // open normal
  } else {
    air_servo_deg = 63; //nearly close
  }
  
  //o2 control 
  o2_error = o2_set - o2_filtered;
  p_control_o2 = kp_o2 * o2_error;
  i_control_o2 = i_control_o2 + (ki_o2 * o2_error *dt_second);
  pid_control_o2 = p_control_o2 + i_control_o2;
  if(pid_control_o2 > 150){
    pid_control_o2 = 150;
  }

  if(pid_control_o2 < 0){
    pid_control_o2 = 0;
  }
  o2_deg = pid_control_o2;
//  Serial.print(" o2 deg:");
//  Serial.print(o2_deg);
 // o2_deg = o2_set; //debug
//  Serial.print(" ");
//  Serial.print(dt_second);
//  Serial.println(""); 
}


void pressure_control(){
  pressure_filtered = (0.01*pressure) + (0.99*pressure_filtered);
  error_pressure = sp_pressure - pressure_filtered;
  p_control_pressure = kp_pressure * error_pressure;
  i_control_pressure = i_control_pressure + (ki_pressure * error_pressure*dt_second);

   if (i_control_pressure > 80){
    i_control_pressure = 80;
  }

   if (i_control_pressure < 0){
    i_control_pressure = 0;
  }

   
  derivative_time = millis() - derivative_time_prev;
  if (derivative_time > 1000){
  d_control_pressure = kd_pressure*(prev_pressure - pressure_filtered);
  
  derivative_time_prev = millis();
  //Serial.print(derivative_time);
  prev_pressure = pressure_filtered;
  }
  pi_control_pressure = p_control_pressure + i_control_pressure;
 
  if (pi_control_pressure >80){
  pi_control_pressure = 80;
  }
  if (pi_control_pressure < 0){
   pi_control_pressure = 0;  
  }
   pid_control_pressure = pi_control_pressure + d_control_pressure;
  if (pid_control_pressure > 80){
   pid_control_pressure = 80;
  }

   
  if (sp_pressure_prev != sp_pressure){
    p_control_pressure = 0;
    i_control_pressure = 0;
    pi_control_pressure = 0;
  }
  
  
  sp_pressure_prev = sp_pressure;
  //prev_pressure = pressure;
}
