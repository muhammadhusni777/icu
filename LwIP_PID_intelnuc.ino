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
float o2_valve_pin = D6;
int air_servo_pin = D5;
int tube_servo_pin = 4;

float p_control;
float kp = 0.003;//best 0.23
float ki = 0.002; //0.0005
float i_control;
float integral_windup;
float pid_control;

float error;

unsigned long breath_time;
unsigned long breath_time_prev;
float breath_freq = 10;
float breath_period;
float breath_in;
float breath_out;

float hfot_set_lpm;

char breath_stat;


int val_01;
int azimuth =0;
// Update these with values suitable for your network.
byte mac[]    = {  0xDE, 0xED, 0xBA, 0xFE, 0xFE, 0xE6 }; //0x00, 0x80, 0xE1, 0x01, 0x01, 0x01
//byte mac[]    = {  0x00, 0x80, 0xE1, 0x01, 0x01, 0x01 };
float val;
String mode = "standby";
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

    
   
  if (String(topic) == "hoho") {
    Serial.print("Changing output to ");
  /*
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
    */
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

  //pinMode (LED_RED, OUTPUT);
  //pinMode (LED_BLUE, OUTPUT);
  //pinMode (LED_GREEN, OUTPUT);
  
 
  
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
// the loop routine runs over and over again forever:
void loop() {
  // read the input on analog pin 0:

  time_now = millis();
// *volume set | inspiration | expiration | air servo | breath freq |o2 set
//default packet data *0.6|1|1|90|10|30
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
    pid_control = 20;
    volume_set = 0;
    o2_lpm_desired = 0;
    client.publish("status", "do nothin");
 
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
  }

  if(mode == "p/ac"){
    client.publish("status", "pressure assist");
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
  pid_control = p_control + i_control + 20;
  if (pid_control > 150){
    pid_control = 150;
  }
  
  if (pid_control < 0){
    pid_control = 0;
  }
  
  //exp filter w = 0.6
  //(1*pid_control) + (1*tube_servo_deg);
  //tube_servo_deg = 150;
  } else {
  //  Serial.print(" out ");
 //   volume_set = 0;
//    error = 0;
//    p_control = 0;
    i_control = 0;
    pid_control = 0;    
    tube_servo_deg = 0;
  
  }
  if (breath_time > breath_period){
    volume = 0;
    breath_time_prev = millis();
  }
  
  client.publish("status","volume assist");
  }



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
 
  dt = (time_now - time_prev);
  dt_second = float(dt)/1000;
  //Serial.print(dt_second);
 // Serial.print(" air servo : ");
 // Serial.print(air_servo_deg);

 //Serial.print(" a servo : ");
  //Serial.print(air_servo_deg);
  
  if (o2_set < 80){
    air_servo_deg = 120;
  } else {
    air_servo_deg = 63;
  }
  air_servo.write(air_servo_deg);
  Serial.print(" Adeg : ");
  Serial.print(air_servo_deg);
  tube_servo_deg = pid_control;

  Serial.print(" Tdeg : ");
  Serial.print(tube_servo_deg);
  
  tube_servo.write(tube_servo_deg);
  
 // Serial.print(breath_stat);
  
  
  
  Serial.print(" o2 set : ");
  Serial.print(o2_set);
  
  //o2_read();
  o2_time = time_now - o2_time_prev;
  if (o2_time > 2000){
   o2_filtered =(0.9 * abs(map(ADS.readADC_Differential_0_1(), 68, 313, 21, 90))) + (0.1*o2_filtered); 
   o2_time_prev = time_now;
  }

  
  
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


  client.publish("volume",dtostrf(volume,6,3,volume_send));
  client.publish("oxygen", dtostrf(o2_filtered, 6, 3, o2_send));
  client.publish("flow", dtostrf(o2_lpm_filtered, 6, 3, flow_send));
  client.publish("flow_sp", dtostrf(o2_lpm_desired, 6, 3, flow_sp_send));
  client.loop();
  time_prev = time_now;  
  
  IWatchdog.reload();
}

/*
void o2_read(){
  int16_t val_01 = ADS.readADC_Differential_0_1();  
  o2_raw = (0.9*val_01) + (0.1*o2_raw);
  o2_filtered = abs(map(o2_raw, 68, 313, 21, 90));
}
*/
