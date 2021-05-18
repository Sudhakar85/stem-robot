#include <Servo.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#define TRIG_PIN D6
#define ECHO_PIN D7
#define SERVO_PIN D8

#define L_M_F D2 // Driver IN 2
#define L_M_R D3 // Driver IN 1
#define R_M_F D4 // Driver IN 3
#define R_M_R D5 // Driver IN 4 

char t;
Servo myservo;

boolean goesForward = false;
long duration;
int distance = 100;
int speedSet = 0;
bool autoMode = false;

// LED_BUILTIN

const char* ssid = "XXXXX";
const char* password = "XXXX";
const char* mqtt_server = "test.mosquitto.org";
const int mqtt_server_port = 1883;
const char* mqtt_client_id = "sudhakar-stem-car";
char* mqtt_topic_program_run = "sudhakar/stem-car/run";
char* mqtt_topic_alive = "sudhakar/stem-car/alive";
char* mqtt_topic_publish_status = "sudhakar/stem-car/status";

IPAddress ip(192,168,1,144);  // Desired IP Address
IPAddress gateway(192, 168, 1, 254); // set gateway to match your network
IPAddress subnet(255, 255, 255, 0);
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

void setup() {

  Serial.begin(115200);
  delay(50);

  connectWifi(); 
  delay(100);

  /*
  myservo.attach(SERVO_PIN);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  pinMode(L_M_F, OUTPUT);
  pinMode(L_M_R, OUTPUT);
  pinMode(R_M_F, OUTPUT);
  pinMode(R_M_R, OUTPUT);

  digitalWrite(L_M_F, LOW);
  digitalWrite(L_M_R, LOW);
  digitalWrite(R_M_F, LOW);
  digitalWrite(R_M_R, LOW);

  */
   
}

void loop() {


if(!mqttClient.connected()) {
      reconnect();
    }
mqttClient.loop();

  /*
  if (Serial.available()) {
    t = Serial.read();
    Serial.println(t);
  } */


/*
if(t == 'r')
{
  right();
 }
 else if (t == 'l')
 {
  left();
 }*/

 if(autoMode == true)
 {
    myservo.write(115);
    distance = readPing();
    delay(100);
    distance = readPing();
    delay(100);
    distance = readPing();
    delay(100);
    distance = readPing();
    delay(100);  
   runAutoMode();
 }

 delay(200);
  
}

void connectWifi()
{
  WiFi.config(ip, gateway, subnet);  
  WiFi.begin(ssid,password);
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println(WiFi.localIP());

  mqttClient.setServer(mqtt_server,mqtt_server_port);
  mqttClient.setCallback(mqtt_callback);
}

void runAutoMode()
{
  int distanceR = 0;
  int distanceF = 0;
  int distanceL = 0;
  delay(50);
  
  if (distance <= 20)
  {
    stop();
    delay(100);
    reverse();
    delay(600);
    stop();
    
    delay(200);
    distanceR = lookRight();
    delay(200);    
    distanceF = readPing();
    delay(200);
    distanceL = lookLeft();
    delay(200);

    Serial.print("L:");
    Serial.println(distanceL);
    Serial.print("F:");
    Serial.println(distanceF);
    Serial.print("R:");
    Serial.println(distanceR);

/*
    if(distanceF > distanceR || distanceF > distanceL)
    {
      Serial.println("Forward");
      forward();
    }
    */
    if (distanceR >= distanceL)
    {
      Serial.println("Turn Left");
      left();
      stop();      
    } else
    {
      Serial.println("Turn Right");
      right();      
      stop();
    }
  } else
  {
    forward();
  }
  delay(500);
  distance = readPing();
}

void forward()
{
  
    digitalWrite(L_M_F, HIGH);
    digitalWrite(L_M_R, LOW);
    digitalWrite(R_M_F, HIGH);
    digitalWrite(R_M_R, LOW);
    goesForward = true;
    delay(5);  
}

void right()
{
  digitalWrite(L_M_F, LOW);
  digitalWrite(L_M_R, LOW);
  digitalWrite(R_M_F, LOW);
  digitalWrite(R_M_R, HIGH);
  delay(1850);
  forward();
}

void left()
{
  digitalWrite(L_M_F, LOW);
  digitalWrite(L_M_R, HIGH);
  digitalWrite(R_M_F, LOW);
  digitalWrite(R_M_R, LOW);
  delay(1850);
  forward();
}

void reverse()
{
  goesForward = false;
  digitalWrite(L_M_F, LOW);
  digitalWrite(L_M_R, HIGH);
  digitalWrite(R_M_F, LOW);
  digitalWrite(R_M_R, HIGH);
  delay(5);
}

void stop()
{
  digitalWrite(L_M_F, LOW);
  digitalWrite(L_M_R, LOW);
  digitalWrite(R_M_F, LOW);
  digitalWrite(R_M_R, LOW);
}

int lookRight()
{
  myservo.write(50);
  delay(500);
  int distance = readPing();
  delay(500);
  myservo.write(115);
  return distance;
}

int lookLeft()
{
  myservo.write(170);
  delay(500);
  int distance = readPing();
  delay(500);
  myservo.write(115);
  return distance;
  delay(100);
}

int readPing() {  
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH, 180000UL);

  int cm = duration * 0.034/2; // speed of the sound

  Serial.print("distance:");
  Serial.println(cm);
  
  if(cm==0)
  {
    cm = 250;
  }
  
  return cm;
}

// Message Format: Mode, F, delay, left, delay, Stop, delay, Right Delay etc..
void mqtt_callback(char *topic, byte* payload, unsigned int length) {
  Serial.print("message received : ");
  Serial.print(topic);
  
  if( strcmp(topic,mqtt_topic_alive)==0)
  {
    mqttClient.publish(mqtt_topic_publish_status, "Alive");
  }
  else if(strcmp(topic,mqtt_topic_program_run)==0)
  {
    mqttClient.publish(mqtt_topic_publish_status, "Started");
    parseCommand((char*)payload);
    mqttClient.publish(mqtt_topic_publish_status, "Done");
  }
}

void parseCommand(char *payload)
{
   char * token = strtok(payload, ",");
     
    while( token != NULL ) {
        Serial.println(token );
        token = strtok(NULL, ",");
    }  
}
void executeCommand(char runMode, int delayValue)
{
  if (runMode == 'F') { //Forward
    forward();
  }
  else if (runMode == 'L') { // Left
    left();
  }
  else if (runMode == 'R') { // Right
    right();
  }
  else if (runMode == 'S') {  // Stop
    stop();
  }
  else if (runMode == 'A') { // AutoRun
    stop();
  }
  else
  {
    delay(delayValue);
  }
}

void reconnect() {  
  while(!mqttClient.connected()) {
    Serial.println("Attempting MQTT server");
    if(mqttClient.connect(mqtt_client_id)) {
      Serial.println("Connected to MQTT server");
      mqttClient.subscribe(mqtt_topic_program_run);
      mqttClient.subscribe(mqtt_topic_alive);      
    } else {
      Serial.print("failed rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again after 5 seconds");
      delay(5000);
    }
  }  
}
