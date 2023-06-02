// MAXIM RATIAU

// Motor A
int motor1Pin1 = 21;
int motor1Pin2 = 22;
int enable1Pin = 12;
// Motor B
int motor2Pin1 = 18;
int motor2Pin2 = 19;
int enable2Pin = 2;
// IR-sensor
int ir5 = 32;
int ir4 = 33;
int ir3 = 15;
int ir2 = 27;
int ir1 = 14;

// LED'S
int LEDROOD = 23;
int LEDGROEN = 13;

// drukknop
int button = 5;

int isgestopt;
unsigned long startMillis;  //some global variables available anywhere in the program
unsigned long currentMillis;
const unsigned long period = 1000;  //the value is a number of milliseconds

// ultrasone sensor
int trigPin = 26;
int echoPin = 25;

float usread;
float cm;

// ultrasone sensor array
char usarray[5];

void forward();
// Setting PWM properties
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 200;


//MQTT
#include <WiFi.h>
#include <PubSubClient.h>
WiFiClient espClient;
PubSubClient client(espClient);

const char* ssid = "embed"; // je eigen SSID
const char* password = "weareincontrol"; // je WiFi wachtwoord
const char* mqtt_server = "192.168.1.90"; // je IP address van je odroid
const int mqtt_port = 1883;
const char* mqtt_username = "maxim"; // username van je influxDB
const char* mqtt_password = "maxim"; // wachtwoord van je influxDB

void setup() {
  // motorpinnen als outputs:
  pinMode(motor1Pin1, OUTPUT); // draairichting regelen motor A
  pinMode(motor1Pin2, OUTPUT); // draairichting regelen motor A
  pinMode(enable1Pin, OUTPUT); // motor enabelen en de snelheid regelen
  pinMode(motor2Pin1, OUTPUT); // draairichting regelen motor B
  pinMode(motor2Pin2, OUTPUT); // draairichting regelen motor B
  pinMode(enable2Pin, OUTPUT); // motor enabelen en de snelheid regelen

  // ir sensor pinnen als inputs:
  pinMode(ir5, INPUT);
  pinMode(ir4, INPUT);
  pinMode(ir3, INPUT);
  pinMode(ir2, INPUT);
  pinMode(ir1, INPUT);

  //LED'S
  pinMode(LEDROOD, OUTPUT);
  pinMode(LEDGROEN, OUTPUT);

  // button
  pinMode(button, INPUT);

  // ultrasone sensor
  pinMode(trigPin, OUTPUT); // transmitter
  pinMode(echoPin, INPUT); // ontvanger
  
  // motor driver
  digitalWrite(enable1Pin, HIGH); // motor A enabelen
  digitalWrite(enable2Pin, HIGH); // motor B enabelen
  analogWrite(enable1Pin, 150); // snelheid regelen motor A
  analogWrite(enable2Pin, 150); // snelheid regelen motor B

  Serial.begin(115200);

  //MQTT
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting...");
  }
  Serial.println("Connected to wifi.");
  Serial.println("-----------------------");

  client.setServer(mqtt_server, mqtt_port);

  delay(1000);
}

void loop() {
  client.loop();

  while (!client.connected()) 
  {
    if (client.connect("ESP32Client", mqtt_username, mqtt_password)) 
    {
     Serial.println("connected to MQTT");
     Serial.println("-----------------------");
    }
    else 
    {
      Serial.print("failed with state ");
      Serial.println(client.state());
      Serial.println("-----------------------");
      delay(2000);
    }
  }

// ultrasone sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  usread = pulseIn(echoPin, HIGH);
  cm = (usread * 0.0343) / 2;

  dtostrf(cm, 2, 2, usarray);
  Serial.println(usarray);
  delay(500);

// publish
  client.publish("home/auto/ultrasone", usarray); // publishen naar de odroid




// line following
  if((analogRead(ir1) <= 2600) && (analogRead(ir2) <= 2600) && (analogRead(ir3) >= 3800) && (analogRead(ir4) <= 2600) && (analogRead(ir5) <= 2600))
  {
    forward(); 
  }
  if((analogRead(ir1) <= 2600) && (analogRead(ir2) >= 3800) && (analogRead(ir3) <= 2600) && (analogRead(ir4) <= 2600) && (analogRead(ir5) <= 2600))
  {
    turnRight();
  } 
  if((analogRead(ir1) >= 3800) && (analogRead(ir2) <= 2600) && (analogRead(ir3) <= 2600) && (analogRead(ir4) <= 2600) && (analogRead(ir5) <= 2600))
  {
    turnRight();
  }
  if((analogRead(ir1) <= 2600) && (analogRead(ir2) <= 2600) && (analogRead(ir3) <= 2600) && (analogRead(ir4) >= 3800) && (analogRead(ir5) <= 2600))
  {
    turnLeft();
  }
  if((analogRead(ir1) <= 2600) && (analogRead(ir2) <= 2600) && (analogRead(ir3) <= 2600) && (analogRead(ir4) <= 2600) && (analogRead(ir5) >= 3800))
  {
    turnLeft();
  }
  if((analogRead(ir1) <= 2600) && (analogRead(ir2) <= 2600) && (analogRead(ir3) >= 3800) && (analogRead(ir4) >= 3800) && (analogRead(ir5) <= 2600))
  {
    turnLeft();
  }
  if((analogRead(ir1) <= 2600) && (analogRead(ir2) >= 3800) && (analogRead(ir3) >= 3800) && (analogRead(ir4) <= 2600) && (analogRead(ir5) <= 2600))
  {
    turnRight();
  }
  if((analogRead(ir1) >= 3800) && (analogRead(ir2) >= 3800) && (analogRead(ir3) >= 3800) && (analogRead(ir4) <= 2600) && (analogRead(ir5) <= 2600))
  {
    turnRight();
  }
  if((analogRead(ir1) <= 2600) && (analogRead(ir2) <= 2600) && (analogRead(ir3) >= 3800) && (analogRead(ir4) >= 3800) && (analogRead(ir5) >= 3800))
  {
    turnLeft();
  }
  if((analogRead(ir1) >= 3800) && (analogRead(ir2) >= 3800) && (analogRead(ir3) >= 3800) && (analogRead(ir4) >= 3800) && (analogRead(ir5) >= 3800))
  {
    Stop();
    delay(2000);
      forward();
      delay(500);
    for(int i = 0;i <= 5000;){
      i++;
      delay(1);
      if(digitalRead(button) == HIGH){
        Serial.println("button pressed");
          break;
      }
    }
    forward();
    delay(100);
  }
  if((analogRead(ir1) <= 3800) && (analogRead(ir2) <= 3800) && (analogRead(ir3) <= 3800) && (analogRead(ir4) <= 3800) && (analogRead(ir5) <= 3800))
  {
    Stop();
  }
  
  if(cm < 15)
  {
    Stop();
  }

}

void forward(){  //forward

  digitalWrite(motor1Pin1, HIGH); //Right Motor forward Pin 
  digitalWrite(motor1Pin2, LOW);  //Right Motor backward Pin 
  digitalWrite(motor2Pin1, LOW);  //Left Motor forward Pin 
  digitalWrite(motor2Pin2, HIGH); //Left Motor backward Pin 

  digitalWrite(LEDROOD, LOW);
  digitalWrite(LEDGROEN, HIGH);

}

void turnRight(){ //turnRight

  digitalWrite(motor1Pin1, LOW);  //Right Motor forward Pin 
  digitalWrite(motor1Pin2, HIGH); //Right Motor backward Pin  
  digitalWrite(motor2Pin1, LOW);  //Left Motor forward Pin 
  digitalWrite(motor2Pin2, HIGH); //Left Motor backward Pin 

  digitalWrite(LEDROOD, LOW);
  digitalWrite(LEDGROEN, HIGH);

}

void turnLeft(){ //turnLeft

  digitalWrite(motor1Pin1, HIGH); //Right Motor forward Pin 
  digitalWrite(motor1Pin2, LOW);  //Right Motor backward Pin 
  digitalWrite(motor2Pin1, LOW); //Left Motor forward Pin 
  digitalWrite(motor2Pin2, HIGH);  //Left Motor backward Pin 

  digitalWrite(LEDROOD, LOW);
  digitalWrite(LEDGROEN, HIGH);

}

void Stop(){ //stop

  digitalWrite(motor1Pin1, LOW); //Right Motor forward Pin 
  digitalWrite(motor1Pin2, LOW); //Right Motor backward Pin 
  digitalWrite(motor2Pin1, LOW); //Left Motor forward Pin 
  digitalWrite(motor2Pin2, LOW); //Left Motor backward Pin 

  digitalWrite(LEDROOD, HIGH);
  digitalWrite(LEDGROEN, LOW);

}

// MAXIM RATIAU