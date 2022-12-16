/*MODIFIED version of code from how to mechatronics
make sure you have the libraries installed and pins 
CREDIT:https://www.youtube.com/@HowToMechatronics/featured
*/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
/*VARIABLES*/
//dc motor 1 speed and direction control
#define ENA 2  //speed
#define IN1 3  //forward
#define IN2 4  //backward
//dc motor 2 speed and direction control
#define IN3 5  //forward
#define IN4 6 //backward
#define ENB 7 //speed

//Red LED
#define redLED 10 //TURNS on when reversing

//WIFI Module pins
#define MOSI 11
#define MISO 12
#define CSN 9
#define SCK 13
#define CE 8

RF24 radio(CE, CSN);
const byte address[6] = "00001";
char receivedData[32] = "";
int xAxis, yAxis;
int motorSpeedA, motorSpeedB = 0;

void setup() {
//Motor variable setup
  pinMode(ENA, OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  pinMode(ENB,OUTPUT);
  
  //Wifi module setup
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
}

void loop() {
  if (radio.available()) {   // If the NRF240L01 module received data
    radio.read(&receivedData, sizeof(receivedData)); // Read the data and put it into character array
    xAxis = atoi(&receivedData[0]); // Convert the data from the character array (received X value) into integer
    delay(10);
    radio.read(&receivedData, sizeof(receivedData));
    yAxis = atoi(&receivedData[0]);
    delay(10);
  }
  
  // Y-axis used for forward and backward control
  if (yAxis < 470) {
    //red lights turn on
    digitalWrite(redLED,HIGH);
    // Set Motor 1 backward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    // Set Motor 2 backward
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    // Convert the declining Y-axis readings for going backward from 470 to 0 into 0 to 255 value for the PWM signal for increasing the motor speed
    motorSpeedA = map(yAxis, 470, 0, 0, 255);
    motorSpeedB = map(yAxis, 470, 0, 0, 255);
  }
  else if (yAxis > 550) {
    //turn red ligths off if on
    digitalWrite(redLED,HIGH);
    // Set Motor 1 forward
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    // Set Motor 2 forward
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    // Convert the increasing Y-axis readings for going forward from 550 to 1023 into 0 to 255 value for the PWM signal for increasing the motor speed
    motorSpeedA = map(yAxis, 550, 1023, 0, 255);
    motorSpeedB = map(yAxis, 550, 1023, 0, 255);
  }
  // If joystick stays in middle the motors are not moving
  else {
    motorSpeedA = 0;
    motorSpeedB = 0;
  }
  // X-axis used for left and right control
  if (xAxis < 470) {
    // Convert the declining X-axis readings from 470 to 0 into increasing 0 to 255 value
    int xMapped = map(xAxis, 470, 0, 0, 255);
    // Move to left - decrease left motor speed, increase right motor speed
    motorSpeedA = motorSpeedA + xMapped;
    motorSpeedB = motorSpeedB - xMapped;
    // Confine the range from 0 to 255
    if (motorSpeedA < 0) {
      motorSpeedA = 0;
    }
    if (motorSpeedB > 255) {
      motorSpeedB = 255;
    }
  }
  if (xAxis > 550) {
    // Convert the increasing X-axis readings from 550 to 1023 into 0 to 255 value
    int xMapped = map(xAxis, 550, 1023, 0, 255);
    // Move right - decrease right motor speed, increase left motor speed
    motorSpeedA = motorSpeedA - xMapped;
    motorSpeedB = motorSpeedB + xMapped;
    // Confine the range from 0 to 255
    if (motorSpeedA > 255) {
      motorSpeedA = 255;
    }
    if (motorSpeedB < 0) {
      motorSpeedB = 0;
    }
  }
  // Prevent buzzing at low speeds (Adjust according to your motors. My motors couldn't start moving if PWM value was below value of 70)
  if (motorSpeedA < 70) {
    motorSpeedA = 0;
  }
  if (motorSpeedB < 70) {
    motorSpeedB = 0;
  }
  analogWrite(enA, motorSpeedA); // Send PWM signal to motor A
  analogWrite(enB, motorSpeedB); // Send PWM signal to motor B
}