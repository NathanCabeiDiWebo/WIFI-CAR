/*MODIFIED version of code from how to mechatronics
make sure you have the libraries installed and pins 
CREDIT:https://www.youtube.com/@HowToMechatronics/featured
*/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

/*VARIABLES*/
//WIFI Module pins
#define MOSI 11
#define MISO 12
#define CSN 9
#define SCK 13
#define CE 8

//to transfer information (5 char string)
RF24 radio(CE,CSN);
const byte address[6] = "00001";
char xyData[32] = "";
int xAxis,yAxis;

void setup() {
  //module set up
  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
}

void loop() {
  //Read X And Y values
  xAxis = analogRead(A1); 
  yAxis = analogRead(A0); 
  // X value
  xAxis.toCharArray(xyData, 5); // Put the String (X Value) into a character array
  radio.write(&xyData, sizeof(xyData)); // Send the array data (X value) to the other NRF24L01 modile
  // Y value
  yAxis.toCharArray(xyData, 5);
  radio.write(&xyData, sizeof(xyData));
  delay(20);

  //check if the thing works lmao
  Serial.print("\n");
  Serial.print("xAxis: ");
  Serial.print(analogRead(xAxis));
  Serial.print("\n");
  Serial.print("yAxis: ");
  Serial.println(analogRead(yAxis));
  Serial.print("\n\n");
  delay(1000);
}