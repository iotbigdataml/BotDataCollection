/* File: RadioRcvr
** Course: 17640
** Project: IoT Order Fulfillment Center
** Copyright: Copyright (c) 2018 Carnegie Mellon University (ajl)
** Versions:
** 1.0 April 2018 - Initial write (ajl).
**
** Description: This class serves as an example for how to write an 
** application for the robot to use the radio to receive data from a PC/Mac
** via the radio base station.
** This example could be used as a basis for writing an application to 
** control and get status to-from Arduinos on the order fulfillment robots.
**
** Parameters: None
**
** Internal Methods:
**  None
**
** External Dependencies: 
**
**  SPI.h - Serial Peripherial Interface (SPI) protocol.
**  nRF24L01.h - Radio library
**  RF24.h - Radio library
**  
** See radio documentation at: https://maniacbug.github.io/RF24/classRF24.html 
*/

#include <SPI.h>
//#include <nRF24L01.h>
#include <RF24.h>


RF24 radio(7, 8);                     // Chip enable (7), Chip Select (8)
const byte address[6] = "01011";      // Radio address - use only the channels that match the
                                      // numbers on your robots.


void setup()
{
  Serial.begin(9600);                 // Debug output in case you are connected to a monitor
  radio.begin();                      // Instantiate the radio object

  if (!radio.isChipConnected())       // See if the radio is connect. Loop while its not connected
  {
    Serial.print("RADIO NOT CONNECTED!");
    while (!radio.isChipConnected());
  }

  radio.openReadingPipe(0, address);  // Open the radio pipe using your address (read about pipes and channels)
  radio.setPALevel(RF24_PA_MIN);      // Set the power level. Since the bots and the radio base station are close I use min power
  radio.startListening();             // Go into receive mode.
  Serial.println("Radio Ready...");
}

void loop()
{
  if (radio.available())              // If we have messages, we print them out - otherwise we do nothing but listen.
  {
    char text[32] = "";
    radio.read(&text, sizeof(text));
//    Serial.print("Received:: ");
    Serial.println(text);
  }
}
