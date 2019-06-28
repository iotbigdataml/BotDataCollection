/* 
**  SPI.h - Serial Peripherial Interface (SPI) protocol.
**  nRF24L01.h - Radio library
**  RF24.h - Radio library
**  
** See radio documentation at: https://maniacbug.github.io/RF24/classRF24.html 
*/

#include <SPI.h>
#include <SoftwareSerial.h>
#include <RF24.h>
#define RX 9                            // Pin used for the receive port on the Arduino
#define TX 10                           // Pin used for the transmit port on the Arduino

SoftwareSerial mySerial(RX, TX, true);  // Here we define the serial port object


RF24 radio(7, 8);                     // Chip enable (7), Chip Select (8)
const byte address[6] = "01011";      // Radio address - use only the channels that match the
                                      // numbers on your robots.


void setup()
{
  Serial.begin(9600);                 // Debug output in case you are connected to a monitor
  mySerial.begin(9600);       // Open the software serial port. Once we open the port we
                              // print a message and wait a second for things to settle
                              // down.
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
   delay(1000);

}

void loop()
{
  if (radio.available())              // If we have messages, we print them out - otherwise we do nothing but listen.
  {
    char text[64] = "";
    radio.read(&text, sizeof(text));
    Serial.println(text);
    mySerial.write(text,sizeof(text));
//    mySerial.write("\n");
    mySerial.flush();

  }
  else
  {
//    mySerial.write("Not available");
//    mySerial.write("\n");
//    mySerial.flush();
//    DO nothing

  }
  
}
