#include <SoftwareSerial.h>

int rx_pin = 6;     // setting digital pin 6 to be the receiving pin
int tx_pin = 7;
SoftwareSerial Bt(rx_pin,tx_pin);
void setup()  // Called only once per startup
{ 
  Serial.begin(9600);
  pinMode(rx_pin, INPUT);  // receiving pin as INPUT
  pinMode(tx_pin, OUTPUT); // transmitting pin as OUTPUT  
  bluetoothInitiate();
} 

void bluetoothInitiate() {

 // this part is copied from the Seeeduino example*
 Bt.begin(9600); // this sets the the module to run at the default bound rate
}
void loop() {
  // put your main code here, to run repeatedly:
}
