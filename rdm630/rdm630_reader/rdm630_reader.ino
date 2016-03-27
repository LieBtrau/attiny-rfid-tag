#include <SoftwareSerial.h>

#include <rdm630.h>
rdm630 rfid(8, 0);  //TX-pin of RDM630 connected to Arduino pin 8
void setup() {
  // put your setup code here, to run once:
  rfid.begin();
  Serial.begin(9600);
  // while the serial stream is not open, do nothing:
   while (!Serial) ;
  Serial.println("I'm ready, folks.");
}

void loop() {
  byte data[100];
  byte length;
  // put your main code here, to run repeatedly:
  if(rfid.available())
  {
    rfid.getData(data,length);
    for(byte i=0;i<length;i++)
    {
      Serial.print(data[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
}
