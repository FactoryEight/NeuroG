/****************************************************************************************************************
* Description: Program to measure heart rate, GSR, and brain activity and send to Google cloud for analysis
*              to determine onset of Anxiety.
*              Uses ???? for heart rate monitoring
*              Uses ???? for GSR
*              Uses Neurosky mobile headset for measuring brain activity
*****************************************************************************************************************/
#define DEBUG 1                        // Will output messages to the serial monitor
#define USE_ARDUINO_INTERRUPTS true    // Set-up low-level interrupts for most acurate BPM math.
#define BAUDRATE 57600                 // The default for the bluesmirf

#include "Arduino.h"
#include "Board.h"
#include "Helium.h"
#include "HeliumUtil.h"
#include <PulseSensorPlayground.h>     // Includes the PulseSensorPlayground Library.  

#define CHANNEL_NAME "NeuroG"          // The name of the Helium channel configured to send data to Google cloud

//  Variables
const int PulseWire = 0;               // PulseSensor PURPLE WIRE connected to ANALOG PIN 0
const int GSRPin=A1;                   // GSR connected to Analog Pin 1
const int LED13 = 13;                  // The on-board Arduino LED, close to PIN 13.
int Threshold = 550;                   // Determine which Signal to "count as a beat" and which to ignore.
                             
PulseSensorPlayground pulseSensor;     // Creates an instance of the PulseSensorPlayground object called "pulseSensor"

Helium  helium(&atom_serial);          // Standard Helium objects
Channel channel(&helium);


void channel_send(void const * data, size_t len)  // used to send data to Helium channel
{
    int    status;
    int8_t result;

    status = channel.send(data, len, &result);
}

byte ReadOneByte()                     // Returns a byte from the Neurosky headset which is connected via bluesmirf bluetooth adaptor
{
  int ByteRead;

  while(!Serial.available());          // wait until there's some data then 
  ByteRead = Serial.read();            // get a single byte of data 

  #if DEBUG  
    Serial.print((char)ByteRead);      // echo the same byte out the USB serial (for debug purposes)
  #endif

  return ByteRead;
}


String getMindwaveData()               // read the data stream coming from the Mindwave and split into components
{                                      // adapted from http://developer.neurosky.com/docs/doku.php?id=mindwave_mobile_and_arduino
  // checksum variables
  byte generatedChecksum = 0;
  byte checksum = 0; 
  int payloadLength = 0;
  byte payloadData[64] = {0};
  
  // system variables
  long lastReceivedPacket = 0;
  boolean bigPacket = false;
  String MindwaveData;                 // a json formatted string of the values returned from the mindwave headset

  
  byte poorQuality = 0;                // indicates quality of signal from mindwave headset
  byte attention = 0;                  // level of attention reading as determined by mindwave headset
  byte meditation = 0;                 // level of meditation reading as determined by mindwave headset

  // Look for sync bytes
  while(ReadOneByte() != 170);

    if(ReadOneByte() == 170) {

      payloadLength = ReadOneByte();
      if(payloadLength > 169)                      //Payload length can not be greater than 169
          return;

      generatedChecksum = 0;        
      for(int i = 0; i < payloadLength; i++) {  
        payloadData[i] = ReadOneByte();            //Read payload into memory
        generatedChecksum += payloadData[i];
      }   

      checksum = ReadOneByte();                      //Read checksum byte from stream      
      generatedChecksum = 255 - generatedChecksum;   //Take one's compliment of generated checksum

        if(checksum == generatedChecksum) {    

        poorQuality = 200;
        attention = 0;
        meditation = 0;

        for(int i = 0; i < payloadLength; i++) {    // Parse the payload
          switch (payloadData[i]) {
          case 2:
            i++;            
            poorQuality = payloadData[i];
            bigPacket = true;            
            break;
          case 4:
            i++;
            attention = payloadData[i];                        
            break;
          case 5:
            i++;
            meditation = payloadData[i];
            break;
          case 0x80:
            i = i + 3;
            break;
          case 0x83:
            i = i + 25;      
            break;
          default:
            break;
          } // switch
        } // for loop

        // save the data into a json format string
        if(bigPacket) {
          MindwaveData += "\"poorQuality\":" + String(poorQuality) + ",";
          MindwaveData += "\"attention\":" + String(attention) + ",";
          MindwaveData += "\"meditation\":" + String(meditation);
        }
      }
    }

  return(MindwaveData);
}

String getHeartRate()                                     // read the heart rate from the pulse sensor
{                                                         // adapted from https://pulsesensor.com/pages/getting-advanced
  String HeartRate="";
 
  int myBPM = pulseSensor.getBeatsPerMinute();            // Calls function on our pulseSensor object that returns BPM as an "int".
                                                          // "myBPM" hold this BPM value now.
  if (pulseSensor.sawStartOfBeat()) {                     // Constantly test to see if "a beat happened".
    HeartRate = "\"heartRate\":" + String(myBPM) + ",";   // Save the heart rate as a json string
  }
  return(HeartRate);

}



String getGSR()                                           // read the galvanic skin response from the sensor on analogue pins
{
  int sensorValue=0;                                      // used for a single reading
  int gsr_average=0;                                      // to average 10 readings
  long sum=0;
  String GSR;                                             // json formatted string to return
  
  for(int i=0;i<10;i++)                                   // Average the 10 measurements to remove the glitch
     {
     sensorValue=analogRead(GSRPin);                      // GSR sensor connected to analogue pin
     sum += sensorValue;
     delay(5);
     }
  gsr_average = sum/10;                                   // calculate the average of the 10 readings
  GSR = "\"GSR\":" + String(gsr_average) + ",";

  return(GSR);
}

String readSensorMeasurements()                           // read from the 3 sensors and concatenate into a single string
{  
  String totalMessage;
 
  totalMessage = "{" ;
  totalMessage += getHeartRate();
  totalMessage += getGSR();
  totalMessage += getMindwaveData();
  totalMessage += "}";
  
  return(totalMessage);
}


void setup()
{
    Serial.begin(BAUDRATE);
    DBG_PRINTLN(F("Starting"));

    // Begin communication with the Helium Atom The baud rate differs
    // per supported board and is configured in Board.h
    helium.begin(HELIUM_BAUD_RATE);

    // Connect the Atom to the Helium Network
    DBG_PRINTLN(F("Connecting"));
    helium_connect(&helium);
    // and do a channel connect
    DBG_PRINTLN(F("Creating Channel"));
    channel_create(&channel, CHANNEL_NAME);
    DBG_PRINTLN(F("Channel created"));

   // Configure the PulseSensor object, by assigning our variables to it.
   pulseSensor.analogInput(PulseWire);  
   pulseSensor.blinkOnPulse(LED13);       //auto-magically blink Arduino's LED with heartbeat.
   pulseSensor.setThreshold(Threshold);  
  
   // Double-check the "pulseSensor" object was created and "began" seeing a signal.
    if (pulseSensor.begin()) {
     Serial.println("Checking......");  //This prints one time at Arduino power-up,  or on Arduino reset.  
    }
}

void loop()
{

    // Send some data to the configured channel
    int8_t result;
    String sensorMeasurements;
    size_t used;

    DBG_PRINTLN(F("Reading Sensors"));
    sensorMeasurements = readSensorMeasurements();
    used = sensorMeasurements.length();
    
    DBG_PRINT(F("Measurements - "));
    DBG_PRINTLN(sensorMeasurements);
    
    channel_send(sensorMeasurements.c_str(), used);
    DBG_PRINTLN(F("Sent to Helium"));

    // Wait a while till the next time
    delay(5000);
}

