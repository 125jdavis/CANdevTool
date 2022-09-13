// CAN Development Tool
// Jesse Davis
// 9/4/2022
// This tool can read and write to CAN bus, and also has provision to
// read and filter a 0-5V sensor, and report the value of that sensor
// via OLED display, for quick sanity checks. CAN bus values from this
// sensor will be Acutal Volts*100. If any function is not desired, it
// can be commented out in the main loop. 

///// LIBRARIES /////
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <SPI.h>
#include <Wire.h>
#include <mcp_can.h>


///// DEFINE /////
#define OLED_RESET 4
#define CAN0_INT 2
Adafruit_SSD1306 display(OLED_RESET);
MCP_CAN CAN0(10);     // Set CS to pin 10


///// GLOBAL VARIABLES /////
float sensor_1;
int filter_1 = 6;
int analogPin = A7;
int timer1, timer2;
byte data[8] = {0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte canMessageData [8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned long rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128]; 
int CANsendRate = 50;
int dispUpdateRate = 100;



///// SETUP LOOP /////
void setup() {

  Serial.begin(115200); // open the serial port at 9600 bps:
  
  // Initialize display
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (128x32) 
  display.display();               // display image on buffer (splash screen)
  delay(1000);  

  // Initialize MCP2515 running at 8MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) Serial.println("MCP2515 Initialized Successfully!");
  else Serial.println("Error Initializing MCP2515...");

  // Set up CAN interrupt pin
  pinMode(CAN0_INT, INPUT); 
  CAN0.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted
}



///// MAIN LOOP /////
void loop() {
    
//    //read analog voltage and get filtered value
//    sensor_1 = readSensor(analogPin,sensor_1,filter_1);   
//
//    //display update timer
//    if (millis() - timer1 > dispUpdateRate) {  
//        timer1 = millis();        // reset timer1       
//        dispUpdate();             // update OLED display
//        timer1 = millis();
//      }
//
//    //Send CAN messages at specified rate
//    if (millis() - timer2 > CANsendRate) {  
//        timer2 = millis();        // reset timer1       
//        sendCAN(sensor_1);       
//        timer2 = millis();
//      }

      //Read CAN messages as they come in
      if(!digitalRead(CAN0_INT)){     // If CAN0_INT pin is low, read receive buffer
          receiveCAN ();
          parseCAN( rxId, rxBuf);
      }
}





///// FUNCTIONS /////

int readSensor(int inputPin, int oldVal, int filt)  // read voltage, map to 0-5v, and filter
{
    int raw = analogRead (inputPin);
    int newVal = map( raw, 0, 1023, 0, 1600);  
    int filtVal = ((newVal*filt) + (oldVal*(16-filt)))>>4;
    return filtVal; 
}

void dispUpdate()   // Send voltage value to OLED display
{
    display.setTextColor(WHITE); 
    display.clearDisplay();             //clear buffer
    display.setTextSize(2);             // text size
    display.setCursor(12,1);
    display.println("Speed:");        // Potentiometer voltage
    display.println(sensor_1/16);
    display.display();

}

void sendCAN(int inputVal) //Send input value to CAN BUS
{
  // send data:  ID = 0x100, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
        
        data[2] = highByte(inputVal);
        data[3] = lowByte(inputVal);
//        byte sndStat = CAN0.sendMsgBuf(0x100, 0, 8, data);
        byte sndStat = CAN0.sendMsgBuf(0x200, 0, 8, data);
        if(sndStat == CAN_OK){
          Serial.println("Message Sent Successfully!");
        } 
        else {
          Serial.println("Error Sending Message...");
        }
}

void receiveCAN ()  //Recive message from CAN BUS
{
  
    CAN0.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)
//    for (byte i =0; i< len; i++){
//      canMessageData[i] = rxBuf[i];
//      //Serial.println(canMessageData[i]);
//    }
//    if((rxId & 0x80000000) == 0x80000000)     // Determine if ID is standard (11 bits) or extended (29 bits)
//      sprintf(msgString, "Extended ID: 0x%.8lX  DLC: %1d  Data:", (rxId & 0x1FFFFFFF), len);
//    else
//      sprintf(msgString, "Standard ID: 0x%.3lX       DLC: %1d  Data:", rxId, len);
//  
//    Serial.print(msgString);
//  
//    if((rxId & 0x40000000) == 0x40000000){    // Determine if message is a remote request frame.
//      sprintf(msgString, " REMOTE REQUEST FRAME");
//      Serial.print(msgString);
//    } else {
//      for(byte i = 0; i<len; i++){
//        sprintf(msgString, " 0x%.2X", rxBuf[i]);
//        Serial.print(msgString);
//      }
//      // report value of sensor sent across CAN Bus in human readable format
//        float var = (rxBuf[0]<<8) + rxBuf[1];
//        Serial.print("Volts:");
//        Serial.println(var/100);
//    }      
//    Serial.println();
}


void parseCAN( unsigned long id, unsigned long msg)
{
  int var1 = 0;
  
  if (id == 0x200) {  //test 
    Serial.print("Address 0x200 recognized ");
    var1 = (rxBuf[2]<<8) + rxBuf[3];
    Serial.print("Speed:");
    float dispVal = var1/16;
    Serial.println(dispVal);
  }
  else if (id == 0x360){  //Haltech Protocol
    float rpmCAN = (rxBuf[0]<<8) + rxBuf[1];
    Serial.print("RPM:");
    Serial.println(rpmCAN);   // y = x

    float mapCAN = (rxBuf[2]<<8) + rxBuf[3];
    Serial.print("MAP:");
    Serial.println(mapCAN/10);   // y = x/10

    float tpsCAN = (rxBuf[4]<<8) + rxBuf[5];
    Serial.print("TPS:");
    Serial.println(tpsCAN/10);   // y = x/10
  }
  else if (id == 0x361){  //Haltech Protocol
    float fuelPrsCAN = (rxBuf[0]<<8) + rxBuf[1];
    Serial.print("FuelPressure:");
    Serial.println(fuelPrsCAN/10 - 101.3);   // y = x/10 - 101.3

    float oilPrsCAN = (rxBuf[2]<<8) + rxBuf[3];
    Serial.print("Oil Pressure:");   // y = x/10 - 101.3
    Serial.println(oilPrsCAN/10 - 101.3); 
  }
//  else if (id == 0x362){  //Haltech Protocol
//    float injDutyCAN = (canMessageData[0]<<8) + canMessageData[1];
//    Serial.print("Injector DC:");
//    Serial.println(injDutyCAN/10);   // y = x/10
//
//    float ignAngCAN = (canMessageData[4]<<8) + canMessageData[5];
//    Serial.print("Ignition Angle:");   // y = x/10
//    Serial.println(ignAngCAN/10); 
//  }
//  else if (id == 0x368){  //Haltech Protocol
//    float afr1CAN = (canMessageData[0]<<8) + canMessageData[1];
//    Serial.print("AFR:");
//    Serial.println(afr1CAN/1000);   // y = x/1000
//  }
//  else if (id == 0x368){  //Haltech Protocol
//    float knockCAN = (rxBuf[0]<<8) + rxBuf[1];
//    Serial.print("Knock Level:");
//    Serial.println(knockCAN/100);   // y = x/100
//  }
//  else if (id == 0x3E0){  //Haltech Protocol
//    float coolantTempCAN = (rxBuf[0]<<8) + rxBuf[1];
//    Serial.print("Coolant Temp:");
//    Serial.println(coolantTempCAN/10);   // y = x/10
//
//    float airTempCAN = (rxBuf[2]<<8) + rxBuf[3];
//    Serial.print("IAT:");
//    Serial.println(airTempCAN/10);   // y = x/10
//
//    float fuelTempCAN = (rxBuf[4]<<8) + rxBuf[5];
//    Serial.print("Fuel Temp:");
//    Serial.println(fuelTempCAN/10);   // y = x/10
//
//    float oilTempCAN = (rxBuf[6]<<8) + rxBuf[7];
//    Serial.print("Oil Temp:");
//    Serial.println(oilTempCAN/10);   // y = x/10
//  }
//  else if (id == 0x3E1){  //Haltech Protocol
//    float transTempCAN = (rxBuf[0]<<8) + rxBuf[1];
//    Serial.print("Trans Temp:");
//    Serial.println(transTempCAN/10);   // y = x/10
//
//    float fuelCompCAN = (rxBuf[4]<<8) + rxBuf[5];
//    Serial.print("Ethanol %:");
//    Serial.println(fuelCompCAN/10);   // y = x/10
//  }

  
}
