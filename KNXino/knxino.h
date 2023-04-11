/**
 *  MIT License

    Copyright (c) 2021 FABIO Di Michele ((https://www.icomed.it)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 **/
 
#include <Wire.h>

#include "SkElapsedTime.h" 
#include "MedUtils.h"

// Wired connections
#define SEND_BUTTON_PIN   A1    // Pulsante di servizio
#define NO_LED_FEEDBACK_CODE    // Disable feedback led
#define EXT_TV            A2    // Rilevazione di tensione
#define EXT_TA            A3    // Rilevazione di corrente
#define KNX_DATAREADY     2     // Pin 1 Gateway KNX
#define IR_SEND_PIN       3     // Trasmettitore infrarosso
#define RELAY_1           4     // Pin relay 1
#define RELAY_2           5     // Pin relay 2
#define RELAY_3           6     // Pin relay 3
#define RELAY_4           7     // Pin relay 4
#define GET_BUTTON        8     // Pulstante di manovra
#define BUZER_PIN         9     // Segnalazione acustica
#define IR_RECEIVE_PIN    11    // Ricevitore infrarosso
#define KNX_BUS           12    // Stato BUS KNX
#define LED_BUILTIN       13    // Led di test

// Object definition scope
#define OBJ_BUTTON    0
#define OBJ_TERM      1
#define OBJ_VRMS      2
#define OBJ_IRMS      3
#define OBJ_CTEMP     7
#define OBJ_UMIDITY   5
#define OBJ_RELE1     6
#define OBJ_RELE2     8
#define OBJ_RELE3     9
#define OBJ_RELE4     10
#define OBJ_LED       11
#define STAT_RELE1    13
#define STAT_RELE2    14
#define STAT_RELE3    15
#define STAT_RELE4    16

// Calibration
#define TA_CALIBRATION   4.9    // SCT013 5A/1V
#define TV_CALIBRATION   137.03 // TV homemade
#define SERIAL_BIT_RATE  115200 // Velocità della seriale

// Costnati
#define WAIT              10     // Attesa tra tra le letture e scritture su KIM

// Features
#define ENABLE_IRR
#define ENABLE_CLI

//------------------------------------------------------------------------------
// Include the IRremote library header
//
#ifdef ENABLE_IRR
  #include <IRremote.h>
#endif
#ifdef ENABLE_CLI
  #include <SimpleCLI.h>
// Debug flag da commentare per firmware di produzione
//#define   _PLOT
//#define   _DEBUG
//#define   _DEBUG_LIB  
#endif

//+=============================================================================
// Initzialization global objects
//

SkElapsedTimeMillis chronoPulsanteSet;
SkElapsedTimeMillis chronoPulsanteTest;
SkElapsedTimeMillis chronoSensors;
SkElapsedTimeMillis chronoBuzzer;

EdgeValue<bool> isChangedRele1;
EdgeValue<bool> isChangedRele2;
EdgeValue<bool> isChangedRele3;
EdgeValue<bool> isChangedRele4;
EdgeValue<float> isChangedTemp;
EdgeValue<short> isChangedHumi;
EdgeValue<float> isChangedIrms;
EdgeValue<float> isChangedVrms;

TermostatoRegolabile termo;

Toggle testButtonToggle;
Toggle setButtonToggle;

// Create CLI Object
#ifdef ENABLE_CLI
SimpleCLI cli;
Command cmdScan;
Command cmdEnv;
Command cmdShow;
Command cmdHelp;
#endif

// Defalut configuration paramiter
struct config_t
{
  int intervallo = 10000;
  int antiribalso = 250;
  float calibrazione = TA_CALIBRATION;
  float tolleranza = 0.02;
  long serialBitRate = SERIAL_BIT_RATE;
  float setpoint = 0;
  float isteresi = 0;
  float mode = 0;
} configuration;

bool boolRele1 = false;
bool boolRele2 = false;
bool boolRele3 = false;
bool boolRele4 = false;

// Storage for the recorded code
#ifdef ENABLE_IRR
  struct storedIRDataStruct {
    IRData receivedIRData;
    uint8_t rawCode[RAW_BUFFER_LENGTH]; // The durations if raw
    uint8_t rawCodeLength; // The length of the code
  } sStoredIRData;

  void storeCode(IRData *aIRReceivedData);
  void sendCode(storedIRDataStruct *aIRDataToSend);
#endif
#ifdef ENABLE_CLI
  String inputStr = "";
#endif
  
void onBuzzer()
{
  chronoBuzzer.start();
  digitalWrite(BUZER_PIN, HIGH);
}

void checkBuzzer(int ms)
{
  if (chronoBuzzer.stop() >= ms) {
    digitalWrite(BUZER_PIN, LOW);
  }
}

void toneBuzzer(unsigned int duration, unsigned int repite)
{
  for (int i=0; i <= repite; i++) {
    digitalWrite(BUZER_PIN, HIGH);
    delay(duration);
    digitalWrite(BUZER_PIN, LOW);
    delay(duration);
  }
}

#ifdef ENABLE_CLI
void ttySerial() {
  if (Serial.available()) {    
    char c = Serial.read();
    if ((c == '\r') || (c == '\n'))
    {  
      if (inputStr.length() > 0)
      {
  #ifdef _DEBUG
          Serial.print(F("> "));
          Serial.println(inputStr);
  #endif        
        cli.parse(inputStr); 
        inputStr = ""; 
      }      
    } else {
      if (inputStr.length() > 9) {
        toneBuzzer(300,1); 
      } else {
        inputStr += c;
      }
    }
  }
}

static String priorityMode(byte mode)
{
  switch (mode) {
    case 0:           
        return F("ETS");
        break;
    case 1:           
        return F("I2C");
        break;
    case 2:           
        return F("Default");
        break;
  } 
}

static String sino(byte mode)
{
  if (mode == 0) 
    return F("NO");
  else     
    return F("SI");
}

static String printObj(unsigned int id)
{
  switch (id) {
    case OBJ_BUTTON:           
        return F("Button");
        break;
    case OBJ_VRMS:           
        return F("Voltage RMS");
        break;
    case OBJ_IRMS:           
        return F("Current RMS");
        break;
    case OBJ_CTEMP:           
        return F("Tempereature");
        break;
    case OBJ_UMIDITY:           
        return F("Umidity");
        break;
    case OBJ_RELE1:           
        return F("Rele 1");
        break;
    case OBJ_RELE2:           
        return F("Rele 2");
        break;
    case OBJ_RELE3:           
        return F("Rele 3");
        break;
    case OBJ_RELE4:           
        return F("Rele 4");
        break;
    case OBJ_LED:           
        return F("Led");
        break;
    case OBJ_TERM:           
        return F("Termostato");
        break;
  } 
}
#endif
// IR TX e RX con IRemote
void pulsanteSet() {

  // If button pressed, send the code.
  int buttonState = digitalRead(SEND_BUTTON_PIN); // Button pin is active LOW

    /*
     * Check for button just released in order to activate receiving
     */
    if (setButtonToggle.getState() == LOW && buttonState == HIGH) {
        // Re-enable receiver
#ifdef _DEBUG        
        Serial.println(F("Button released"));
#endif
#ifdef ENABLE_IRR
        IrReceiver.start();
#endif  
    }
#ifdef ENABLE_IRR
    // Check for static button state
    if (buttonState == LOW) {
        IrReceiver.stop();
        // Button pressed send stored data or repeat
#ifdef _DEBUG         
        Serial.println(F("Button pressed, now sending"));
#endif
        if (setButtonToggle.getState() == buttonState) {
            sStoredIRData.receivedIRData.flags = IRDATA_FLAGS_IS_REPEAT;
        }
        sendCode(&sStoredIRData);

    // Button is not pressed, check for incoming data
    } else if (IrReceiver.available()) {
        storeCode(IrReceiver.read());
        IrReceiver.resume(); // resume receiver
    }
    setButtonToggle.setState(buttonState);  
#endif 
}

// Stores the code for later playback in sStoredIRData
// Most of this code is just logging
#ifdef ENABLE_IRR
void storeCode(IRData *aIRReceivedData) {
    if (aIRReceivedData->flags & IRDATA_FLAGS_IS_REPEAT) {
  #ifdef _DEBUG       
        Serial.println(F("Ignore repeat"));
  #endif        
        return;
    }
    if (aIRReceivedData->flags & IRDATA_FLAGS_IS_AUTO_REPEAT) {
  #ifdef _DEBUG         
        Serial.println(F("Ignore autorepeat"));
  #endif        
        return;
    }
    if (aIRReceivedData->flags & IRDATA_FLAGS_PARITY_FAILED) {
  #ifdef _DEBUG         
        Serial.println(F("Ignore parity error"));
  #endif        
        return;
    }

    // Copy decoded data
    sStoredIRData.receivedIRData = *aIRReceivedData;

    if (sStoredIRData.receivedIRData.protocol == UNKNOWN) {
  #ifdef _DEBUG 
        Serial.print(F("Received unknown code saving "));
        Serial.print(IrReceiver.decodedIRData.rawDataPtr->rawlen - 1);
        Serial.println(F(" TickCounts as raw "));
  #endif         
        sStoredIRData.rawCodeLength = IrReceiver.decodedIRData.rawDataPtr->rawlen - 1;
        // Store the current raw data in a dedicated array for later usage
        IrReceiver.compensateAndStoreIRResultInArray(sStoredIRData.rawCode);
    } else {
  #ifdef _DEBUG      
        IrReceiver.printIRResultShort(&Serial);
  #endif   
        sStoredIRData.receivedIRData.flags = 0; // clear flags -esp. repeat- for later sending     
    }
}

void sendCode(storedIRDataStruct *aIRDataToSend) {
    if (aIRDataToSend->receivedIRData.protocol == UNKNOWN) {
        // Assume 38 KHz
        IrSender.sendRaw(aIRDataToSend->rawCode, aIRDataToSend->rawCodeLength, 38);
  #ifdef _DEBUG
        Serial.print(F("Sent raw "));
        Serial.print(aIRDataToSend->rawCodeLength);
        Serial.println(F(" marks or spaces"));
  #endif
    } else {
        IrSender.write(&aIRDataToSend->receivedIRData, NO_REPEATS);
  #ifdef _DEBUG_MEGA
        Serial.print(F("Sent: "));
        IrReceiver.printIRResultShort(&Serial); //, &aIRDataToSend->receivedIRData);
  #endif        
    }
}
#endif 
#ifdef ENABLE_CLI
void listI2Cdevices() {
  byte err, address;
  int nDevices;

  //Serial.println(F("Scanning..."));
  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    err = Wire.endTransmission();

    if (err == 0)
    {
      Serial.print(F("I2C device found at address 0x"));
      if (address<16) 
        Serial.print(F("0"));
      Serial.println(address,HEX);

      nDevices++;
    }
    else if (err == 4) 
    {
      Serial.print(F("Unknown error at address 0x"));
      if (address<16) 
        Serial.print(F("0"));
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println(F("No I2C devices found."));
  else
    //Serial.println(F("done."));
  Serial.flush();
}
#endif