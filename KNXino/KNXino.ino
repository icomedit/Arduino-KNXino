/**
 *  MIT License

    Copyright (c) 2021 FABIO Di Michele (https://www.icomed.it)

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
 
#include <KIMlib.h>
#include "knxino.h"

// EmonLibrary examples openenergymonitor.org, Licence GNU GPL V3
#include "EmonLib.h"                   // Include Emon Library

// Sonda di temperatura e umindità
#include <AM232X.h>

AM232X AM2322;
EnergyMonitor emon1;                   // Create an instance

KIMaip knxIno(KNX_DATAREADY, KNX_BUS);
DPT button(OBJ_BUTTON, &knxIno);
DPT termostato(OBJ_TERM, &knxIno);
DPT tensione(OBJ_VRMS, &knxIno);
DPT corrente(OBJ_IRMS, &knxIno);
DPT temperatura(OBJ_CTEMP, &knxIno);
DPT umidita(OBJ_UMIDITY, &knxIno);
DPT rele1(OBJ_RELE1, &knxIno);
DPT rele2(OBJ_RELE2, &knxIno);
DPT rele3(OBJ_RELE3, &knxIno);
DPT rele4(OBJ_RELE4, &knxIno);
DPT led(OBJ_LED, &knxIno);
DPT statRele1(STAT_RELE1, &knxIno);
DPT statRele2(STAT_RELE2, &knxIno);
DPT statRele3(STAT_RELE3, &knxIno);
DPT statRele4(STAT_RELE4, &knxIno);

UserParameter scansioneSensori(&knxIno);  // espresso in secondi da 0 a 255 * 1000
UserParameter antibouning(&knxIno);       // espresso in milli secondi da 0 a 255 * 10
UserParameter setpoint(&knxIno);          // espresso in gradi Centrigradi da 0 a 255
UserParameter isteresi(&knxIno);          // espresso in gradi Centrigradi da 0 a 255
UserParameter marcia(&knxIno);            // espresso in si o no da 0 e 1
UserParameter calibrazioneTA(&knxIno);    // espresso in decimi da 0 a 255 / 10
UserParameter tolleranzaTA(&knxIno);      // espresso in percentuale da 0 a 100 / 100
UserParameter kBitRate(&knxIno);          // espresso in migliaglia da 0 a 255 * 1000
UserParameter cBitRate(&knxIno);          // espresso in centinaia da 0 a 255

void setup() {

  // leggi i parametri utente impostate da ETS
  allingeUserParameter();

  // Chiedo lo stato all'oggetto
  allingeReleeStatus();
   
  // initialize the LED pin as an output:
  pinMode(BUZER_PIN, OUTPUT);           // Buzzer
  pinMode(LED_BUILTIN, OUTPUT);         // Led
  digitalWrite(LED_BUILTIN, LOW);

  // initialize the pushbutton pin as an input:
  pinMode(GET_BUTTON, INPUT_PULLUP);    // Pulsante di test
  pinMode(SEND_BUTTON_PIN, INPUT);      // Pulsante di set

  // Relay shild test
  pinMode(RELAY_1, OUTPUT);
  pinMode(RELAY_2, OUTPUT);
  pinMode(RELAY_3, OUTPUT);
  pinMode(RELAY_4, OUTPUT);

#ifdef _DEBUG
  delay(WAIT);
  printSystemParameter();
  printUserParameter();
#endif

  //IrReceiver.begin(IR_RECEIVE_PIN);  // Start the receiver
  //IrSender.begin(IR_SEND_PIN);

  if (! AM2322.begin() )
  {
    toneBuzzer(500, 3);
#ifdef _DEBUG
    Serial.println(F("Sensor HT not found"));
#endif
  }

  cmdScan = cli.addCmd("lsi2c");
  cmdScan.setDescription("Scanning available I2C devices.");
  cmdEnv = cli.addCmd("env");
  cmdEnv.setDescription("List of system parameters");
  cmdShow = cli.addCmd("show");
  cmdShow.setDescription("List of user parameters");
  cmdHelp = cli.addCommand("help");

#ifdef _PLOT
  Serial.println(F("\rIrms\tVrms\tHUMI\tTEMP"));
#endif

  chronoPulsanteTest.start();
  chronoPulsanteSet.start();
  onBuzzer();
}

void loop() {

  // play a note for 500 ms:
  checkBuzzer(500);

  // Input command via serial monitor
  ttySerial();
  ttySelectCmd();
  
  // put your main code here, to run repeatedly:
  if (knxIno.recive()) {

#ifdef _DEBUG
    systemEvent(knxIno.getSystemEvent());
    delay(WAIT);
#endif
    bool stat = false;
    led.getValue(stat);
    digitalWrite(LED_BUILTIN, stat);
    writeRele();
  }

  if ((chronoSensors.stop() >= configuration.intervallo) && (configuration.intervallo > 0)) { // ogni minuto ?    
    readPowerMeter();
    readDHT();
#ifdef _PLOT
    Serial.println();
#endif
    chronoSensors.start();
  }

  // gestioni pulsanti KNXino
  if ((chronoPulsanteTest.stop() >= configuration.antiribalso) && (configuration.antiribalso > 0)) {
    pulsanteTest();
    chronoPulsanteTest.start();
  }

  if ((chronoPulsanteSet.stop() >= configuration.antiribalso) && (configuration.antiribalso > 0)) {
    pulsanteSet();
    chronoPulsanteSet.start();
  }
}

// Sub-Routine  ************************************************************************************************************************************************************************************************

void pulsanteTest() {
  bool stat = false;
  // read the state of the pushbutton value:
  if (digitalRead(GET_BUTTON) == LOW)
  {
    stat = testButtonToggle.changeStatus();
    button.setValue(stat);
#ifdef _DEBUG
    Serial.print(F("Toggle value: "));
    Serial.println(testButtonToggle.getState());
#endif
  }
}

void readDHT()
{
  // Read temperature as Celsius
  int status = AM2322.read();

  // DISPLAY DATA, sensor only returns one decimal.
  short h = AM2322.getHumidity();
  float t = AM2322.getTemperature();

#ifdef _PLOT
  if (status == 0) {
    Serial.print(h, 1);
    Serial.print(F("\t"));
    Serial.print(t, 1);
  } else {
    Serial.print(F("ERROR\tERROR"));
  }
#endif

  if (status == 0) {

    // invia dato al bus KNX
    isChangedTemp.setValue(t);
    if (isChangedTemp.isChanged()) {
#ifdef _DEBUG
      Serial.print(F("Acceso: "));
      Serial.println(termo.getState(t));
#endif
      //termostato.setValue(termo.getState(t));
      temperatura.setValue(t);
    }

    isChangedHumi.setValue(h);
    if (isChangedHumi.isChanged())
      umidita.setValue(h);
  }
}

void readPowerMeter()
{
  float Irms = emon1.calcIrms(1480);  // Calculate Irms only

  emon1.calcVI(2,50);         // Calculate all. No.of half wavelengths (crossings), time-out
  float Vrms = emon1.Vrms;
  
#ifdef _PLOT
  Serial.print(Irms, 4);         // Apparent power
  Serial.print(F("\t"));
  Serial.print(Vrms);
  Serial.print(F("\t"));
#endif
  
  isChangedIrms.setValue(Irms);
  if (isChangedIrms.isTolleranceChanged(configuration.tolleranza)) // errore del 5% sulla lettura
    corrente.setValue(Irms);

  isChangedVrms.setValue(Vrms);
  if (isChangedVrms.isTolleranceChanged(configuration.tolleranza))
    tensione.setValue(Vrms);
}

void printSystemParameter() {
  //do something when var equals 0001h
  Serial.print(F("Version: "));
  Serial.print(knxIno.getVersion());
  Serial.println();
  delay(WAIT);
  //do something when var equals 0002h
  Serial.print(F("KNX Address: "));
  //Serial.print(knxIno.getKNXaddress(),BIN);
  word address = knxIno.getKNXaddress();
  byte digit[3];

  digit[2] = (byte) ((address & 0xFF00) >> 8);
  digit[1] = (byte) ((address & 0x00F0) >> 4);
  digit[0] = (byte) ((address & 0x000F));
  for (int i = 0; i < 3; i++) {
    if (i > 0) Serial.print(F("/"));
    Serial.print(digit[i], DEC);
  }
  Serial.println();
  delay(WAIT);
  //do something when var equals 0003h
  Serial.print(F("Programming Mode: "));
  Serial.print(priorityMode(knxIno.getProgMode()));
  Serial.println();
  delay(WAIT);
  //do something when var equals 0005h
  Serial.print(F("I2C Address: ? "));
  Serial.print(priorityMode(knxIno.getI2Caddress()));
  Serial.println();
  delay(WAIT);
  //do something when var equals 0006h
  Serial.print(F("Actual I2C Address: 0x"));
  Serial.print(knxIno.getI2CacutalAddress(), HEX);
  Serial.println();
  delay(WAIT);
  //do something when var equals 0011h
  Serial.print(F("Bit to Enable System-Event: "));
  Serial.print(knxIno.getSYSevent(), BIN);
  Serial.println();
  delay(WAIT);
  //do something when var equals 0012h
  Serial.print(F("Enable Error-Message: "));
  Serial.print(sino(knxIno.getEnableError()));
  Serial.println();
  delay(WAIT);
  //do something when var equals 0020h
  Serial.print(F("TimeOut I2C message: "));
  Serial.print(knxIno.getI2Ctimeout());
  Serial.print(F(" ms"));
  Serial.println();
  delay(WAIT);
  //do something when var equals 0021h
  Serial.print(F("TimeOut indication: "));
  Serial.print(knxIno.getIndTimeout());
  Serial.print(F(" ms"));
  Serial.println();
  delay(WAIT);
  //do something when var equals 0022h
  Serial.print(F("TimeOut indication no Master: "));
  Serial.print(knxIno.getMasterTimeout());
  Serial.print(F(" ms"));
  Serial.println();
  delay(WAIT);
  //do something when var equals 0023h
  Serial.print(F("Retry sending: "));
  Serial.print(knxIno.getRtySend());
  Serial.println();
  delay(WAIT);
  //do something when var equals 0100h
  Serial.print(F("Setting for TrasparentMode: "));
  Serial.print(knxIno.getTransMode(), BIN);
  Serial.println();
  delay(WAIT);
  //do something when var equals 0110h
  Serial.print(F("GroupLink Disabled: "));
  Serial.print(sino(knxIno.getGroupLink()));
  Serial.println();
  delay(WAIT);
  //do something when var equals 0200h
  Serial.print(F("State Device: "));
  if (knxIno.getDevStatus() == KIM_READY) {
    Serial.print(F("Normal mode"));    
  } else {
    Serial.print(knxIno.getDevStatus(), BIN);
  }
  Serial.println();
  delay(WAIT);
  Serial.flush();
}

void systemEvent(unsigned long sysEvent)
{
  union l_tag {                             // Siccome la temperatura è un dato FLOAT, si usa la funzione Union per "spacchettare" i 4 BYTE che la compongono.
    unsigned long temp_long ;               // Se copi dentro "u.temp_float" un valore float automaticamente ottieni i relativi quattro byte nel array "u.temp_byte[n]", con n compreso tra 0 e 3,
    unsigned int temp_int[2] ;              // Viceversa se copy nel array i quattro byte, ottieni il tuo valore float dentro u.temp_float.
  } l;
  l.temp_long = sysEvent;
  unsigned int idEvent = l.temp_int[1];
  unsigned int objN = l.temp_int[0];

#ifdef _DEBUG_LIB
  Serial.print(F("Sytem Event: 0x"));
  Serial.println(l.temp_long, HEX);
#endif
#ifdef _DEBUG
  Serial.print(printObj(objN));
  Serial.println(printIdEvent(idEvent));
#endif
}

void ttySelectCmd() {
  // Interpetre dei comandi
  if (cli.available()) {
    Command c = cli.getCmd();
    //int argNum = c.countArgs();
    if (c == cmdScan) {
      //Serial.println(F("Scanning available I2C devices..."));
      listI2Cdevices();
    } else if (c == cmdEnv) {
      printSystemParameter();
    } else if (c == cmdShow) {
      printUserParameter();
    } else if (c == cmdHelp) {
      Serial.println(F("List of console commands:"));
      Serial.println(cli.toString());
    }
  }
}

void printUserParameter()
{
  Serial.print(F("Serial bit rate: "));
  Serial.print(configuration.serialBitRate);
  Serial.println(F(" bps"));
  Serial.print(F("Rapporto TA: "));
  Serial.println(configuration.calibrazione);
  Serial.print(F("Tolleranza TA: "));
  Serial.print(configuration.tolleranza * 100, 0);
  Serial.println(F(" %"));
  Serial.print(F("Intervallo Scansione Sensori: "));
  Serial.print(configuration.intervallo / 1000);
  Serial.println(F(" sec."));
  Serial.print(F("Intervallo Antirimbalso Pulsanti: "));
  Serial.print(configuration.antiribalso);
  Serial.println(F(" ms"));
  if (termo.getMode())
    Serial.print(F("INVERNO"));
  else
    Serial.print(F("ESTATE"));
  Serial.println();
  Serial.print(F("\tSetpoint: "));
  Serial.print(configuration.setpoint, 0);
  Serial.println(F(" *C"));
  Serial.print(F("\tDelta: "));
  Serial.print(configuration.isteresi, 1);
  Serial.println(F(" *C"));
}

void allingeUserParameter()
{
  configuration.serialBitRate = ((long)kBitRate.getValue() * 1000) + (long)cBitRate.getValue();
  delay(WAIT);
  // Inizializza Seriale
#ifdef _DEBUG
  Serial.begin(SERIAL_BIT_RATE);
  delay(WAIT);
  Serial.println(F("\n\n\nDEBUG MODE\n"));
#else
  Serial.begin(configuration.serialBitRate);
#endif
  delay(WAIT);
  //configuration.calibrazione = (float)calibrazioneTA.getValue() / 10;
  emon1.current(EXT_TA, configuration.calibrazione); // Current: input pin, calibration.

  emon1.voltage(EXT_TV, TV_CALIBRATION, 1);  // Voltage: input pin, calibration, phase_shift
  delay(WAIT);
  configuration.tolleranza = (float)tolleranzaTA.getValue() / 100;
  delay(WAIT);
  configuration.intervallo = (int)scansioneSensori.getValue() * 1000;
  //if (configuration.intervallo == 0)
  //  configuration.intervallo = 1000;
  delay(WAIT);
  configuration.antiribalso = (int)antibouning.getValue() * 10;
  //if (configuration.antiribalso == 0)
  //  configuration.antiribalso = 250;
  delay(WAIT);
  configuration.setpoint = (float)setpoint.getValue();
  termo.setPoint(configuration.setpoint);
  delay(WAIT);
  configuration.isteresi = (float)isteresi.getValue() / 10;
  termo.setDelta(configuration.isteresi);
  delay(WAIT);
  termo.setMode(marcia.getValue());
  configuration.mode = termo.getMode();
  delay(WAIT);
}

void allingeReleeStatus() 
{
  //bool res = false;
  statRele1.getStatusValue();
  //rele1.setStatusValue(res);
  //digitalWrite(RELAY_1, res);
  delay(WAIT);
  statRele2.getStatusValue();
  //rele2.setStatusValue(res);
  //digitalWrite(RELAY_2, res);
  delay(WAIT);
  statRele3.getStatusValue();
  //rele3.setStatusValue(res);
  //digitalWrite(RELAY_3, res);
  delay(WAIT);
  statRele4.getStatusValue();
  //rele4.setStatusValue(res);
  //digitalWrite(RELAY_4, res);
  delay(WAIT);
}

void writeRele()
{
  rele1.getValue(boolRele1);
  isChangedRele1.setValue(boolRele1);
  if (isChangedRele1.isChanged()) {
    digitalWrite(RELAY_1, isChangedRele1.getValue());
    boolRele1 = isChangedRele1.getValue();
    statRele1.setValue(boolRele1);
  }
  rele2.getValue(boolRele2);
  isChangedRele2.setValue(boolRele2);
  if (isChangedRele2.isChanged()) {
    digitalWrite(RELAY_2, isChangedRele2.getValue());
    boolRele2 = isChangedRele2.getValue();
    statRele2.setValue(boolRele2);
  }
  rele3.getValue(boolRele3);
  isChangedRele3.setValue(boolRele3);
  if (isChangedRele3.isChanged()) {
    digitalWrite(RELAY_3, isChangedRele3.getValue());
    boolRele3 = isChangedRele3.getValue();
    statRele3.setValue(boolRele3);
  }
  rele4.getValue(boolRele4);
  isChangedRele4.setValue(boolRele4);
  if (isChangedRele4.isChanged()) {
    digitalWrite(RELAY_4, isChangedRele4.getValue());
    boolRele4 = isChangedRele4.getValue();
    statRele4.setValue(boolRele4);
  }
}
