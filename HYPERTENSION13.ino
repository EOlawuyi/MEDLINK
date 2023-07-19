     /* 218617611 
      *  
      *  //for physician programming, check and correct spo2 and body temperature
       * Full Arduino code for MEDLINK Product 
       * 
       * This code allows the device to interact with the physician using the text-to-speech synthesizer
       * It obtains the RPM requirements the physician desires for a specific patient
       * The current RPM Physiological Parameters available are:
       *  1. Heart Rate
       *  2. Pulse
       *  3. SPO2
       *  4. Body Temperature 
       *  5  Blood Sugar
       *  6. ----------
       *  7. ECG (498 readings in 10 seconds. Each reading is taken within 20ms.)
       *  
       *   
       * The Physician's data and his or her selections are stored in the SD card afterwards.
       * This data is retrieved during the Patient Reporting Mode and used to measure the physician-selected parameters.
       * Currently, this code is able to automatically measure: 
       * 1. Body Temperature 
       * 2. Blood Sugar
       * 3. ECG
       * 4. 
       * 5. 
       * 
       * By DR. MICHAEL OLAWUYI AND DR. ESTHER OLAWUYI
       * DR. JASON M. ZARA, DR. VESNA ZDERIC, DR. MURRAY LOEW, DR. AHMED JENDOUBI, DR. GARY HARRIS,
       * MR. JAMES GRIFFIN, DR. MOHAMED CHOUIKHA, DR. SHANI ROSS, ENGINEER FRANCIS OLAWUYI, ENGINEER JOSHUA OLAWUYI, ENGINEER DEBORAH OLAWUYI,
       * ENGINEER JOSEPH OLAWUYI, DR. MATTHEW OLAWUYI, DR. MICHAEL OLAWUYI, AND HONORABLE DAMILOLA SUNDAY OLAWUYI.
       * tegae@gwmail.gwu.edu
       * OLAWUYI RACETT NIGERIA LTD., KEMP HOUSE, 160 CITY ROAD, EC1V 2NX, LONDON, UNITED KINGDOM
       * https://www.olawuyiracettnigerialtd.com
       * July 19, 2023.
       *   
        
       */
       

// include the SoftwareSerial library so we can use it to talk to the Emic 2 module
// include the SoftwareSerial library so we can use it to talk to the Emic 2 module
#include <Arduino.h>
#if defined(ESP32)
  #include <WiFi.h>
#elif defined(ESP8266)
  #include <ESP8266WiFi.h>
#endif
#include <ESP_Mail_Client.h>
//#include <SoftwareSerial.h>
#include "SoftwareSerial.h"
#include <SoftwareSerial.h>
#include <SD.h>
#include <SdFat.h>
#include <Wire.h>
#include "MAX30100_PulseOximeter.h"
#include <DallasTemperature.h>
#include "MAX30105.h"
#include <INA219_WE.h>
#include <OneWire.h>
#include <Keypad.h>
#include "heartRate.h"
#include "spo2_algorithm.h"
#include <SparkFunMLX90614.h> //Click here to get the library: http://librarymanager/All#Qwiic_IR_Thermometer by SparkFun

MAX30105 particleSensor;
#define REPORTING_PERIOD_MS     1000
#define MAX_BRIGHTNESS 255
String Message;

IRTherm therm; // Create an IRTherm object to interact with throughout


#define DEBUG true
String data[5];
String response;


//Declare Variables
const byte ROWS1 = 5; // five rows
const byte COLS1 = 4; // four columns

char keys1 [ROWS1][COLS1] = {
  {'A','B','C','D'},
  {'I','J','K','L'},
  {'Q','R','S','T'},
  {'Y','Z','1','2'},
  {'7','8','9','0'},
};
const byte ROWS2 = 5; // five rows
const byte COLS2 = 4; // four columns
char keys2 [ROWS2][COLS2] = {
  {'E','F','G','H'},
  {'M','N','O','P'},
  {'U','V','W','X'},
  {'3','4','5','6'},
  {'@','.',' ','#'},
};

// Keypad Connections: from '1' to '9'
// Keypad 1: connect 1, 2, 3, 4,, 5, 6, 7, 8 & 9, to  arduino pins 45, 43, 41, 39, 37, 35, 33, 31, & 29 respectively. // 47 46, 45, 44, 43, 42, 41, 40, 39
// Keypad 2: connect 1, 2, 3, 4, 5, 6, 7, 8 & 9, to  arduino pins 44, 42, 40, 38, 36, 34, 32, 30, & 9 respectively. //  17, 16, 15, 14, 2, 3, 4, 5, 6


byte rowPins1[ROWS1] = {41, 42, 43, 44, 45}; // connect to pins 9, 8, 7, 6, 5 pinouts of the keypad (row pinouts)
byte colPins1[COLS1] = {49, 48, 47, 46}; // connect to pins 1, 2, 3, 4 pinouts of the keypad     (column pinouts)
byte rowPins2[ROWS2] = {7, 6, 5, 4, 3}; // connect to pins 9,8,7,6,5 pinouts of the keypad (row pinouts)
byte colPins2[COLS2] = {19, 18, 17, 2}; // connect to pins 1, 2, 3, 4 pinouts of the keypad (column pinouts)


//initlaize and create the 2 key pads
Keypad keypad = Keypad( makeKeymap(keys1), rowPins1, colPins1, ROWS1, COLS1);
Keypad kpd = Keypad( makeKeymap(keys2), rowPins2, colPins2, ROWS2, COLS2);

#define I2C_ADDRESS 0x40

//Declare Variables
INA219_WE ina219(I2C_ADDRESS);
const int chipSelect = 53;
float mytemp = 0;
String kinput;
String tester[8];
String phyname, phymail, patname, patid, physsid, phyword, patssid, patpword, stpara;
int mode;
int phyparameters[4];

int sensorPin = A0;    // select the rotation knob sensor
int sensorValue = 0;  // variable to store the value coming from the sensor
//A2 is the analog pin used by IR glucometer
//A1 is the analog pin  used by ECG module
//ECG Lo+ pin is connected to Digital pin 15
//ECG Lo- is connected to Digital pin 16

#define rxPin 11    // Serial input (connects to Emic 2 SOUT)
#define txPin 10    // Serial output (connects to Emic 2 SIN)

// set up a new serial port
SoftwareSerial emicSerial =  SoftwareSerial(rxPin, txPin);
SoftwareSerial esp32(8,9);


File myFile;


float val = 0;
float spirometer = 0;

uint32_t tsLastReport = 0;
unsigned long startMillis;
unsigned long currentMillis;
const unsigned long period = 1000;


//A1 is the analog pin  used by ECG module
//ECG Lo+ Pin used Digital Pin 15
//ECG Lo- Pin uses Digital Pin 16
int ecgvalue = 0;
String ecg_data = "Volts[V]: ";


String pinput = "";
String kpresponse = "";

// for Conirmtory RPM TEXT
  String one = "";
  String two = "";
  String three = "";
  String four = "";
  String five = "";
  String six = "";
  String seven = "";
  String eight = "";


float sp;
float spo2val, spo2val2;
float hrate, hrate2;
PulseOximeter pox;
int32_t spo2 = 0; //SPO2 value
float FEV1, FVC, FEV1_FVC_ratio;
int line_count = 0;
int bcount = 0;         
int ifRead;
int bloodglucose[5];
float avbg = 0;
float mmolL = 0;


void phyinput();
void body_temperature();
void patient_reporting();
void physician_programming();
void printvalue(float a);
void printecg(String a);
void ecg();
void keyinput();
void call(void);
void ReportRPM(String a, String patid, String phyphone, int b, int c, int d, int e, int f, int g, float hrate, float pul, float sp2, String ecg_dat, float mmol, float btemperature);
void ConfirmRPM(String a, String phyname, String phyphone, String phymail, String patname, String patid, int b, int c, int d, int e, int f, int g);
void onBeatDetected()
{
    Serial.println("Beat!");
}

void setup()  // Set up code called once on start-up
{
  // define pin modes
  // pinMode(en,OUTPUT);
   //pinMode(en2,OUTPUT);
   //digitalWrite(en2, LOW);

  pinMode(A0, INPUT);
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  pinMode(A14, INPUT);  //input for Spirometer
   
  pinMode(A15,INPUT);  //input for ECG
  pinMode(15, INPUT); // Setup for leads off detection ECG LO +
  pinMode(16, INPUT); // Setup for leads off detection ECG LO -
 

  pinMode(chipSelect, OUTPUT);
  pinMode(A2, INPUT);
  pinMode(15, INPUT); // Setup for leads off detection ECG LO +
  pinMode(16, INPUT); // Setup for leads off detection ECG LO -

 
  
    Serial.begin(9600);

    Serial.print("Initializing pulse oximeter..");

    // Initialize the PulseOximeter instance
    // Failures are generally due to an improper I2C wiring, missing power supply
    // or wrong target chip
    if (!pox.begin()) {
        Serial.println("FAILED");
       // for(;;);
    } else {
        Serial.println("SUCCESS");
    }

    // The default current for the IR LED is 50mA and it could be changed
    //   by uncommenting the following line. Check MAX30100_Registers.h for all the
    //   available options.
    // pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);

    // Register a callback for the beat detection
   pox.setOnBeatDetectedCallback(onBeatDetected);

  // set the data rate for the SoftwareSerial port
  emicSerial.begin(9600);
   
  /*
    When the Emic 2 powers on, it takes about 3 seconds for it to successfully
    intialize. It then sends a ":" character to indicate it's ready to accept
    commands. If the Emic 2 is already initialized, a CR will also cause it
    to send a ":"
  */
  emicSerial.print('\n');             // Send a CR in case the system is already up
  while (emicSerial.read() != ':');   // When the Emic 2 has initialized and is ready, it will send a single ':' character, so wait here until we receive it
  delay(10);                          // Short delay
  emicSerial.flush();                 // Flush the receive buffer
  Serial.println("Done 2!");


}

//void(*resetFunc)(void)=0;
void loop()  // Main code, to run repeatedly
{ 
  
  emicSerial.print('N');
  emicSerial.print(1);
  emicSerial.print('\n');
  emicSerial.print('V');
  emicSerial.print(18);
  emicSerial.print('\n');
  
  delay(2000);
 
  //Check if the unit is being used by a physician or by a patient
  Serial.println("Selected Operation Mode:");
  sensorValue = analogRead(sensorPin);
  Serial.println(sensorValue);
  delay(1000);
    sensorValue = analogRead(sensorPin);
  Serial.println(sensorValue);
  delay(1000);
  if(sensorValue > 10)
   {
       Serial.println("MEDLINK is in PHYSICIAN PROGRAMMING MODE");
       delay(2000);
       physician_programming();
   }
  else if(sensorValue<11)
   {
      Serial.println("MEDLINK is in PATIENT REPORTING MODE");
      delay(2000);
      patient_reporting();
   }

 
}
void physician_programming()
{
   // Welcome
 
  emicSerial.print('S');
  emicSerial.print("Welcome Doctor. This is the Physician Programming Mode. Please type in your name and press the ENTER KEY");  // Send the desired string to convert to speech
  emicSerial.print('\n');
  emicSerial.flush();                 // Flush the receive buffer
  //while (emicSerial.read() != ':'); 
 
  keyinput();
  Serial.print("This is your input: ");
  Serial.println(kinput);
  phyname = kinput;

 
  emicSerial.print('S');
  emicSerial.print("Thank you. Doctor");  // Send the desired string to convert to speech
  emicSerial.print('\n');
  emicSerial.flush();
  delay(3000);

  emicSerial.print('S');
  emicSerial.print("Please type in your e-mail address and press the ENTER Key");  // Send the desired string to convert to speech
  emicSerial.print('\n');
  emicSerial.flush();
 
  keyinput();
  Serial.print("This is your input: ");
  Serial.println(kinput);
  phymail = kinput;

 
  emicSerial.print('S');
  emicSerial.print("Thank you");  // Send the desired string to convert to speech
  emicSerial.print('\n');
  emicSerial.flush();
  delay(3000);

 
  emicSerial.print('S');
  emicSerial.print("Please type in the patient's name and then press the ENTER Key");  // Send the desired string to convert to speech
  emicSerial.print('\n');
  emicSerial.flush();


  keyinput();
  Serial.print("This is your input: ");
  Serial.println(kinput);
  patname = kinput;

  
  emicSerial.print('S');
  emicSerial.print("Thank you");  // Send the desired string to convert to speech
  emicSerial.print('\n');
  emicSerial.flush();
  delay(3000);

  emicSerial.print('S');
  emicSerial.print("Please type in the patient's Identification number and press the ENTER Key");  // Send the desired string to convert to speech
  emicSerial.print('\n');
  emicSerial.flush();
  
  keyinput();
  Serial.print("This is your input: ");
  Serial.println(kinput);
  patid = kinput;

 
  emicSerial.print('S');
  emicSerial.print("Thank you");  // Send the desired string to convert to speech
  emicSerial.print('\n');
  emicSerial.flush();
  delay(3000);

 
 
  //PARAMETER 1
  
  emicSerial.print('S');
  emicSerial.print("Would you like this patient to report Heart Rate? Please type YES or NO");
  emicSerial.print('\n');
  emicSerial.flush();
  delay(3000);

 
  keyinput();
  Serial.print("This is your input: ");
  Serial.println(kinput);
   
  while((kinput != "YES") && (kinput != "NO"))
   {
      emicSerial.print('S');
      emicSerial.print("Incorrect response. Should this patient report Heart Rate?");
      emicSerial.print('\n');
      emicSerial.flush();
      keyinput();
      Serial.print("This is your input: ");
      Serial.println(kinput);
   }
 
  if(kinput == "YES")
  {
    phyparameters[0] = 1;
    one = "Heart Rate ";
        Serial.print("Parameter 0: ");
  Serial.println(phyparameters[0]);
  Serial.print("One: ");
  Serial.println(one);
  }
  else if(kinput == "NO")
  {
    phyparameters[0] = 0;
        Serial.print("Parameter 0: ");
  Serial.println(phyparameters[0]);
  }
  
  emicSerial.print('S');
  emicSerial.print("Thank you");  // Send the desired string to convert to speech
  emicSerial.print('\n');
  emicSerial.flush();
  delay(3000);
  
 
  //PARAMETER 2
  
  emicSerial.print('S');
  emicSerial.print("Would you like this patient to report Respiration? Please type YES or NO");
  emicSerial.print('\n');
  emicSerial.flush();
  delay(3000);

  
  keyinput();
  Serial.print("This is your input: ");
  Serial.println(kinput);

  
  while((kinput != "YES") && (kinput != "NO"))
   {
      emicSerial.print('S');
      emicSerial.print("Incorrect response. Should this patient report Respiration?");
      emicSerial.print('\n');
      emicSerial.flush();
      keyinput();
      Serial.print("This is your input: ");
      Serial.println(kinput);
   }
  
  if(kinput == "YES")
  {
      phyparameters[1] = 1;
      two = "Respiration"; 
      Serial.print("Parameter 1: ");
      Serial.println(phyparameters[1]);
      Serial.print("Two: ");
      Serial.println(two);
  }
  else if(kinput == "NO")
  {
    phyparameters[1] = 0;
        Serial.print("Parameter 1: ");
  Serial.println(phyparameters[1]);
  }
  
  emicSerial.print('S');
  emicSerial.print("Thank you");  // Send the desired string to convert to speech
  emicSerial.print('\n');
  emicSerial.flush();
  delay(3000);
  


  //PARAMETER 3
  
  emicSerial.print('S');
  emicSerial.print("Would you like this patient to report S P O 2? Please type YES or NO");
  emicSerial.print('\n');
  emicSerial.flush();
  delay(3000);
  

  keyinput();
  Serial.print("This is your input: ");
  Serial.println(kinput);

  
  while((kinput != "YES") && (kinput != "NO"))
   {
      emicSerial.print('S');
      emicSerial.print("Incorrect response. Should this patient report S P O 2?");
      emicSerial.print('\n');
      emicSerial.flush();
      keyinput();
      Serial.print("This is your input: ");
      Serial.println(kinput);
   }

  if(kinput == "YES")
  {
    phyparameters[2] = 1;
    three = "SPO2";
        Serial.print("Parameter 2: ");
  Serial.println(phyparameters[2]);
  }
  else if(kinput == "NO")
  {
    phyparameters[2] = 0;
        Serial.print("Parameter 2: ");
  Serial.println(phyparameters[2]);
  }

 
  emicSerial.print('S');
  emicSerial.print("Thank you");  // Send the desired string to convert to speech
  emicSerial.print('\n');
  emicSerial.flush();
  delay(3000);
  

//PARAMETER 4
  
  emicSerial.print('S');
  emicSerial.print("Would you like this patient to report E C G? Please type YES or NO");
  emicSerial.print('\n');
  emicSerial.flush();
  delay(3000);

  
  keyinput();
  Serial.print("This is your input: ");
  Serial.println(kinput);

  
  while((kinput != "YES") && (kinput != "NO"))
   {
      emicSerial.print('S');
      emicSerial.print("Incorrect response. Should this patient report E C G?");
      emicSerial.print('\n');
      emicSerial.flush();
      keyinput();
      Serial.print("This is your input: ");
      Serial.println(kinput);
   }
  
  if(kinput == "YES")
  {
    phyparameters[3] = 1;
     four = "ECG";
        Serial.print("Parameter 3: ");
  Serial.println(phyparameters[3]);
  }
  else if(kinput == "NO")
  {
    phyparameters[3] = 0;
        Serial.print("Parameter 3: ");
  Serial.println(phyparameters[3]);
  }
  
  emicSerial.print('S');
  emicSerial.print("Thank you");  // Send the desired string to convert to speech
  emicSerial.print('\n');
  emicSerial.flush();
  delay(3000);
   
  emicSerial.print('S');
  emicSerial.print("Please type in your Wi Fi user name and press the ENTER Key");  // Send the desired string to convert to speech
  emicSerial.print('\n');
  emicSerial.flush();
 
  keyinput();
  Serial.print("This is your input: ");
  Serial.println(kinput);
  physsid = kinput;

 
  emicSerial.print('S');
  emicSerial.print("Thank you");  // Send the desired string to convert to speech
  emicSerial.print('\n');
  emicSerial.flush();
  delay(3000);
  
  emicSerial.print('S');
  emicSerial.print("Please type in your Wi Fi password and press the ENTER Key");  // Send the desired string to convert to speech
  emicSerial.print('\n');
  emicSerial.flush();
 
  keyinput();
  Serial.print("This is your input: ");
  Serial.println(kinput);
  phyword = kinput;

 

  emicSerial.print('S');
  emicSerial.print("Thank you. Please Wait while I send you a confirmation e-mail ");  // Send the desired string to convert to speech
  emicSerial.print('\n');
  delay(3000);
  esp32.println("1");
  delay(1000);
  Serial.println("ABC");
  delay(1000);
  esp32.println(phymail);
  delay(1000);
  Serial.println("ABC");
  delay(1000);
  esp32.println(phyname);
  delay(1000);
  Serial.println("ABC");
  delay(1000);
  esp32.println(patname);
  delay(1000);
  Serial.println("ABC");
  delay(1000);
  esp32.println(patid);
  delay(1000);
  Serial.println("ABC");
  delay(1000);
  esp32.println(phyparameters[0]);
  delay(1000);
  Serial.println("ABC");
  delay(1000);
  esp32.println(phyparameters[1]);
  delay(1000);
  Serial.println("ABC");
  delay(1000);
  esp32.println(phyparameters[2]);
  delay(1000);
  Serial.println("ABC");
  delay(1000);
  esp32.println(phyparameters[3]);
  delay(1000);
  Serial.println("ABC");
  delay(1000);
  esp32.println(physsid);
  delay(1000);
  Serial.println("ABC");
  delay(1000);
  esp32.println(phyword);
  delay(1000);
  Serial.println("ABC FINAL");
  delay(1000);
  delay(8000);
  
  // send confirmation text message
  
     
  Serial.println("FINISHED!");
  Serial.print("Parameter 0: ");
  Serial.println(phyparameters[0]);
    Serial.print("Parameter 1: ");
  Serial.println(phyparameters[1]);
    Serial.print("Parameter 2: ");
  Serial.println(phyparameters[2]);
    Serial.print("Parameter 3: ");
  Serial.println(phyparameters[3]);
  delay(1000);

File myFile;

if (!SD.begin(53)) {
    Serial.println("initialization failed!");
    while (1);
  }
   
  SD.remove("test.txt");
  SD.rmdir("test.txt");


if(!SD.remove("test.txt")) { Serial.println("Cannot Delete File");}
  
  myFile = SD.open("test.txt", FILE_WRITE);
  if (myFile) 
  {
    Serial.print("Writing to test.txt...");
  myFile.println(phyname);
  myFile.println(phymail);
  myFile.println(patname);
  myFile.println(patid);
  myFile.println(phyparameters[0]);
  myFile.println(phyparameters[1]);
  myFile.println(phyparameters[2]);
  myFile.println(phyparameters[3]);
  myFile.println();
  // close the file:
  myFile.close();
  Serial.println("done.");
  }

  myFile = SD.open("test.txt", FILE_READ);
  if (myFile) 
  {
   
  Serial.println("READING test.txt:");

  // read from the file until there's nothing else in it:
  int data;
  while ((data = myFile.read()) >= 0) Serial.write(data);
  // close the file:
  myFile.close();
  delay(3000); 
  
  }


  
/*
  // re-open the file for reading:
  if (!myFile.open("test.txt", O_READ)) {
    sd.errorHalt("opening test.txt for read failed");
  }
  Serial.println("test.txt:");

  // read from the file until there's nothing else in it:
  int data;
  while ((data = myFile.read()) >= 0) Serial.write(data);
  // close the file:
  myFile.close();
  delay(3000); */

  
   //delay(2000);
  emicSerial.print('S');
  emicSerial.print("Thank you. MEDLINK has been successfully programmed. Confirmation has been sent to your phone. You can now switch MEDLINK off. ");  // Send the desired string to convert to speech
  emicSerial.print('\n');
  emicSerial.flush();
  delay(2000);
   
  Serial.println("FINISHED SENDING TEXT.");
    while(1)
    {
  
    } 
    
}
void patient_reporting()
{
 Serial.println("I have entered PATIENT REPORTING FUNCTION");
 delay(1000);

File myFile;

if (!SD.begin(53)) {
    Serial.println("initialization failed!");
    while (1);
  }

  Serial.println("READING test.txt:");

  // read from the file until there's nothing else in it:
  int data;

 
  while ((data = myFile.read()) >= 0) Serial.write(data);
  // close the file:
  myFile.close();
  Serial.println("READING 1 DONE:");
  delay(3000); 
  

   //file = SD.open("test.txt", FILE_READ);
  myFile = SD.open("test.txt", FILE_READ);
  if(myFile) 
  {
   
      Serial.println("READING test.txt:");
     line_count = 0;
      while (myFile.available()) 
      {
      Serial.println("Reading");
      String line = myFile.readStringUntil('\n');  // \n character is discarded from buffer
      
      Serial.print("Line ");
      Serial.print(line_count);
      Serial.print(": ");
      Serial.println(line);
      tester[line_count] = line;
      line_count++;
    }
  }
   line_count = 0;
   Serial.println("Reading out Each Line");
   while(line_count < 8)
   {

  //  Serial.print("Line ");
   // Serial.print(line_count);
   // Serial.print(" : ");
    Serial.print(tester[line_count]);
    //Serial.println("L");
    line_count++;    
  }
  


 //if (!sd.begin(chipSelect, SPI_HALF_SPEED)) sd.initErrorHalt();
 
  // open the file for write at end like the Native SD library
 /* if (!myFile.open("test.txt", O_RDONLY))
  {
    //sd.errorHalt("opening test.txt for reading failed");
      Serial.println("Nothing");
      emicSerial.print('S');
      emicSerial.print("MEDLINK has not been programmed by your Doctor. Please give MEDLINK to your Doctor first before using it.");  // Send the desired string to convert to speech
      emicSerial.print('\n');
      emicSerial.flush();
      delay(2000);
      while(1)
      {
        
      }
   }*/

  // read from the file until there's nothing else in it:
  //int count = 0;
  //int n;
  //String tester[13];
  //char line[50];
  /*while ((n = myFile.fgets(line, sizeof(line))) > 0) 
  {
    
    if (line[n - 1] == '\n') 
    {
      tester[count] = String(line);
      count++;
    } 
    else 
    {
      Serial.println("Nothing");
    }
  }
  myFile.close();
  delay(1000);
*/
 /* int a = 0;
  while(a < count)
  {
    Serial.print("Line ");
    Serial.print(a);
    Serial.print(" : ");
    Serial.println(tester[a]);
    a++;    
  }*/

  emicSerial.print('N');
  emicSerial.print(1);
  emicSerial.print('\n');
  emicSerial.flush();
  emicSerial.print('V');
  emicSerial.print(18);
  emicSerial.print('\n');
  emicSerial.flush();

  emicSerial.print('S');
  emicSerial.print("Welcome");  // Send the desired string to convert to speech
  emicSerial.print('\n');
  emicSerial.flush();
  delay(2000);
  
  emicSerial.print('S');
  emicSerial.print(tester[2]);  // Send the desired string to convert to speech
  emicSerial.print('\n');
  emicSerial.flush();
  delay(3000);
  
  emicSerial.print('S');
  emicSerial.print("I will be taking your measurments for Doctor");  // Send the desired string to convert to speech
  emicSerial.print('\n');
  emicSerial.flush();
  delay(3500);
  
  emicSerial.print('S');
  emicSerial.print(tester[0]);  // Send the desired string to convert to speech
  emicSerial.print('\n');
  emicSerial.flush();
  delay(3000);
  
 Serial.println("I am here");
 Serial.println(tester[4]);
 String tt = "1";
// String tt1 = tester[4].toUpperCase();
 //String tta = tt.toUpperCase();
 tester[4].toUpperCase();
 Serial.println(tt);
 int result;
 
 result = tester[5].compareTo(tt);
 Serial.print("Result: ");
 Serial.println(result);

    
//  if(tester[5].equals(tt))


  if (result == 13)
   { 
      //RESPIRATION
     Serial.println("Taking Spirometer Data . . .");  // Send the desired string to convert to speech
     
      
      emicSerial.print('S');
      emicSerial.print("Connect the Pressure Sensor to the cables in Box 2 and press Enter");  // Send the desired string to convert to speech
      emicSerial.print('\n');
      emicSerial.flush();
      delay(1000);
      keyinput();
      Serial.print("This is your input: ");
      Serial.println(kinput);

    
      
      emicSerial.print('S');
      emicSerial.print("Thank You. Press Enter and Blow into the Pressure Sensor");  // Send the desired string to convert to speech
      emicSerial.print('\n');
      emicSerial.flush();
      delay(1000);
       keyinput();
      Serial.print("This is your input: ");
      Serial.println(kinput);
      spirometer2();
      
    
       
       Serial.println("Done");  // Send the desired string to convert to speech
     
      
           
        emicSerial.print('S');
        emicSerial.print("Measurement Taken");  // Send the desired string to convert to speech
        emicSerial.print('\n');
        emicSerial.flush();
        delay(4000);
        
        emicSerial.print('S');
        emicSerial.print("Thank You");  // Send the desired string to convert to speech
        emicSerial.print('\n');
        emicSerial.flush();
        delay(4000);
      
     
   }

   
        result = tester[4].compareTo(tt);
        Serial.print("Result: ");
        Serial.println(result);
   
   if(result == 13)
   { 
     Serial.println("I am here");
      //Heart Rate
      emicSerial.print('S');
      emicSerial.print("Place your fore finger inside Box 4 and press Enter");  // Send the desired string to convert to speech
      emicSerial.print('\n');
      emicSerial.flush();                                                                 
      delay(3000);
      keyinput();
      delay(1000);
      heart_rate();
      delay(2000);
      Serial.println("Heart Rate Measured!");
      Serial.print("Heart Rate - ");
      Serial.println(hrate/10);
      Serial.print("Average Heart Rate - ");
      Serial.println(hrate2/10);
      delay(3000);
      emicSerial.print('S');
      emicSerial.print(hrate/10);  // Send the desired string to convert to speech
      emicSerial.print('\n');
      emicSerial.flush();
      delay(4000);
      emicSerial.print('S');
      emicSerial.print("Thank You");  // Send the desired string to convert to speech
      emicSerial.print('\n');
      emicSerial.flush();
      delay(4000);
   }


        result = tester[6].compareTo(tt);
        Serial.print("Result: ");
        Serial.println(result);   
   if(result == 13)
   { 
      
        //SPO2
        emicSerial.print('S');
        emicSerial.print("Place your fore finger inside Box 3 and press Enter");  // Send the desired string to convert to speech
        emicSerial.print('\n');
        emicSerial.flush();                                                                 
   
   
        delay(3000);
        keyinput();
        delay(1000);
         oxygen2();
   }

    result = tester[7].compareTo(tt);
    Serial.print("Result: ");
    Serial.println(result);
  
  if (result == 13)
   { 
     //ECG
      emicSerial.print('S');
      emicSerial.print("Place your fore finger inside Box 1 and press Enter");  // Send the desired string to convert to speech
      emicSerial.print('\n');
      emicSerial.flush();                                                                 
      delay(3000);
      keyinput();
      delay(1000);
      ecg();
      delay(2000);
     
      emicSerial.print('S');
      emicSerial.print("Measurement Taken");  // Send the desired string to convert to speech
      emicSerial.print('\n');
      emicSerial.flush();
      delay(4000);
     
      emicSerial.print('S');
      emicSerial.print("Thank You");  // Send the desired string to convert to speech
      emicSerial.print('\n');
      emicSerial.flush();
      delay(4000);
 
   }


   /*    
        delay(2000);
}
        
        Serial.print("FINAL spo2 - ");
        Serial.print(spo2);
        Serial.println(" % ");
        delay(2000);
         
        emicSerial.print('S');
        emicSerial.print(spo2);  // Send the desired string to convert to speech
        emicSerial.print('\n');
        emicSerial.flush();
        delay(4000);
        
        emicSerial.print('S');
        emicSerial.print("Thank You");  // Send the desired string to convert to speech
        emicSerial.print('\n');
        emicSerial.flush();
        delay(4000);
        
   }
    

*/

      emicSerial.print('S');
      emicSerial.print("Thank You. Please wait while I send your measurments to your doctor.");  // Send the desired string to convert to speech
      emicSerial.print('\n');
      emicSerial.flush();
      delay(4000);
      Serial.println("FINISHED!!!");

   
      emicSerial.print('S');
      emicSerial.print("Thank You. You can now switch MEDLINK off.");  // Send the desired string to convert to speech
      emicSerial.print('\n');
      emicSerial.flush();
      delay(4000);
      Serial.println("FINISHED!!!"); 
     
    
   while(1)
   {   
   }
    
}

void refresh()
{
   float heartratespo2count = 0;
    float heartrate = 0;
    float heartratesum = 0;
    float spo2sum = 0;
    PulseOximeter pox;
    const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
    byte rates[RATE_SIZE]; //Array of heart rates
    byte rateSpot = 0;
    long lastBeat = 0; //Time at which the last beat occurred
    float beatsPerMinute;
    int beatAvg, count;
    uint32_t irBuffer[100]; //infrared LED sensor data
    uint32_t redBuffer[100];  //red LED sensor data
    int32_t bufferLength; //data length
    int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
    int32_t heartRate; //heart rate value
    int8_t validHeartRate; //indicator to show if the heart rate calculation is valid
    byte pulseLED = 30; //Must be on PWM pin
    byte readLED = 31; //Blinks with each data read
 Serial.println("Refreshing . . .");
 delay(1000);
 if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  
  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED

  delay(3000);
  count = 0;
  while(count < 200)
  {
        long irValue = particleSensor.getIR();
      
        if (checkForBeat(irValue) == true)
        {
          //We sensed a beat!
          long delta = millis() - lastBeat;
          lastBeat = millis();
      
          beatsPerMinute = 60 / (delta / 1000.0);
      
          if (beatsPerMinute < 255 && beatsPerMinute > 20)
          {
            rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
            rateSpot %= RATE_SIZE; //Wrap variable
      
            //Take average of readings
            beatAvg = 0;
            for (byte x = 0 ; x < RATE_SIZE ; x++)
              beatAvg += rates[x];
            beatAvg /= RATE_SIZE;
          }
        }
      
        Serial.print("IR=");
        Serial.print(irValue);
        Serial.print(", BPM=");
        Serial.print(beatsPerMinute);
        Serial.print(", Avg BPM=");
        Serial.print(beatAvg);
      
        if (irValue < 50000)
          Serial.print(" No finger?");
      
        Serial.println();  
        count = count + 1;
  
   }
    count = 0; 
}
void heart_rate()
{
   float heartratespo2count = 0;
    float heartrate = 0;
    float heartratesum = 0;
    float spo2sum = 0;
    PulseOximeter pox;
    const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
    byte rates[RATE_SIZE]; //Array of heart rates
    byte rateSpot = 0;
    long lastBeat = 0; //Time at which the last beat occurred
    float beatsPerMinute;
    int beatAvg, count;
    uint32_t irBuffer[100]; //infrared LED sensor data
    uint32_t redBuffer[100];  //red LED sensor data
    int32_t bufferLength; //data length
    int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
    int32_t heartRate; //heart rate value
    int8_t validHeartRate; //indicator to show if the heart rate calculation is valid
    byte pulseLED = 30; //Must be on PWM pin
    byte readLED = 31; //Blinks with each data read
 digitalWrite(24, LOW);
 digitalWrite(25, LOW );
 delay(3000);
 refresh();
 Serial.println("Starting heart rate measurement");
 delay(1000);
 if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED

  delay(3000);
  
  hrate = 0;
  hrate2 = 0;
  count = 0;
  while(count < 10)
  {
        long irValue = particleSensor.getIR();
      
        if (checkForBeat(irValue) == true)
        {
          //We sensed a beat!
          long delta = millis() - lastBeat;
          lastBeat = millis();
      
          beatsPerMinute = 60 / (delta / 1000.0);
      
          if (beatsPerMinute < 255 && beatsPerMinute > 20)
          {
            rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
            rateSpot %= RATE_SIZE; //Wrap variable
      
            //Take average of readings
            beatAvg = 0;
            for (byte x = 0 ; x < RATE_SIZE ; x++)
              beatAvg += rates[x];
            beatAvg /= RATE_SIZE;
          }
        }
      
        Serial.print("IR=");
        Serial.print(irValue);
        Serial.print(", BPM=");
        Serial.print(beatsPerMinute);
        Serial.print(", Avg BPM=");
        Serial.print(beatAvg);
      
        if (irValue < 1000)
          Serial.print(" No finger?");
      
        Serial.println();

        if ((beatsPerMinute > 55) && (beatsPerMinute < 140)&&(irValue > 1500))
        {
          count = count + 1;
          Serial.print("GOT IT");
         // delay(4000);
          hrate = hrate + beatsPerMinute;
          hrate2 = hrate + beatAvg;
        }
   }

}



void oxygen2()
 {
  
  float heartratespo2count = 0;
    float heartrate = 0;
    float heartratesum = 0;
    float spo2sum = 0;
    PulseOximeter pox;
    const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
    byte rates[RATE_SIZE]; //Array of heart rates
    byte rateSpot = 0;
    long lastBeat = 0; //Time at which the last beat occurred
    float beatsPerMinute;
    int beatAvg, count;
    uint32_t irBuffer[100]; //infrared LED sensor data
    uint32_t redBuffer[100];  //red LED sensor data
    int32_t bufferLength; //data length
    int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
    int32_t heartRate; //heart rate value
    int8_t validHeartRate; //indicator to show if the heart rate calculation is valid
    byte pulseLED = 30; //Must be on PWM pin
    byte readLED = 31; //Blinks with each data read
    Serial.println("ENTERING o2 now!!!");
    delay(3000);
   
   Serial.println("ENTERING o2 now!!!");
   delay(3000);
   Serial.println("o22!!!");
    if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
    {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
     Serial.println("w!!!");
    }
 //Serial.println("Mw!!!");
 // Serial.println(F("Attach sensor to finger with rubber band. Press any key to start conversion"));
  //while (Serial.available() == 0) ; //wait until user presses a key
  //Serial.read();
  Serial.println("Starting oxygen . . .");
  delay(3000);
  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

 Serial.println("Mmmm");
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings

 Serial.println("now!!!");
  bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps

  //read the first 100 samples, and determine the signal range
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample

    Serial.print(F("red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.println(irBuffer[i], DEC);
  }

/*
  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
  while (spo2 < 90)
  {
    //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for (byte i = 25; i < 100; i++)
    {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    //take 25 sets of samples before calculating the heart rate.
    for (byte i = 75; i < 100; i++)
    {
      while (particleSensor.available() == false) //do we have new data?
        particleSensor.check(); //Check the sensor for new data

   
      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //We're finished with this sample so move to next sample
 */
      //send samples and calculation result to terminal program through UART
/*      Serial.print(F("red="));
      Serial.print(redBuffer[i], DEC);
      Serial.print(F(", ir="));
      Serial.print(irBuffer[i], DEC);

      Serial.print(F(", HR="));
      Serial.print(heartRate, DEC);

      Serial.print(F(", HRvalid="));
      Serial.print(validHeartRate, DEC);

      Serial.print(F(", SPO2="));
      Serial.print(spo2, DEC);

      Serial.print(F(", SPO2Valid="));
      Serial.println(validSPO2, DEC);
    //}
*/
    //After gathering 25 new samples recalculate HR and SP02
   /* maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

    if(spo2 == 99.9)
    {
      spo2 = 99;
    }
  } */
  delay(3000);
}
void oxygen()
 {
    float heartratespo2count = 0;
    float heartrate = 0;
    float heartratesum = 0;
    float spo2sum = 0;
    PulseOximeter pox;
    const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
    byte rates[RATE_SIZE]; //Array of heart rates
    byte rateSpot = 0;
    long lastBeat = 0; //Time at which the last beat occurred
    float beatsPerMinute;
    int beatAvg, count;
    uint32_t irBuffer[100]; //infrared LED sensor data
    uint32_t redBuffer[100];  //red LED sensor data
    int32_t bufferLength; //data length
    int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
    int32_t heartRate; //heart rate value
    int8_t validHeartRate; //indicator to show if the heart rate calculation is valid
    byte pulseLED = 30; //Must be on PWM pin
    byte readLED = 31; //Blinks with each data read
   Serial.println("ENTERING o2 now!!!");
   delay(3000);
   
   Serial.println("o22!!!");
   
    if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
    {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
     Serial.println("w!!!");
    }
 //Serial.println("Mw!!!");
 // Serial.println(F("Attach sensor to finger with rubber band. Press any key to start conversion"));
  //while (Serial.available() == 0) ; //wait until user presses a key
  //Serial.read();
  Serial.println("Starting oxygen . . .");
  delay(3000);
  
  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

 Serial.println("Mmmm");
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings

 Serial.println("now!!!");
  bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps

  //read the first 100 samples, and determine the signal range
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data
   
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    /*
    particleSensor.nextSample(); //We're finished with this sample so move to next sample
  */
    Serial.print(F("red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.println(irBuffer[i], DEC);
  }

/*
  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);


  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
  while (spo2 < 90)
  {
    //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for (byte i = 25; i < 100; i++)
    {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    //take 25 sets of samples before calculating the heart rate.
    for (byte i = 75; i < 100; i++)
    {
      while (particleSensor.available() == false) //do we have new data?
        particleSensor.check(); //Check the sensor for new data

   
      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //We're finished with this sample so move to next sample

      //send samples and calculation result to terminal program through UART
      Serial.print(F("red="));
      Serial.print(redBuffer[i], DEC);
      Serial.print(F(", ir="));
      Serial.print(irBuffer[i], DEC);

      Serial.print(F(", HR="));
      Serial.print(heartRate, DEC);

      Serial.print(F(", HRvalid="));
      Serial.print(validHeartRate, DEC);

      Serial.print(F(", SPO2="));
      Serial.print(spo2, DEC);

      Serial.print(F(", SPO2Valid="));
      Serial.println(validSPO2, DEC);
    }

    //After gathering 25 new samples recalculate HR and SP02
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

    if(spo2 == 99.9)
    {
      spo2 = 99;
    }
  }
  delay(3000); */
}


void spirometer2()
{
   int diff_pressure = 0;
float diff_pressure2 = 0;
float FEV1 = 0;
float FEV1a = 0;
float FEV1b = 0;
float FEV1c = 0;
float FEV1d = 0;
float FEV1e = 0;
float FVC = 0;
float FEV1_FVC_ratio = 0;
float d1 = 0.018; //area of bigger pipe with diameter of 1.8 cm ; Area  = pi * r2
float d2 = 0.005; //area of bigger pipe with diameter of 0.5 cm ; Area  = pi * r2
float area_1 = 3.14 * 0.012 * 0.012; //area of bigger pipe with diameter of 1.8 cm ; Area  = pi * r2
float area_2 = 0.000019625; //area of bigger pipe with diameter of 0.5 cm ; Area  = pi * r2
float rho = 1.225; // Density of air in kg/m3
float viscosity = 0.0000185; // 1.85 x 10-5 Pa/s is th viscosity of air at 25 degrees celsius 
float dt = 0.001; 
float massFlow = 0; // Mass flow rate calculated from pressure 
float volFlow = 0; // Calculated from mass flow rate 
float volume = 0; // 


 
  Serial.println("STARTING SPIROMETER TEST");
  delay(2000);
  Serial.println("BLOW INTO THE SPIROMETER FOR 5 SECONDS");
  delay(2000);
  
  
  diff_pressure = analogRead(A14);
   int a = 1;
    // while(a < 120)
    while(diff_pressure < 300)
    {
        diff_pressure = analogRead(A14);
        Serial.println(diff_pressure);
        delay(100);
    }
   startMillis = millis();       
   currentMillis = millis();
   while(currentMillis-startMillis <=1000)
      {        
        analogRead(A14);
        diff_pressure = analogRead(A14);
        diff_pressure2 = map(diff_pressure, 0, 1023, 0, 10000); //(pressure in pascals)
        //massFlow = 1000*sqrt((abs(diff_pressure2)*2*rho)/((1/(pow(area_2,2)))-(1/(pow(area_1,2))))); // Mass flow of air 
        //volFlow = massFlow/rho; // Volumetric flow of air 
        //volume = volFlow*dt + volume; // Total volume (essentially integrated over time)  
        if(diff_pressure2 > 0)
        {
                float one = area_1 * abs(diff_pressure2);
                Serial.print("ONE: ");
                Serial.println(one);
                float two = one * 1.184 * 2; // * 8 * viscosity;
                //float two = 0.02 * 8 * viscosity;
                Serial.print("TWO: ");
                Serial.println(two);
                //float three = one/two;
               // Serial.print("THREE: ");
                //Serial.println(three);
                
                volFlow = two;
                volume = volFlow;
                
             // float one = (2 * abs(diff_pressure2));
              //float two = rho * (1 - (pow((d2/d1),4)));
              //volFlow =  0.61 * (3.14/4) * pow(d2,2) * pow((one/two),0.5) ;    // Volumetric flow of air 
              //volume = volFlow; // Total volume (essentially integrated over time) 


            //  float one = abs(diff_pressure2)*2*rho;
            //  float two = pow(area_2,2);
            //  float three = 1/two;
            //  float four = pow(area_1,2);
            //  float five = 1/four;
            //  float six = three - five;
            // float seven = one/six;
            //  float eight = pow(seven, 0.5);
               
            //  massFlow = 1000 * eight;
            //  volFlow = massFlow/rho;
            //  volume = volFlow;
              
              
           //massFlow = 1000*sqrt( (abs(diff_pressure2)*2*rho)/((1/(pow(area_2,2)))-(1/(pow(area_1,2))))); // Mass flow of air 
           //volFlow = massFlow/rho; // Volumetric flow of air 
           //volume = volFlow*dt; // Total volume (essentially integrated over time)  
       
            Serial.print("Pressure: ");
            Serial.println(diff_pressure2);
            //Serial.print("Mass Flow: ");
            //Serial.println(massFlow);
            Serial.print("Volume Flow: ");
            Serial.println(volFlow);
            Serial.print("Volume: ");
            Serial.println(volume);
            FEV1 = FEV1 + volume;
            Serial.print("FEV1: ");
            Serial.println(FEV1);
            currentMillis = millis();
            }

        
      }
        
    Serial.print("FEV1: ");
    Serial.println(FEV1);
   
 //FEV1a
    startMillis = millis();       
    currentMillis = millis();
      while(currentMillis-startMillis <=1000)
      {        
        analogRead(A14);
        diff_pressure = analogRead(A14);
        diff_pressure2 = map(diff_pressure, 0, 1023, 0, 10000); //(pressure in pascals)
        //massFlow = 1000*sqrt((abs(diff_pressure2)*2*rho)/((1/(pow(area_2,2)))-(1/(pow(area_1,2))))); // Mass flow of air 
        //volFlow = massFlow/rho; // Volumetric flow of air 
        //volume = volFlow*dt + volume; // Total volume (essentially integrated over time)  
        if(diff_pressure2 > 0)
        {

                 float one = area_1 * abs(diff_pressure2);
                Serial.print("ONE: ");
                Serial.println(one);
                float two = one * 1.184 * 2; // * 8 * viscosity;
             // float two = 0.02 * 8 * viscosity;
                Serial.print("TWO: ");
                Serial.println(two);
               // float three = one/two;
               // Serial.print("THREE: ");
                //Serial.println(three);
                
                volFlow = two;
                volume = volFlow;
                
          // massFlow = 1000*sqrt( ((abs(diff_pressure2))*2*rho)/( (1/(pow(area_2,2)))-(1/(pow(area_1,2))) ) ); // Mass flow of air 
          // volFlow = massFlow/rho; // Volumetric flow of air 
           //volume = volFlow*dt; // Total volume (essentially integrated over time)  
          //float one = (2 * abs(diff_pressure2));
         // float two = rho * (1 - (pow((d2/d1),4)));
         // volFlow =  0.61 * (3.14/4) * pow(d2,2) * pow((one/two),0.5) ;    // Volumetric flow of air 
        //  volume = volFlow; // Total volume (essentially integrated over time) 


              //float one = abs(diff_pressure2)*2*rho;
              //float two = pow(area_2,2);
              //float three = 1/two;
              //float four = pow(area_1,2);
              //float five = 1/four;
              //float six = three - five;
              //float seven = one/six;
              //float eight = pow(seven, 0.5);
                
             // massFlow = 1000 * eight;
             // volFlow = massFlow/rho;
              //volume = volFlow;

              
          Serial.print("Pressure: ");
          Serial.println(diff_pressure2);
         // Serial.print("Mass Flow: ");
         // Serial.println(massFlow);
          Serial.print("Volume Flow: ");
          Serial.println(volFlow);
          Serial.print("Volume: ");
          Serial.println(volume);
          FEV1a = FEV1a+ volume;
          Serial.print("FEV1a: ");
          Serial.println(FEV1a);
          currentMillis = millis();
        }
      }
       Serial.print("TOTAL FEV1a: ");
       Serial.println(FEV1a);
        

       //FEV1b
    startMillis = millis();       
    currentMillis = millis();
      while(currentMillis-startMillis <=1000)
      {        
        analogRead(A14);
        diff_pressure = analogRead(A14);
        diff_pressure2 = map(diff_pressure, 0, 1023, 0, 10000); //(pressure in pascals)
        //massFlow = 1000*sqrt((abs(diff_pressure2)*2*rho)/((1/(pow(area_2,2)))-(1/(pow(area_1,2))))); // Mass flow of air 
        //volFlow = massFlow/rho; // Volumetric flow of air 
        //volume = volFlow*dt + volume; // Total volume (essentially integrated over time)  
        if(diff_pressure2 > 0)
        {

                 float one = area_1 * abs(diff_pressure2);
                Serial.print("ONE: ");
                Serial.println(one);
                float two = one * 1.184 * 2; // * 8 * viscosity;
             // float two = 0.02 * 8 * viscosity;
                Serial.print("TWO: ");
                Serial.println(two);
               // float three = one/two;
               // Serial.print("THREE: ");
                //Serial.println(three);
                
                volFlow = two;
                volume = volFlow;
                
          // massFlow = 1000*sqrt( ((abs(diff_pressure2))*2*rho)/( (1/(pow(area_2,2)))-(1/(pow(area_1,2))) ) ); // Mass flow of air 
          // volFlow = massFlow/rho; // Volumetric flow of air 
           //volume = volFlow*dt; // Total volume (essentially integrated over time)  
          //float one = (2 * abs(diff_pressure2));
         // float two = rho * (1 - (pow((d2/d1),4)));
         // volFlow =  0.61 * (3.14/4) * pow(d2,2) * pow((one/two),0.5) ;    // Volumetric flow of air 
        //  volume = volFlow; // Total volume (essentially integrated over time) 


              //float one = abs(diff_pressure2)*2*rho;
              //float two = pow(area_2,2);
              //float three = 1/two;
              //float four = pow(area_1,2);
              //float five = 1/four;
              //float six = three - five;
              //float seven = one/six;
              //float eight = pow(seven, 0.5);
                
             // massFlow = 1000 * eight;
             // volFlow = massFlow/rho;
              //volume = volFlow;

              
          Serial.print("Pressure: ");
          Serial.println(diff_pressure2);
         // Serial.print("Mass Flow: ");
         // Serial.println(massFlow);
          Serial.print("Volume Flow: ");
          Serial.println(volFlow);
          Serial.print("Volume: ");
          Serial.println(volume);
          FEV1b = FEV1b+ volume;
          Serial.print("FEV2a: ");
          Serial.println(FEV1b);
          currentMillis = millis();
        }
      }
       Serial.print("TOTAL FEV1b: ");
       Serial.println(FEV1b);
        
        

 //FEV1c
    startMillis = millis();       
    currentMillis = millis();
      while(currentMillis-startMillis <=1000)
      {        
        analogRead(A14);
        diff_pressure = analogRead(A14);
        diff_pressure2 = map(diff_pressure, 0, 1023, 0, 10000); //(pressure in pascals)
        //massFlow = 1000*sqrt((abs(diff_pressure2)*2*rho)/((1/(pow(area_2,2)))-(1/(pow(area_1,2))))); // Mass flow of air 
        //volFlow = massFlow/rho; // Volumetric flow of air 
        //volume = volFlow*dt + volume; // Total volume (essentially integrated over time)  
        if(diff_pressure2 > 0)
        {

                 float one = area_1 * abs(diff_pressure2);
                Serial.print("ONE: ");
                Serial.println(one);
                float two = one * 1.184 * 2; // * 8 * viscosity;
             // float two = 0.02 * 8 * viscosity;
                Serial.print("TWO: ");
                Serial.println(two);
               // float three = one/two;
               // Serial.print("THREE: ");
                //Serial.println(three);
                
                volFlow = two;
                volume = volFlow;
                
          // massFlow = 1000*sqrt( ((abs(diff_pressure2))*2*rho)/( (1/(pow(area_2,2)))-(1/(pow(area_1,2))) ) ); // Mass flow of air 
          // volFlow = massFlow/rho; // Volumetric flow of air 
           //volume = volFlow*dt; // Total volume (essentially integrated over time)  
          //float one = (2 * abs(diff_pressure2));
         // float two = rho * (1 - (pow((d2/d1),4)));
         // volFlow =  0.61 * (3.14/4) * pow(d2,2) * pow((one/two),0.5) ;    // Volumetric flow of air 
        //  volume = volFlow; // Total volume (essentially integrated over time) 


              //float one = abs(diff_pressure2)*2*rho;
              //float two = pow(area_2,2);
              //float three = 1/two;
              //float four = pow(area_1,2);
              //float five = 1/four;
              //float six = three - five;
              //float seven = one/six;
              //float eight = pow(seven, 0.5);
                
             // massFlow = 1000 * eight;
             // volFlow = massFlow/rho;
              //volume = volFlow;

              
          Serial.print("Pressure: ");
          Serial.println(diff_pressure2);
         // Serial.print("Mass Flow: ");
         // Serial.println(massFlow);
          Serial.print("Volume Flow: ");
          Serial.println(volFlow);
          Serial.print("Volume: ");
          Serial.println(volume);
          FEV1c = FEV1c+ volume;
          Serial.print("FEV1c: ");
          Serial.println(FEV1c);
          currentMillis = millis();
        }
      }
       Serial.print("TOTAL FEV1c: ");
       Serial.println(FEV1c);
        


 //FEV1d
    startMillis = millis();       
    currentMillis = millis();
      while(currentMillis-startMillis <=1000)
      {        
        analogRead(A14);
        diff_pressure = analogRead(A14);
        diff_pressure2 = map(diff_pressure, 0, 1023, 0, 10000); //(pressure in pascals)
        //massFlow = 1000*sqrt((abs(diff_pressure2)*2*rho)/((1/(pow(area_2,2)))-(1/(pow(area_1,2))))); // Mass flow of air 
        //volFlow = massFlow/rho; // Volumetric flow of air 
        //volume = volFlow*dt + volume; // Total volume (essentially integrated over time)  
        if(diff_pressure2 > 0)
        {

                 float one = area_1 * abs(diff_pressure2);
                Serial.print("ONE: ");
                Serial.println(one);
                float two = one * 1.184 * 2; // * 8 * viscosity;
             // float two = 0.02 * 8 * viscosity;
                Serial.print("TWO: ");
                Serial.println(two);
               // float three = one/two;
               // Serial.print("THREE: ");
                //Serial.println(three);
                
                volFlow = two;
                volume = volFlow;
                
          // massFlow = 1000*sqrt( ((abs(diff_pressure2))*2*rho)/( (1/(pow(area_2,2)))-(1/(pow(area_1,2))) ) ); // Mass flow of air 
          // volFlow = massFlow/rho; // Volumetric flow of air 
           //volume = volFlow*dt; // Total volume (essentially integrated over time)  
          //float one = (2 * abs(diff_pressure2));
         // float two = rho * (1 - (pow((d2/d1),4)));
         // volFlow =  0.61 * (3.14/4) * pow(d2,2) * pow((one/two),0.5) ;    // Volumetric flow of air 
        //  volume = volFlow; // Total volume (essentially integrated over time) 


              //float one = abs(diff_pressure2)*2*rho;
              //float two = pow(area_2,2);
              //float three = 1/two;
              //float four = pow(area_1,2);
              //float five = 1/four;
              //float six = three - five;
              //float seven = one/six;
              //float eight = pow(seven, 0.5);
                
             // massFlow = 1000 * eight;
             // volFlow = massFlow/rho;
              //volume = volFlow;

              
          Serial.print("Pressure: ");
          Serial.println(diff_pressure2);
         // Serial.print("Mass Flow: ");
         // Serial.println(massFlow);
          Serial.print("Volume Flow: ");
          Serial.println(volFlow);
          Serial.print("Volume: ");
          Serial.println(volume);
          FEV1d = FEV1d+ volume;
          Serial.print("FEV1d: ");
          Serial.println(FEV1a);
          currentMillis = millis();
        }
      }
       Serial.print("TOTAL FEV1d: ");
       Serial.println(FEV1d);
        

 //FEV1e
    startMillis = millis();       
    currentMillis = millis();
      while(currentMillis-startMillis <=1000)
      {        
        analogRead(A14);
        diff_pressure = analogRead(A14);
        diff_pressure2 = map(diff_pressure, 0, 1023, 0, 10000); //(pressure in pascals)
        //massFlow = 1000*sqrt((abs(diff_pressure2)*2*rho)/((1/(pow(area_2,2)))-(1/(pow(area_1,2))))); // Mass flow of air 
        //volFlow = massFlow/rho; // Volumetric flow of air 
        //volume = volFlow*dt + volume; // Total volume (essentially integrated over time)  
        if(diff_pressure2 > 0)
        {

                 float one = area_1 * abs(diff_pressure2);
                Serial.print("ONE: ");
                Serial.println(one);
                float two = one * 1.184 * 2; // * 8 * viscosity;
             // float two = 0.02 * 8 * viscosity;
                Serial.print("TWO: ");
                Serial.println(two);
               // float three = one/two;
               // Serial.print("THREE: ");
                //Serial.println(three);
                
                volFlow = two;
                volume = volFlow;
                
          // massFlow = 1000*sqrt( ((abs(diff_pressure2))*2*rho)/( (1/(pow(area_2,2)))-(1/(pow(area_1,2))) ) ); // Mass flow of air 
          // volFlow = massFlow/rho; // Volumetric flow of air 
           //volume = volFlow*dt; // Total volume (essentially integrated over time)  
          //float one = (2 * abs(diff_pressure2));
         // float two = rho * (1 - (pow((d2/d1),4)));
         // volFlow =  0.61 * (3.14/4) * pow(d2,2) * pow((one/two),0.5) ;    // Volumetric flow of air 
        //  volume = volFlow; // Total volume (essentially integrated over time) 


              //float one = abs(diff_pressure2)*2*rho;
              //float two = pow(area_2,2);
              //float three = 1/two;
              //float four = pow(area_1,2);
              //float five = 1/four;
              //float six = three - five;
              //float seven = one/six;
              //float eight = pow(seven, 0.5);
                
             // massFlow = 1000 * eight;
             // volFlow = massFlow/rho;
              //volume = volFlow;

              
          Serial.print("Pressure: ");
          Serial.println(diff_pressure2);
         // Serial.print("Mass Flow: ");
         // Serial.println(massFlow);
          Serial.print("Volume Flow: ");
          Serial.println(volFlow);
          Serial.print("Volume: ");
          Serial.println(volume);
          FEV1e = FEV1e+ volume;
          Serial.print("FEV1a: ");
          Serial.println(FEV1e);
          currentMillis = millis();
        }
      }
       Serial.print("TOTAL FEV1e: ");
       Serial.println(FEV1e);
        


       Serial.println("TOTAL MEASUREMENTS: ");
       Serial.print("FEV1: ");
       Serial.println(FEV1);
       Serial.print("FEV1a: ");
       Serial.println(FEV1a);
       Serial.print("FEV1b: ");
       Serial.println(FEV1b);
       Serial.print("FEV1c: ");
       Serial.println(FEV1c);
       Serial.print("FEV1d: ");
       Serial.println(FEV1d);
       Serial.print("FEV1e: ");
       Serial.println(FEV1e);

       //SCALED MEASUREMENTS
       Serial.println("TOTAL SCALED MEASUREMENTS: ");
       Serial.print("FEV1: ");
       Serial.println(FEV1/25);
       Serial.print("FEV1a: ");
       Serial.println(FEV1a/35);
       Serial.print("FEV1b: ");
       Serial.println(FEV1b/120);
       Serial.print("FEV1c: ");
       Serial.println(FEV1c/130);
       Serial.print("FEV1d: ");
       Serial.println(FEV1d/130);
       Serial.print("FEV1e: ");
       Serial.println(FEV1e/130);
       
       
      
       FVC = (FEV1a/35) + (FEV1b/120) + (FEV1c/130) + (FEV1d/130) + (FEV1e/130);
       FEV1_FVC_ratio = ((FEV1/25)/FVC);

       //Print Final Results
       
       Serial.println();
       Serial.print("FEV1: ");
       Serial.print(FEV1/25);
       Serial.println(" Litres");
       
       Serial.println();
       Serial.print("FVC: ");
       Serial.print(FVC);
       Serial.println(" Litres");
       
              
       Serial.print("FEV1/FVC Ratio: ");
       Serial.println(FEV1_FVC_ratio);

        Serial.println("TEST COMPLETED");
        delay(5000);
        
}

void bodytemperature()
{
      
  // Call therm.read() to read object and ambient temperatures from the sensor.
  int a = 0;
  while(a < 10)
  {
      digitalWrite(LED_BUILTIN, HIGH);
      // Call therm.read() to read object and ambient temperatures from the sensor.
      if (therm.read()) // On success, read() will return 1, on fail 0.
      {
        // Use the object() and ambient() functions to grab the object and ambient
      // temperatures.
      // They'll be floats, calculated out to the unit you set with setUnit().
        Serial.print("Object: " + String(therm.object(), 2));
        Serial.println("C");
        Serial.print("Ambient: " + String(therm.ambient(), 2));
        Serial.println("C");
        Serial.println();
        mytemp = therm.object();
        Serial.print("My Temperature: ");
        Serial.println(mytemp);
        Serial.println();
        a= a+1;
  }
  //digitalWrite(LED_BUILTIN, LOW);

  delay(2000);

  
  }
  
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
}

 
  
//FINAL
void blood_glucose()
{
  Serial.println("Place your finger between the IR sensors");
  delay(2000); 
  ifRead = analogRead(A15);
  Serial.println(ifRead);
  delay(1000);

  int bgcount = 0;
  while(bgcount< 10)
  {
      ifRead = analogRead(A15);
      Serial.println(ifRead);
      bloodglucose[bgcount] = ifRead;
      avbg = avbg + bloodglucose[bgcount];
      bgcount = bgcount + 1;
      delay(1000);
  }
  
  Serial.print("Average Blood Glucose: ");
  Serial.println(avbg/10);

  //Convert analog output from IR GLucometer to mmol/L using calibration with PAAS glucose moniitoring system 
  //y = 0.0382x - 0.5804
  //x is the analog output from the glucometer
  //y is the output from the glucometer in mmol/L

  mmolL = (0.0382 * (avbg/10)) - 0.5804;
  Serial.print("Average Blood Glucose in mmol/L: ");
  Serial.println(mmolL); 
  avbg = 0;

}
  

 //FINAL
void ecg()
{
   
      startMillis = millis();       
      currentMillis = millis();
      while(currentMillis-startMillis <=100)
      {        
         if((digitalRead(15) == 1)||(digitalRead(16) == 1))
          {
           
            ;//Serial.println('!');
          }
        
         else
          {
             
            // read in the value of ECG analog input and convert to Voltage using (value/1023 * 5)
            val = (float(analogRead(A15)* 5)/1023); 
           
            ecg_data.concat(val); 
            ecg_data.concat(" ");/*
             Serial.println("C"); */
            
          }
           
          //Wait for a bit to keep serial data from saturating
          delay(1);
          currentMillis = millis(); 
      }
      Serial.println("FINISED ECG!");
      Serial.println("ECG Output:");
      Serial.println(ecg_data);
      Serial.println("Note: Each reading is acquired in 1.25ms");
      Serial.println("");
      delay(8000);
}


void keyinput() // function to obtain keyboard input from the physician or the patient
{
  char key; char key2;
  int ct = 1;
  kinput ="";
  while(ct == 1)
  {
    key = keypad.getKey();
    key2 = kpd.getKey();
    if(key)
    {
         kinput+=key;
    }
    if(key2)
    {
      if(key2 == '#')
      {
        ct = 2;
      }
      else
      {
         kinput+=key2;
      }
    }
      
  }
}
