     /*
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

/* MEDLINK and the software code is owned by Dr. Jason M. Zara, Dr. Vesna Zderic, Dr. Murray H. Loew, Dr. Erkinay Abliz, Dr. Shani Ross,
Dr. Ahmed Jendoubi, Dr. Mohamed Chouikha, Dr. Gary Harris, Mr. James Griffin, Engineer Francis Olawuyi, Dr. Matthew Olawuyi,
Engineer Joshua Olawuyi, Engineer Deborah Olawuyi, Engineer Joseph Olawuyi, Dr. Michael Olawuyi, Honorable Damilola Sunday Olawuyi
and Dr. Esther Olawuyi.
OLAWUYI RACETT NIGERIA LTD.,
KEMP HOUSE, 160 CITY ROAD, 
EC1V 2NX, LONDON, UNITED KINGDOM
https://www.olawuyiracettnigerialtd.com
tegae@gwmail.gwu.edu
July 20, 2023.
*/
       */
       

// include the SoftwareSerial library so we can use it to talk to the Emic 2 module
#include <SoftwareSerial.h>
#include <SdFat.h>
#include <Wire.h>
#include "MAX30100_PulseOximeter.h"
#include "MAX30105.h"
#include <INA219_WE.h>
#include <TinyGPS++.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Keypad.h>
#include "heartRate.h"
#include "spo2_algorithm.h"

MAX30105 particleSensor;

#define REPORTING_PERIOD_MS     1000


#define MAX_BRIGHTNESS 255
String textMessage;        // Variable to store text message
String ledState;           // Create a variable to store LED state
String Message;

SoftwareSerial GPSmodule(A8, A9); // U_TXD, U_RXD 
int en = 6; //power key
#define DEBUG true
String data[5];
String response;
String ppvet = "0001";
String sendData(String command, const int timeout, boolean debug);



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
  {'.','@',' ','#'},
};

// Keypad Connections: from '1' to '9'
// Keypad 1: connect 1, 2, 3, 4,, 5, 6, 7, 8 & 9, to  arduino pins 45, 43, 41, 39, 37, 35, 33, 31, & 29 respectively.
// Keypad 2: connect 1, 2, 3, 4, 5, 6, 7, 8 & 9, to  arduino pins 44, 42, 40, 38, 36, 34, 32, 30, & 9 respectively.

byte rowPins1[ROWS1] = {29, 31, 33, 35, 37}; // connect to pins 9, 8, 7, 6, 5 pinouts of the keypad (row pinouts)
byte colPins1[COLS1] = {45, 43, 41, 39}; // connect to pins 1, 2, 3, 4 pinouts of the keypad     (column pinouts)
byte rowPins2[ROWS2] = {28, 30, 32, 34, 36}; // connect to pins 9,8,7,6,5 pinouts of the keypad (row pinouts)
byte colPins2[COLS2] = {44, 42, 40, 38}; // connect to pins 1, 2, 3, 4 pinouts of the keypad (column pinouts)

//initlaize and create the 2 key pads
Keypad keypad = Keypad( makeKeymap(keys1), rowPins1, colPins1, ROWS1, COLS1);
Keypad kpd = Keypad( makeKeymap(keys2), rowPins2, colPins2, ROWS2, COLS2);


#define ONE_WIRE_BUS 2

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
 
#define DEBUG true
#define I2C_ADDRESS 0x40
#define REPORTING_PERIOD_MS     1000

//Declare Variables
INA219_WE ina219(I2C_ADDRESS);
const int chipSelect = 53;

String kinput;
String phyname, phyphone, patname, patid, physsid, phypword, patssid, patpword, stpara;
int mode;
int phyparameters[8];

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


SdFat sd;
SdFile myFile;


float val = 0;
float heartratespo2count = 0;
float heartrate = 0;
float heartratesum = 0;
float spo2sum = 0;
float btemperature = 0;
PulseOximeter pox;
float ppulse = 0;
float emg= 0;
float spirometer = 0;

uint32_t tsLastReport = 0;
unsigned long startMillis;
unsigned long currentMillis;
const unsigned long period = 1000;

int bcount = 0;
int ifRead;
int bloodglucose[5];
float avbg = 0;
float mmolL = 0;

//A1 is the analog pin  used by ECG module
//ECG Lo+ Pin used Digital Pin 15
//ECG Lo- Pin uses Digital Pin 16
const unsigned long period2 = 10000;
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
const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
float beatsPerMinute;
int beatAvg, count;
float hrate, hrate2;
float prate, prate2;

uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data

int32_t bufferLength; //data length
int32_t spo2 = 0; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

byte pulseLED = 30; //Must be on PWM pin
byte readLED = 31; //Blinks with each data read


int x, y;
void phyinput();
void body_temperature();
void patient_reporting();
void physician_programming();
void printvalue(float a);
void printecg(String a);
void ecg();
void call(void);
String sendData(String command, const int timeout, boolean debug);
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
  pinMode(en,OUTPUT);
  pinMode(A0, INPUT);
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  pinMode(chipSelect, OUTPUT);
  pinMode(A2, INPUT);
  pinMode(15, INPUT); // Setup for leads off detection ECG LO +
  pinMode(16, INPUT); // Setup for leads off detection ECG LO -
  pinMode(A12, INPUT); // Setup for leads glucometer reading

  pinMode(22, OUTPUT); //Relay connections for Heart Rate, Pulse, and SPO2 sesnors
  pinMode(23, OUTPUT);  
  pinMode(24, OUTPUT);
  pinMode(25, OUTPUT);
  pinMode(26, OUTPUT);
  pinMode(27, OUTPUT);

  //LOW TURNS THE 8V CHANNEL RELAY ON
  //HIGH TURNS THE 8V CHANNEL RELAY OFF
   
  digitalWrite(22,HIGH);  
  digitalWrite(23,HIGH);
  digitalWrite(24,HIGH);  
  digitalWrite(25,HIGH);
  digitalWrite(26,HIGH);  
  digitalWrite(27,HIGH);
  delay(1000);
 
  
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
    
   sensors.begin(); //for body temperature sensor on digital pin 2
   delay(1000);
  
  GPSmodule.begin(9600);    // Initialise serial communicstion with GPS module at 9600 bps
  delay(2000);              // Give time to your GSM GPS module logon to GSM network
  Serial.println("GPSmodule ready...");  //Print test in Serial Monitor
   
  digitalWrite(en, LOW);
  delay(3000);
  digitalWrite(en, HIGH);
  delay(8000);
  
 
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
  if(sensorValue < 401)
   {
       Serial.println("MEDLINK is in PHYSICIAN REPORTING MODE");
       delay(2000);
       physician_programming();
   }
  else if(sensorValue>400)
   {
      Serial.println("MEDLINK is in PATIENT PROGRAMMING MODE");
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
  emicSerial.print("Please type in your phone number and press the ENTER Key");  // Send the desired string to convert to speech
  emicSerial.print('\n');
  emicSerial.flush();
 
  keyinput();
  Serial.print("This is your input: ");
  Serial.println(kinput);
  phyphone = kinput;

 
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
  emicSerial.print("Would you like this patient to report heart rate? Please type YES or NO");
  emicSerial.print('\n');
  emicSerial.flush();
  delay(3000);

 
  keyinput();
  Serial.print("This is your input: ");
  Serial.println(kinput);
   
  while((kinput != "YES") && (kinput != "NO"))
   {
      emicSerial.print('S');
      emicSerial.print("Incorrect response. Should this patient report heart rate?");
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
  emicSerial.print("Would you like this patient to report Pulse? Please type YES or NO");
  emicSerial.print('\n');
  emicSerial.flush();
  delay(3000);

  
  keyinput();
  Serial.print("This is your input: ");
  Serial.println(kinput);

  
  while((kinput != "YES") && (kinput != "NO"))
   {
      emicSerial.print('S');
      emicSerial.print("Incorrect response. Should this patient report Pulse?");
      emicSerial.print('\n');
      emicSerial.flush();
      keyinput();
      Serial.print("This is your input: ");
      Serial.println(kinput);
   }
  
  if(kinput == "YES")
  {
      phyparameters[1] = 1;
      two = "Pulse"; 
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
    three = "SPO2 ";
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
  emicSerial.print("Would you like this patient to report Blood Sugar ? Please type YES or NO");
  emicSerial.print('\n');
  emicSerial.flush();
  delay(3000);

  
  keyinput();
  Serial.print("This is your input: ");
  Serial.println(kinput);

  
  while((kinput != "YES") && (kinput != "NO"))
   {
      emicSerial.print('S');
      emicSerial.print("Incorrect response. Should this patient report Blood Sugar?");
      emicSerial.print('\n');
      emicSerial.flush();
      keyinput();
      Serial.print("This is your input: ");
      Serial.println(kinput);
   }
  
  if(kinput == "YES")
  {
    phyparameters[3] = 1;
     four = "Blood Sugar ";
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
   

 

//PARAMETER 5
  
  emicSerial.print('S');
  emicSerial.print("Would you like this patient to report E C G ? Please type YES or NO");
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
    phyparameters[4] = 1;
     five = "ECG "; 
        Serial.print("Parameter 4: ");
  Serial.println(phyparameters[4]);
  }
  else if(kinput == "NO")
  {
    phyparameters[4] = 0;
        Serial.print("Parameter 4: ");
  Serial.println(phyparameters[4]);
  }

  
  emicSerial.print('S');
  emicSerial.print("Thank you");  // Send the desired string to convert to speech
  emicSerial.print('\n');
  emicSerial.flush();
  delay(3000);
  
//automatically say No  for - EMG and Respiratry Data, because MEDlINK 2 cannot currenty measure these parameters 
  phyparameters[5] = 0;
  phyparameters[6] = 0;


/*
//PARAMETER 6
  
  emicSerial.print('S');
  emicSerial.print("Would you like this patient to report E M G? Please type YES or NO");
  emicSerial.print('\n');
  emicSerial.flush();
  delay(3000);

  
  keyinput();
  Serial.print("This is your input: ");
  Serial.println(kinput);

  
  while((kinput != "YES") && (kinput != "NO"))
   {
      emicSerial.print('S');
      emicSerial.print("Incorrect response. Should this patient report E M G?");
      emicSerial.print('\n');
      emicSerial.flush();
      keyinput();
      Serial.print("This is your input: ");
      Serial.println(kinput);
   }
   
  if(kinput == "YES")
  {
    phyparameters[5] = 1;
    six = "EMG ";
        Serial.print("Parameter 5: ");
  Serial.println(phyparameters[5]);
  }
  else if(kinput == "NO")
  {
    phyparameters[5] = 0;
        Serial.print("Parameter 5: ");
  Serial.println(phyparameters[5]);
  }

 
  emicSerial.print('S');
  emicSerial.print("Thank you");  // Send the desired string to convert to speech
  emicSerial.print('\n');
  emicSerial.flush();
  delay(3000);
  

//PARAMETER 7
  
  
  emicSerial.print('S');
  emicSerial.print("Would you like this patient to report Respiratory Data? Please type YES or NO");
  emicSerial.print('\n');
  emicSerial.flush();
  delay(3000);

  
  keyinput();
  Serial.print("This is your input: ");
  Serial.println(kinput);

  
  while((kinput != "YES") && (kinput != "NO"))
   {
      emicSerial.print('S');
      emicSerial.print("Incorrect response. Should this patient report Respiratory Data?");
      emicSerial.print('\n');
      emicSerial.flush();
      keyinput();
      Serial.print("This is your input: ");
      Serial.println(kinput);
   }
  
  if(kinput == "YES")
  {
    phyparameters[6] = 1;
    seven = "Spirometer";
        Serial.print("Parameter 6: ");
  Serial.println(phyparameters[6]);
  }
  else if(kinput == "NO")
  {
    phyparameters[6] = 0;
        Serial.print("Parameter 6: ");
  Serial.println(phyparameters[6]);
  }

  
  emicSerial.print('S');
  emicSerial.print("Thank you");  // Send the desired string to convert to speech
  emicSerial.print('\n');
  emicSerial.flush();
  delay(3000);
  
*/

//PARAMETER 8
    
  emicSerial.print('S');
  emicSerial.print("Would you like this patient to report Body Temperature? Please type YES or NO");
  emicSerial.print('\n');
  emicSerial.flush();
  delay(3000);
  

  keyinput();
  Serial.print("This is your input: ");
  Serial.println(kinput);

  
  while((kinput != "YES") && (kinput != "NO"))
   {
      emicSerial.print('S');
      emicSerial.print("Incorrect response. Should this patient report Body Temperature?");
      emicSerial.print('\n');
      emicSerial.flush();
      keyinput();
      Serial.print("This is your input: ");
      Serial.println(kinput);
   }
  
  if(kinput == "YES")
  {
    phyparameters[7] = 1;
    eight = "Temperature";
        Serial.print("Parameter 7: ");
  Serial.println(phyparameters[7]);
  }
  else if(kinput == "NO")
  {
    phyparameters[7] = 0;
        Serial.print("Parameter 7: ");
  Serial.println(phyparameters[7]);
  }


  emicSerial.print('S');
  emicSerial.print("Thank you");  // Send the desired string to convert to speech
  emicSerial.print('\n');
  emicSerial.flush();
  delay(3000);
  


  emicSerial.print('S');
  emicSerial.print("Thank you. Please Wait. ");  // Send the desired string to convert to speech
  emicSerial.print('\n');
  delay(3000);
 
  // send confirmation text message
  
 stpara = String(phyparameters[0]) + "-" + String(phyparameters[1]) + "-" + String(phyparameters[2]) + "-"+ String(phyparameters[3]) + "-" + String(phyparameters[4]) + "-" + String(phyparameters[5]);
//Serial.print("Parameter String: ");
//Serial.println(stpara);
     
  Serial.println("FINISHED!");
  Serial.print("Parameter 0: ");
  Serial.println(phyparameters[0]);
    Serial.print("Parameter 1: ");
  Serial.println(phyparameters[1]);
    Serial.print("Parameter 2: ");
  Serial.println(phyparameters[2]);
    Serial.print("Parameter 3: ");
  Serial.println(phyparameters[3]);
    Serial.print("Parameter 4: ");
  Serial.println(phyparameters[4]);
    Serial.print("Parameter 5: ");
  Serial.println(phyparameters[5]);
    Serial.print("Parameter 6: ");
  Serial.println(phyparameters[6]);
    Serial.print("Parameter 7: ");
  Serial.println(phyparameters[7]);
  delay(1000);


  if (!sd.begin(chipSelect, SPI_HALF_SPEED)) sd.initErrorHalt();
  
  if(!sd.remove("test.txt")) { Serial.println("Cannot Delete File");}
  
  // open the file for write at end like the Native SD library
  if (!myFile.open("test.txt", O_RDWR | O_CREAT | O_AT_END)) {
    sd.errorHalt("opening test.txt for write failed");
  }
  
   // if the file opened okay, write to it:
  Serial.print("Writing to test.txt...");
  myFile.println(phyname);
  myFile.println(phyphone);
  myFile.println(patname);
  myFile.println(patid);
  myFile.println(phyparameters[0]);
  myFile.println(phyparameters[1]);
  myFile.println(phyparameters[2]);
  myFile.println(phyparameters[3]);
  myFile.println(phyparameters[4]);
  myFile.println(phyparameters[5]);
  myFile.println(phyparameters[6]);
  myFile.println(phyparameters[7]);
  myFile.println();
  // close the file:
  myFile.close();
  Serial.println("done.");
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

  
  sendSMS1("MEDLINK 0001", phyname, phyphone, patname, patid, one, two, three, four, five, six, seven, eight);
 
  delay(8000);
  
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

 if (!sd.begin(chipSelect, SPI_HALF_SPEED)) sd.initErrorHalt();
 
  // open the file for write at end like the Native SD library
  if (!myFile.open("test.txt", O_RDONLY))
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
   }

  // read from the file until there's nothing else in it:
  int count = 0;
  int n;
  String tester[13];
  char line[50];
  while ((n = myFile.fgets(line, sizeof(line))) > 0) 
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

  int a = 0;
  while(a < count)
  {
    Serial.print("Line ");
    Serial.print(a);
    Serial.print(" : ");
    Serial.println(tester[a]);
    a++;    
  }

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
  
 
  if(tester[4] == "1\n")
   { 
    
      //Heart Rate
      emicSerial.print('S');
      emicSerial.print("Place your fore finger inside Box 1 and press Enter");  // Send the desired string to convert to speech
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
      heartrate = hrate/10;
      delay(3000);
      emicSerial.print('S');
      emicSerial.print(heartrate);  // Send the desired string to convert to speech
      emicSerial.print('\n');
      emicSerial.flush();
      delay(4000);
      emicSerial.print('S');
      emicSerial.print("Thank You");  // Send the desired string to convert to speech
      emicSerial.print('\n');
      emicSerial.flush();
      delay(4000);
      
   }
   
    if (tester[5] == "1\n")
   { 
      //Pulse
      emicSerial.print('S');
      emicSerial.print("Place your fore finger inside Box 2 and press Enter");  // Send the desired string to convert to speech
      emicSerial.print('\n');
      emicSerial.flush();                                                                 
      delay(3000);
      keyinput();
      delay(1000);
      Pulse();
      delay(2000);
      Serial.println("Pulse Measured!");
      Serial.print("Pulse - ");
      Serial.println(prate/10);
      Serial.print("Average Pulse - ");
      Serial.println(prate2/10);
      ppulse = prate/10;
      delay(2000);  
      emicSerial.print('S');
      emicSerial.print(ppulse);  // Send the desired string to convert to speech
      emicSerial.print('\n');
      emicSerial.flush();
      delay(4000);
      emicSerial.print('S');
      emicSerial.print("Thank You");  // Send the desired string to convert to speech
      emicSerial.print('\n');
      emicSerial.flush();
      delay(4000);
      
   }
     if (tester[6] == "1\n")
   { 
      
        //SPO2
        emicSerial.print('S');
        emicSerial.print("Place your fore finger inside Box 3 and press Enter");  // Send the desired string to convert to speech
        emicSerial.print('\n');
        emicSerial.flush();                                                                 
        delay(3000);
        keyinput();
        delay(1000);
        oxygen();
        delay(2000);
        
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
    
  
 if (tester[7] == "1\n")
   {  
     
      //Blood Glucose
       
      emicSerial.print('S');
      emicSerial.print("Place your fore finger inside Box 5 and press Enter");  // Send the desired string to convert to speech
      emicSerial.print('\n');
      emicSerial.flush();                                                                 
      delay(1000);
      keyinput();
      Serial.print("This is your input: ");
      Serial.println(kinput);
      
      
      blood_glucose();
      
      emicSerial.print('S');
      emicSerial.print(mmolL);  // Send the desired string to convert to speech
      emicSerial.print('\n');
      emicSerial.flush();
      delay(4000);
      
      emicSerial.print('S');
      emicSerial.print("Thank You");  // Send the desired string to convert to speech
      emicSerial.print('\n');
      emicSerial.flush();
      delay(4000);  
   }

  if (tester[8] == "1\n")
   { 
      //ECG
     Serial.println("Taking ECG . . .");  // Send the desired string to convert to speech
     
      
      emicSerial.print('S');
      emicSerial.print("Connect the E C G cable in Box 7 and press Enter");  // Send the desired string to convert to speech
      emicSerial.print('\n');
      emicSerial.flush();
      delay(1000);
      keyinput();
      Serial.print("This is your input: ");
      Serial.println(kinput);

    
      
      emicSerial.print('S');
      emicSerial.print("Thank You. Place the Electrodes on your body and press Enter");  // Send the desired string to convert to speech
      emicSerial.print('\n');
      emicSerial.flush();
      delay(1000);
       keyinput();
      Serial.print("This is your input: ");
      Serial.println(kinput);
     
      
    
        ecg();
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

 if (tester[11] == "1\n")
   { 
      //Body Temperature
      Serial.println("Taking Body Temperature . . .");  // Send the desired string to convert to speech
     
      
      emicSerial.print('S');
      emicSerial.print("Place your fore finger on the sensor at Number 4 and press Enter");  // Send the desired string to convert to speech
      emicSerial.print('\n');
      emicSerial.flush();                                                                 
      delay(1000);
      keyinput();
      Serial.print("This is your input: ");
      Serial.println(kinput);
     
      body_temperature();
    
      
      emicSerial.print('S');
      emicSerial.print(btemperature);  // Send the desired string to convert to speech
      emicSerial.print('\n');
      emicSerial.flush();
      delay(4000);
      emicSerial.print('S');
      emicSerial.print("Thank You");  // Send the desired string to convert to speech
      emicSerial.print('\n');
      emicSerial.flush();
      delay(4000);
      
   }

 


     

      emicSerial.print('S');
      emicSerial.print("Thank You. Please wait while I send your measurments to your doctor.");  // Send the desired string to convert to speech
      emicSerial.print('\n');
      emicSerial.flush();
      delay(4000);

      sendSMS2("MEDLINK 0001", tester[1], tester[3], heartrate, ppulse, spo2, mmolL, ecg_data, emg, spirometer, btemperature);
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
            val = (float(analogRead(A1)* 5)/1023); 
           
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

 
//FINAL
void body_temperature()
{
      int btcount = 0;
      while(btcount < 16)
     {

          sensors.requestTemperatures(); // Send the command to get temperatures
           
            // put your main code here, to run repeatedly:
          btemperature = sensors.getTempCByIndex(0);
          Serial.print("Temperature: ");
          Serial.print(btemperature);
          Serial.println(" oC ");
          delay(1000);
          btcount++;
       }   
        Serial.print("FINISHED: ");
}

//FINAL
void blood_glucose()
{
  Serial.println("Place your finger between the IR sensors");
  delay(2000); 
  ifRead = analogRead(A12);
  Serial.println(ifRead);
  delay(1000);

  int bgcount = 0;
  while(bgcount< 5)
  {
      ifRead = analogRead(A12);
      Serial.println(ifRead);
      bloodglucose[bgcount] = ifRead;
      avbg = avbg + bloodglucose[bgcount];
      bgcount = bgcount + 1;
      delay(1000);
  }
  
  Serial.print("Average Blood Glucose: ");
  Serial.println(avbg/5);

  //Convert analog output from IR GLucometer to mmol/L using calibration with PAAS glucose moniitoring system 
  //y = 0.0382x - 0.5804
  //x is the analog output from the glucometer
  //y is the output from the glucometer in mmol/L

  mmolL = (0.0382 * (avbg/5)) - 0.5804;
  Serial.print("Average Blood Glucose in mmol/L: ");
  Serial.println(mmolL); 
  avbg = 0;

}
 
void refresh()
{
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
      
        if (irValue < 50000)
          Serial.print(" No finger?");
      
        Serial.println();

        if ((beatsPerMinute > 55) && (beatsPerMinute < 140)&&(irValue > 50000))
        {
          count = count + 1;
          Serial.print("GOT IT");
         // delay(4000);
          hrate = hrate + beatsPerMinute;
          hrate2 = hrate + beatAvg;
        }
   }
 digitalWrite(24, HIGH);
 digitalWrite(25, HIGH);
 delay(3000);  
}


void Pulse()
{
 digitalWrite(22,LOW);
 digitalWrite(23,LOW);
 delay(3000);
 refresh();
 Serial.println("Starting Pulse measurement");
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
  prate = 0;
  prate2 = 0;
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
      
        if (irValue < 50000)
          Serial.print(" No finger?");
      
        Serial.println();

        if ((beatsPerMinute > 55) && (beatsPerMinute < 140)&&(irValue > 50000))
        {
          count = count + 1;
          Serial.print("GOT IT");
         // delay(4000);
          prate = prate + beatsPerMinute;
          prate2 = prate + beatAvg;
        }
   }
 digitalWrite(22,HIGH);
 digitalWrite(23,HIGH);
 delay(3000);  
}
void oxygen()
 {
   Serial.println("eNTERTINGg o2 now!!!");
    digitalWrite(26,LOW);
    digitalWrite(27,LOW);
    delay(3000);
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
  }
  digitalWrite(26,HIGH);
  digitalWrite(27,HIGH);
  delay(3000);
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
// Function that sends confirmatory text to physician
void sendSMS1(String deviceID, String pname, String pphone, String patientname, String patientid, String para1, String para2, String para3, String para4, String para5, String para6, String para7, String para8)
{
  /* one = "";
  two = "";
  three = "";
  four = "";
  five = "";
  six = "";
  seven = "";
  eight = ""; */
  
  Serial.println(one);
  delay(100);
  Serial.println(two);
  delay(100);
  Serial.println(three);
  delay(100);
  Serial.println(four);
  delay(100);
  Serial.println(five);
  delay(100);
  Serial.println(six);
  delay(100);
  Serial.println(seven);
  delay(100);
  Serial.println(eight);
  delay(100);
  Serial.println("Entering message function");
   //digitalWrite(en, LOW);
   //delay(3000);
   //digitalWrite(en, HIGH);
   //delay(8000);
  GPSmodule.print("AT+CMGF=1\r"); // AT command to set GPSmodule to SMS mode
  delay(100); // Set delay for 100 ms
  GPSmodule.print("AT+CNMI=2,2,0,0,0\r");// Set module to send SMS data to serial out upon receipt
  delay(100); // Set delay for 100 ms
   
   int lo =  pphone.length();
   String ppho = pphone.substring(1,lo);
   String tnum =  "+234" + ppho;
   Serial.println(tnum);

   String tnumber = "AT+CMGS = \"";
   tnumber = tnumber + tnum + "\"" + "\r";
   Serial.print("Tnumber  - ");
   Serial.println(tnumber);

     //String tnum = "+234" + pphone;
     //String tnumber = "AT+CMGS = \"";
     // tnumber = tnumber + tnum + "\"\r";
  // REPLACE THE X's WITH THE RECIPIENT'S MOBILE NUMBER
  // USE INTERNATIONAL FORMAT CODE FOR MOBILE NUMBERS - for example +371 is international code for Latvia
  GPSmodule.print(tnumber);
  // GPSmodule.print("AT+CMGS = \"+2349018410510\"\r");
  delay(100);
  // Send the SMS
  GPSmodule.println(deviceID);
  //GPSmodule.println("Volume Measured!");
  delay(100);
   //GPSmodule.println("CONFIRMATORY TEXT");
   //delay(100);
   //GPSmodule.print("Name - ");
  delay(100);
  GPSmodule.println(pname);
  delay(100);
  // GPSmodule.print("Phone Number - ");
  delay(100);
  GPSmodule.println(pphone);
  delay(100);
  // GPSmodule.print("Patient - ");
  delay(100);
  GPSmodule.println(patientname);
  delay(100);
   //GPSmodule.print("Patient ID - ");
  delay(100);
  GPSmodule.println(patientid);
  delay(100); 
  //GPSmodule.println("Physiological Parameters:  ");
  //delay(100);   
  GPSmodule.print(para1);
  delay(100); 
  GPSmodule.print(" ");
  delay(100);
  GPSmodule.println(para2);
  delay(100);
  GPSmodule.print(" ");
  delay(100);
  GPSmodule.println(para3);
  delay(100);
  GPSmodule.print(" ");
  delay(100);
  GPSmodule.println(para4);
  delay(100);
  GPSmodule.print(" ");
  delay(100);
  GPSmodule.println(para5);
  delay(100);
  GPSmodule.print(" ");
  delay(100);
  GPSmodule.println(para6);
  delay(100);
  GPSmodule.print(" ");
  delay(100);
  GPSmodule.println(para7);
  delay(100);
  GPSmodule.print(" ");
  delay(100);
  GPSmodule.println(para8);
  delay(100);
  /*

  GPSmodule.println(three);
  delay(200);
  GPSmodule.println(four);
  delay(200);
  GPSmodule.println(five);
  delay(200);
  GPSmodule.println(six);
  delay(200);
  GPSmodule.println(seven);
  delay(200);
  GPSmodule.println(eight);
  delay(200);
  */
  // End AT command with a ^Z, ASCII code 26
  GPSmodule.println((char)26);
  delay(200);
  GPSmodule.println();
  // Give module time to send SMS
  delay(9000);
  Serial.println("Leaving message function");
  delay(2000);
  
}
// Function that sends RPM text to physician
void sendSMS2(String deviceID, String pphone, String patientid, float para1, float para2, float para3, float para4, String para5, float para6, float para7, float para8)
{
  /* one = "";
  two = "";
  three = "";
  four = "";
  five = "";
  six = "";
  seven = "";
  eight = ""; */
  Serial.println(pphone);
  delay(100);
  
  Serial.println(para1);
  delay(100);
  Serial.println(para2);
  delay(100);
  Serial.println(para3);
  delay(100);
  Serial.println(para4);
  delay(100);
  Serial.println(para5);
  delay(100);
  Serial.println(para6);
  delay(100);
  Serial.println(para7);
  delay(100);
  Serial.println(para8);
  delay(100);
  Serial.println("Entering message function");
   //digitalWrite(en, LOW);
   //delay(3000);
   //digitalWrite(en, HIGH);
   //delay(8000);
  GPSmodule.print("AT+CMGF=1\r"); // AT command to set GPSmodule to SMS mode
  delay(100); // Set delay for 100 ms
  GPSmodule.print("AT+CNMI=2,2,0,0,0\r");// Set module to send SMS data to serial out upon receipt
  delay(100); // Set delay for 100 ms

   int lo =  pphone.length();
   String ppho = pphone.substring(1,lo-1);
   String tnum =  "+234" + ppho;
   Serial.println(tnum);

    String tnumber = "AT+CMGS = \"";
    tnumber = tnumber + tnum + "\"" + "\r";
    Serial.print("Tnumber  - ");
    Serial.println(tnumber);
  // REPLACE THE X's WITH THE RECIPIENT'S MOBILE NUMBER
  // USE INTERNATIONAL FORMAT CODE FOR MOBILE NUMBERS - for example +371 is international code for Latvia
  
   GPSmodule.print(tnumber);
  //GPSmodule.print("AT+CMGS = \"+2349018410510\"\r");
  delay(100);
  // Send the SMS
  GPSmodule.println(deviceID);
  //GPSmodule.println("Volume Measured!");
  delay(100);
  GPSmodule.print("Patient ID - ");
  delay(100);
  GPSmodule.print(patientid);
  delay(100);
   //GPSmodule.println("CONFIRMATORY TEXT");
   //delay(100);
   //GPSmodule.print("Name - ");
   //check if temperature was measured. If so, send data to physician
   if (para1 != 0)
   {
      Serial.print("Got Heart Rate!");
      GPSmodule.print(para1);
      delay(100);
      GPSmodule.println("bpm");
      delay(100);
   }
   if (para2 != 0)
   {
      Serial.print("Got pulse");
      GPSmodule.print(para2);
      delay(100);
      GPSmodule.println("bpm");
      delay(100);
   }
   if (para3 != 0)
   {
      Serial.print("Got spO2");
      GPSmodule.print(para3);
      delay(100);
      GPSmodule.println("%");
      delay(100);
   }
   if (para4 != 0)
   {
       Serial.print("Got Blood Sugar!");
      GPSmodule.print(para4);
      delay(100);
      GPSmodule.println("mmolL");
      delay(100);
   }
   if (para5 != 0)
   {
      Serial.print("Got ECG");
      GPSmodule.print(para3);
      delay(100);
   }

   if (para8 != 0)
   {
      Serial.print("Got Body Temperature");
      GPSmodule.print(para8);
      delay(100);
      GPSmodule.print((char)247);
      delay(100);
      GPSmodule.println("C");
      delay(100);
  }

  //GPSmodule.println("Physiological Parameters:  ");
  //delay(100);  
  /* 
  GPSmodule.print(para1);
  delay(100); 
  GPSmodule.print(" ");
  delay(100);
  GPSmodule.println(para2);
  delay(100);
  GPSmodule.print(" ");
  delay(100);
  GPSmodule.println(para3);
  delay(100);
  GPSmodule.print(" ");
  delay(100);
  GPSmodule.println(para4);
  delay(100);
  GPSmodule.print(" ");
  delay(100);
  GPSmodule.println(para5);
  delay(100);
  GPSmodule.print(" ");
  delay(100);
  GPSmodule.println(para6);
  delay(100);
  GPSmodule.print(" ");
  delay(100);
  GPSmodule.println(para7);
  delay(100);
  GPSmodule.print(" ");
  delay(100);
  GPSmodule.println(para8);
  delay(100);
  

  GPSmodule.println(three);
  delay(200);
  GPSmodule.println(four);
  delay(200);
  GPSmodule.println(five);
  delay(200);
  GPSmodule.println(six);
  delay(200);
  GPSmodule.println(seven);
  delay(200);
  GPSmodule.println(eight);
  delay(200);
  */
  // End AT command with a ^Z, ASCII code 26
  GPSmodule.println((char)26);
  delay(200);
  GPSmodule.println();
  // Give module time to send SMS
  delay(9000);
  Serial.println("Leaving message function");
  delay(2000);
  Serial.println(tnumber);
}
