
#include <Wire.h>
#include <ADXL345.h>
#include "MC20_Arduino_Interface.h"
#include "MC20_Common.h"
#include "MC20_GNSS.h"
#include <Adafruit_NeoPixel.h>


#define RGB_LED_NUM           1
#define GROVE_BUTTON_PIN      2
#define RGB_LED_PIN           10
#define GROVE_POWER_PIN       12



unsigned long lastRecordTime = 0;
unsigned long nowRecordTime = 0;
int ledState = 0;
// SMS
char phone[32];
// replace "***********" with the phone number of your emergency contact
char preSetPhoneNum[12] = "***********";   
char dateTime[32];
char buffer[64];
char *s = NULL;
int inComing = 0;
char mymessage[128];
int help = 0;


GPSTracker gpsTracker = GPSTracker();
GNSS gnss = GNSS();

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(RGB_LED_NUM, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);

ADXL345 adxl; //variable adxl is an instance of the ADXL345 library

bool is_where_message();
void send_message(char *_phone, char *_message);
void show_led();
bool is_dropping();
bool is_button_press();
void adxl_setting();
void get_coordinate(char *_message);


void setup(){
  SerialUSB.begin(115200);
  // write high to grove power pin to enable all the Grove ports,
  // or only Grove D2 port is usable. 
  pinMode(GROVE_POWER_PIN, OUTPUT);
  digitalWrite(GROVE_POWER_PIN, HIGH);

  gnss.Power_On();

  SerialUSB.println("Power On!");

  adxl.powerOn();
  adxl_setting();

  pixels.begin(); 

  if(!gpsTracker.waitForNetworkRegister())
  {
    SerialUSB.println("Network error!");
    return;
  }
  while(!gnss.open_GNSS(EPO_RL_MODE)){
    delay(1000);
  }

  SerialUSB.println("Open GNSS OK.");
}

void loop(){
  show_led();

  if (is_dropping()) {
    SerialUSB.println("dropping");
    get_coordinate(mymessage);
    send_message(preSetPhoneNum, mymessage);
    MC20_clean_buffer(mymessage, 128);
    ledState = 2;
    help = 1;
  }

  if (is_button_press()) {
    SerialUSB.println("pressing");
    get_coordinate(mymessage);
    send_message(preSetPhoneNum, mymessage);
    MC20_clean_buffer(mymessage, 128);
    ledState=3;
  }

  if (1 == inComing) {
    if(is_where_message()) {
      get_coordinate(mymessage);
      send_message(phone, mymessage);
      MC20_clean_buffer(mymessage, 128);
    }
  }
  if (MC20_check_readable()) {
    ledState=3;
    inComing = 1;
  }
}


void get_coordinate(char *_message) {
    gnss.getCoordinate();
    if (help == 0) sprintf(_message, "My coordinates are: Latitude %s, Longitude %s", gnss.str_latitude, gnss.str_longitude);
    if (help == 1) {
      sprintf(_message, "Help! I am in danger! My coordinates are: Latitude %s, Longitude %s", gnss.str_latitude, gnss.str_longitude);
      help = 0;
    }
    SerialUSB.println(_message); 
}


bool is_where_message() {
    MC20_read_buffer(buffer, 64);
    SerialUSB.println(buffer);

    if(NULL != (s = strstr(buffer,"+CMTI: \"SM\""))) { //SMS: $$+CMTI: "SM",24$$
        char message[128];
        int messageIndex = atoi(s+12);
        gpsTracker.readSMS(messageIndex, message, 128, phone, dateTime);
        SerialUSB.print("Recv SMS: ");
        SerialUSB.println(message);
        SerialUSB.println(phone);
        if (NULL != (s = strstr(message, "where"))) {
          MC20_clean_buffer(buffer,64); 
          inComing = 0;
          return true;
        }
     }
     MC20_clean_buffer(buffer,64);  
     inComing = 0;
     return false;
}

void send_message(char *_phone, char *_message) {
  gpsTracker.sendSMS(_phone, _message);
  return;
}

bool is_button_press() {
  if (digitalRead(GROVE_BUTTON_PIN)) {
    for (int j=0;j<5;j++) {
     if (digitalRead(GROVE_BUTTON_PIN) == 0) return false;
     delay(10);
    }
    return true;
  }
}

void show_led() {
  nowRecordTime = millis();
  if ((nowRecordTime > lastRecordTime) && (nowRecordTime - lastRecordTime > 300)) {
    lastRecordTime = nowRecordTime + 1000;
    // ledState = 0, led turns dark;
    // ledState = 1, led turns blue, which means idle;
    // ledState = 2, led turns cyan, which means detected dropping;
    // ledState = 3, led turns green, which means button is pressed
    // ledState = 4, led turns red, which means got message
    if (ledState == 0) {
      pixels.setPixelColor(0, pixels.Color(0,0,0)); // Moderately bright blue color.
      pixels.show(); // This sends the updated pixel color to the hardware.
      ledState = 1;
      return;
    }
    if (ledState == 1) {
      pixels.setPixelColor(0, pixels.Color(0,0,255)); // Moderately bright blue color.
      pixels.show(); // This sends the updated pixel color to the hardware.
      ledState = 0;
      return;
    }  

    if (ledState == 2) {
      pixels.setPixelColor(0, pixels.Color(0,255,255)); 
      pixels.show(); 
      lastRecordTime += 1000;
      ledState = 0;
      return;
    }
    if (ledState == 3) {
      pixels.setPixelColor(0, pixels.Color(0,255,0)); 
      pixels.show(); 
      lastRecordTime += 1000;
      ledState = 0;
      return;
    }
  }
}


bool is_dropping() {
  double xyz[3];
  double ax,ay,az,a;

  for (int i=0;i<20;i++) {
    adxl.getAcceleration(xyz);
    ax = xyz[0];
    ay = xyz[1];
    az = xyz[2];
    a = ax*ax + ay*ay +az*az;

    if (a > 0.20) {
      return false;
    }
    delay(15);
  }
  return true;
}



void adxl_setting() {
    //set activity/ inactivity thresholds (0-255)
  adxl.setActivityThreshold(75); //62.5mg per increment
  adxl.setInactivityThreshold(75); //62.5mg per increment
  adxl.setTimeInactivity(10); // how many seconds of no activity is inactive?
 
  //look of activity movement on this axes - 1 == on; 0 == off 
  adxl.setActivityX(1);
  adxl.setActivityY(1);
  adxl.setActivityZ(1);
 
  //look of inactivity movement on this axes - 1 == on; 0 == off
  adxl.setInactivityX(1);
  adxl.setInactivityY(1);
  adxl.setInactivityZ(1);
 
  //look of tap movement on this axes - 1 == on; 0 == off
  adxl.setTapDetectionOnX(0);
  adxl.setTapDetectionOnY(0);
  adxl.setTapDetectionOnZ(1);
 
  //set values for what is a tap, and what is a double tap (0-255)
  adxl.setTapThreshold(50); //62.5mg per increment
  adxl.setTapDuration(15); //625us per increment
  adxl.setDoubleTapLatency(80); //1.25ms per increment
  adxl.setDoubleTapWindow(200); //1.25ms per increment
 
  //set values for what is considered freefall (0-255)
  adxl.setFreeFallThreshold(7); //(5 - 9) recommended - 62.5mg per increment
  adxl.setFreeFallDuration(45); //(20 - 70) recommended - 5ms per increment
 
  //setting all interrupts to take place on int pin 1
  //I had issues with int pin 2, was unable to reset it
  adxl.setInterruptMapping( ADXL345_INT_SINGLE_TAP_BIT,   ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_DOUBLE_TAP_BIT,   ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_FREE_FALL_BIT,    ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_ACTIVITY_BIT,     ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_INACTIVITY_BIT,   ADXL345_INT1_PIN );
 
  //register interrupt actions - 1 == on; 0 == off  
  adxl.setInterrupt( ADXL345_INT_SINGLE_TAP_BIT, 1);
  adxl.setInterrupt( ADXL345_INT_DOUBLE_TAP_BIT, 1);
  adxl.setInterrupt( ADXL345_INT_FREE_FALL_BIT,  1);
  adxl.setInterrupt( ADXL345_INT_ACTIVITY_BIT,   1);
  adxl.setInterrupt( ADXL345_INT_INACTIVITY_BIT, 1);
}

