
#include <Wire.h>
#include <ADXL345.h>
#include "MC20_Arduino_Interface.h"
#include "MC20_Common.h"
#include "MC20_GNSS.h"
#include <Adafruit_NeoPixel.h>

#define RGBPIN       10
#define LED_NUM      1
#define GrovePowerPin   12
#define GroveButton  2

// state machine
#define LED_ON       1

unsigned long time_last=0;
unsigned long time_now=0;
int led_state=0;
// SMS
char phone[32];
char dateTime[32];
char buffer[64];
char *s = NULL;
int inComing = 0;
char mymessage[128];


GPSTracker gpsTracker = GPSTracker();
GNSS gnss = GNSS();

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(LED_NUM, RGBPIN, NEO_GRB + NEO_KHZ800);

ADXL345 adxl; //variable adxl is an instance of the ADXL345 library


void stateMachine();
bool is_where_message();
void send_message(char *_phone, char *_message);
void show_led();
bool is_dropping();
bool is_button_press();
void adxl_setting();
void get_coordinate(char *_mymessage);


void setup(){
  SerialUSB.begin(115200);
  pinMode(GrovePowerPin, OUTPUT);
  // write high to grove power pin to enable all the Grove ports,
  // or only Grove D2 port is usable. 
  digitalWrite(GrovePowerPin, HIGH);

  gpsTracker.Power_On();
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
  stateMachine();
  
}


void stateMachine() {
  // 呼吸灯
  if (1 == LED_ON) {
    show_led();
  }
  // 掉落检测
  if (is_dropping()) {
    SerialUSB.println("dropping");
    get_coordinate(mymessage);
    // send_message("13512399523", mymessage);
    MC20_clean_buffer(mymessage, 128);
    led_state = 2;
  }
  // 按下按钮后发送坐标
  if (is_button_press()) {
    get_coordinate(mymessage);
    // send_message("13512399523", mymessage);
    MC20_clean_buffer(mymessage, 128);
    led_state=3;
  }
  // 收到where短信后发送坐标
  if (MC20_check_readable()) {
    led_state=3;
    SerialUSB.println("got message");
    inComing = 1;
  }
  if (1 == inComing) {
    if(is_where_message()) {
      get_coordinate(mymessage);
      // send_message(phone, mymessage);
      MC20_clean_buffer(mymessage, 128);
    }
  }
}

void get_coordinate(char *_mymessage) {
    char *buf;
    gnss.getCoordinate();
    SerialUSB.println(gnss.longitude, 6);
    strcpy(_mymessage, "My coordinate is: longitude");
    // strcpy((_mymessage+strlen(_mymessage)), buf = gcvt(gnss.longitude, 6, buf));
    strcpy((_mymessage+strlen(_mymessage)), "12341234");
    strcpy((_mymessage+strlen(_mymessage)), " latitude");
    // strcpy((_mymessage+strlen(_mymessage)), buf = gcvt(gnss.latitude, 6, buf));
    SerialUSB.println(_mymessage);  

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
          SerialUSB.println("got where");
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
}

bool is_button_press() {
  if (digitalRead(GroveButton)) {
    for (int j=0;j<5;j++) {
     if (digitalRead(GroveButton) == 0) return false;
     delay(10);
    }
    return true;
  }
}

void show_led() {
  time_now = millis();
  if ((time_now > time_last) && (time_now - time_last > 1000)) {
    time_last = time_now;
    if (led_state == 1) {
      pixels.setPixelColor(0, pixels.Color(0,0,100)); // Moderately bright blue color.
      pixels.show(); // This sends the updated pixel color to the hardware.
      led_state = 0;
      return;
    }  
    if (led_state == 0) {
      pixels.setPixelColor(0, pixels.Color(0,0,0)); // Moderately bright blue color.
      pixels.show(); // This sends the updated pixel color to the hardware.
      led_state = 1;
      return;
    }
    if (led_state == 2) {
      pixels.setPixelColor(0, pixels.Color(0,100,100)); // Moderately bright blue color.
      pixels.show(); // This sends the updated pixel color to the hardware.
      time_last += 5000;
      led_state = 0;
      return;
    }
    if (led_state == 3) {
      pixels.setPixelColor(0, pixels.Color(0,100,0)); // Moderately bright blue color.
      pixels.show(); // This sends the updated pixel color to the hardware.
      time_last += 3000;
      led_state = 0;
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

