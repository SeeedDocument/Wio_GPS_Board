---
title: WIO GPS Board
category: Arduino
bzurl:
oldwikiname:
prodimagename: 
surveyurl:
sku:
---

![](https://github.com/SeeedDocument/GPS_Tracker/blob/master/images/GPSTracker.jpg?raw=true)

What is WIO GPS Board? WIO GPS Board is an Arduino compatible development board that helps you to track any moving things on the planet. By integrating GSM&GPRS as well as GPS&BeiDou in one board, it provides an all-in-one solution for your out-door IoT project.

If you are following the latest news about Low-Power Wide-Area Network(LPWAN), you may know that 2016 is a special year because there are new technology like LoRa and Sigfox came out, which remarkably pushed the development of WAN. The WIO GPS Board has similarities as LoRa and Sigfox but also quite different. It is more suitable for out-door moving tracking and situations that has high frequency customer interaction and medium volume of data, for example sharing bicycle, logistics location, pet tracking.

Comparing to traditional GPS tracking solution, Seeed’s WIO GPS Board is much more easier to use and customize for the fast changing IOT market. As a user-friendly Grove development board, the look of WIO GPS Board may remind you of Wio Link, another started product of Seeed. So you can also find 6 grove connector on the board. As an Arduino compatible board, users can use the WIO GPS Board with Arduino IDE directly.

Why do we call the WIO GPS Board a solution, because it is really not just a board. At SeeedStudio, we have the ability to help customer customize the WIO GPS Board for their own project and finish the project all the way to mass production. As WIO GPS Board uses standard module that is mass production, we are ready to mass production even when customer is at their prototyping stage.

We have prepared everything, the last thing we need is your creative projects, so if you are making an out-door IOT tracking project, please don’t hesitate to use the WIO GPS Board!


## Features

* Multi-GNSS engine for combined GPS and BeiDou, higher precision location
* EPO™ Technology, providing predicted Extended Prediction Orbit to speed up TTFF without need of extra server
* Based on EPO™ data, QuecFastFix™ Online function further reduces TTFF in cold start, making cold start TTFF comparable to that in hot start
* GLP(GNSS Low Power) Mode, 40% power consumption for normal mode and slightly lower precision
* Period Mode, also reduce power consumption by controlling the sleep time
* Always Locate™ Technology, an intelligent algorithm for power saving
* Easy™(Embedded Assist System) Technology, self-generated orbit prediction for instant positioning fix, reducing the Time To First Fix(TTFF)
* LOCUS™ Technology, innate navigation data logger solution with no need for host and external flash
* AT Command: GSM 07.07, 07.05 and enhanced AT Command
* Bluetooth 3.0 with SPP and HFP-AG
* 6 Grove Connectors
* Nano SIM and TF card 2 in 1 socket
* Arduino IDE compatible
* Low power and small size


### Quectel MC20 Module

* Ultra compact size: 18.7 × 16.0 × 2.1mm
* Multi navigation constellation: GPS/ BeiDou/ QZSS
* GNSS receiver channels: 99 acquisition/ 33 tracking channels
* Powerful AGPS functions: Autonomous AGPS EASY™/ Offline AGPS EPO™/ Online AGPS QuecFastFix
* Built-in LNA for better GNSS sensitivity (-167dBm@Tracking): able to use passive GNSS antenna without the need of any extra LNA
* Enhanced GNSS features: SDK command/ AIC/ LOCUS/ GLP
* GSM quad-band: 850/900/1800/1900MHz
* Multi internet protocols: TCP/ UDP/ PPP/ HTTP/ FTP/ SSL
* Support Voice, SMS, QuecFOTATM, DSSS, OpenCPU
* Support Bluetooth V3.0: SPP & HFP-AG profiles

!!!Warning
    Unlike most Arduino & Genuino boards, the WIO GPS Board runs at 3.3V. The maximum voltage that the I/O pins can tolerate is 3.3V. Applying voltages higher than 3.3V to any I/O pin could damage the board.

## Specification

| Item            | Function                              | Value                                                                      |
| --------------  | ------------------------------------- | -------                                                                    |
| Microcontroller | Processor                             | ATSAMD21G18A-MU, ARM Cortex-M0+, 48MHz                                     |
|                 | Flash Memory                          | 256KB                                                                      |
|                 | SRAM                                  | 32KB                                                                       |
|                 | Operating Voltage                     | 3.3V                                                                       |
|                 | DC Current per I/O Pin                | 7 mA                                                                       |
|                 | Clock Speed                           | 48 MHz                                                                     |
| GSM/GPRS        | GSM                                   | 850/900/1800/1900MHz, Class 4 (2W @850/900MHz), Class 1 (1W @1800/1900MHz) |
|                 |                                       | AT Command: GSM 07.07, 07.05 and enhanced AT Command                       |
|                 |                                       | Low Power Consumption: 1.2mA@DRX=5                                         |
|                 | GPRS                                  | GPRS Multi-slot Class 12: Down to 85.6kbps, Up to 85.6kbps                 |
|                 |                                       | Protocol: TCP/UDP/FTP/HTTP/PPP/SSL                                         |
|                 | SMS                                   | Peer to Peer Message, SMS broadcast, Text and PDU mode                     |
|                 | Audio                                 | Echo cancellation, Noise elimination                                       |
|                 | Bluetooth                             | Bluetooth 3.0: SPP, HFP-AG                                                 |
| GNSS            | System                                | GPS L1 1575.42MHz, BeiDou B1 1561.10MHz                                    |
|                 | Precision                             | <2.5 m CEP                                                                 |
|                 | Technology                            | EASY/LOCUS/AlwaysLocate/EPO/GLP/AIC                                        |
| Peripheral      | Grove                                 | 2 x Grove Digital Port                                                     |
|                 |                                       | 2 x Analog Port                                                            |
|                 |                                       | 1 x UART                                                                   |
|                 |                                       | 1 x I2C                                                                    |
|                 | Antenna                               | GSM Antenna                                                                |
|                 |                                       | Bluetooth Antenna                                                          |
|                 |                                       | GNSS Antenna                                                               |
|                 | Others                                | USB: Power supply and upload program                                       |
|                 |                                       | JST 1.0 connecter for battery                                              |
|                 |                                       | 3.5mm Audio Jack                                                           |
|                 |                                       | GSM Power Button, Reset Button                                             |
|                 |                                       | 1 x User RGB LED SK6812                                                    |
|                 |                                       | Speaker Interface                                                          |
|                 |                                       | Nano SIM and TF card 2 in 1 socket                                         |
| Size            | Length                                | 54.7mm                                                                     |
|                 | Width                                 | 48.2mm                                                                     |
|                 | Weight                                |                                                                            |


## Application Ideas

* Intelligent Transportation
* Pet Tracker
* Outdoor sports equipments
* Driving recorder
* Wearable device
* Property Security


!!!Tip
    Use Grove modules to expand your application

There are 6 Grove connects on board. If this is your first time to hear about Grove, please put had on [Grove System](http://wiki.seeed.cc/Grove_System/) for more details.
In brief, Groves is hundreds of sensor that in standard style, which is consist of sensors, actuators, displays as well as communication.

## Hardware Overview

![](https://github.com/SeeedDocument/GPS_Tracker/blob/master/images/GPS_Tracker_v1.2_top.png?raw=true)

![](https://github.com/SeeedDocument/GPS_Tracker/blob/master/images/GPS_Tracker_v1.2_bottom.png?raw=true)


!!!Tip
    If you want to use the on-board Grove connector, please use digitalWrite(12, HIGH) to open 3V3_B. Otherwise you can't provide power to Grove modules.

## Pin Map

|Pin Name|External Interrupt|PWM|Analog In|Analog Out|Function|
|--------|-----------|---|---------|----------|--------|
|RX      |YES        |YES|         |          |Grove UART|
|TX      |YES        |YES|         |          |Grove UART|
|D2      |YES        |YES|         |          |Grove Digital|
|D3      |YES        |YES|         |          |Grove Digital|
|D4      |YES        |   |         |          |Grove Digital|
|D5      |YES        |YES|         |          |Grove Digital|
|D7      |YES        |   |         |          |Power of MC20|
|D10     |YES        |YES|         |          |User LED|
|D11     |YES        |YES|         |          |Check Earphone|
|D12     |YES        |YES|         |          |Grove Power Control|
|D13     |YES        |YES|         |          |GSM Power Button|
|SDA     |YES        |   |         |          |Grove I2C|
|SCL     |YES        |   |         |          |Grove I2C|
|A0      |YES        |   | YES     |  YES     |Grove Analog|
|A1      |YES        |   | YES     |  YES     |Grove Analog|
|A2      |YES        |   | YES     |  YES     |Grove Analog|
|A3      |YES        |   | YES     |  YES     |Grove Analog|
|A4      |YES        |   | YES     |          |Check battery|

!!!Note
    All pins can act as Digital Input and Output


## Getting Started - Arduino IDE

!!!Note
    This chapter is based on Win10 and Arduino IDE v1.6.0

First you need to install the latest Arduino IDE, and [ADD Seeeduino LoRa to your Arduino IDE](http://wiki.seeed.cc/Seeed_Arduino_Boards/).

### Install the Driver (For Windows)

When the first time to insert the board, you should get a USB COM device name WIO GPS Board that need to install a driver. Click [here](https://github.com/SeeedDocument/Seeeduino_LoRa/raw/master/res/driver.zip) to download driver for the board.



To make sure the driver was installed successful, open your Device Manager to see if **WIO GPS Board** exists.

### Install the library of WIO GPS Board(For Windows, Linux and Mac)

Click [here](https://github.com/lanselambor/MC20_GPS_Tracker/archive/master.zip) to download the library(zip file) of WIO GPS Board and import the zip library into your Arduino IDE. If you are new to this, please click [here](https://www.arduino.cc/en/Guide/Libraries#toc4) for more information.


### Blink

Different from other boards, there is an User LED SK6812 on WIO GPS Board, which is an intelligent control LED light source and similar to ws2812b. Now we are going to upload our first demo - Blink to WIO GPS Board, which will introduce how to use the User LED.

First, we have to install [Adafruit NeoPixel Library](https://github.com/adafruit/Adafruit_NeoPixel/archive/master.zip)(for ws2812b LED) and import it into Arduino IDE.

Then open your Arduino IDE and click on **File > Examples > MC20\_GPS\_Traker > Blink** to open the sketch or copy the blow code:

```c
#include "MC20_Arduino_Interface.h"
#include <Adafruit_NeoPixel.h>

#define RGBPIN       10
#define LED_NUM      1

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(LED_NUM, RGBPIN, NEO_GRB + NEO_KHZ800);

void setup() {

  pixels.begin(); // This initializes the NeoPixel library.
}

void loop() {

  // For a set of NeoPixels the first NeoPixel is 0, second is 1, all the way up to the count of pixels minus one.
  // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
  pixels.setPixelColor(0, pixels.Color(0,0,100)); // Moderately bright blue color.
  pixels.show(); // This sends the updated pixel color to the hardware.
  delay(1000);   // Wait for 1 second
  pixels.setPixelColor(0, pixels.Color(0,0,0));   // Turn off the led.
  pixels.show(); 
  delay(1000); 
}
```

And Then,

* Click on **Tools > Board > WIO GPS Board**
* Click on **Tools > Port** to select a right port number. (*Don't choose COM1*)


Then click on the **Upload** button on the left-top of Arduino IDE, seconds later the sketch was uploaded successful.


If the uploading is success, you should the some info in red and you will see the User LED, it's blinking in blue.



### Check battery

WIO GPS Board is designed for tracking moving things outdoor. So you can power the board via a 3.7V Lipo battery with JST1.0 connector and check the voltage of your battery easily.

!!!Warning
    Make sure the positive and negative end of your battery is connected right, otherwise the board may be damaged.

Battery status pin had beed connect to A4, which allow you to measure voltage of the battery via coding.

Open your Arduino IDE and click on **File > Examples > MC20\_GPS\_Traker > Check_Battery** to open the sketch or copy the blow code:

```c
#include "MC20_Arduino_Interface.h"

const int pin_battery_voltage = A4;

void setup() {
    SerialUSB.begin(115200);
}

void loop() {

    int a = analogRead(pin_battery_voltage);
    float v = a/1023.0*3.3*2.0;        // there's an 10M and 10M resistor divider
    SerialUSB.print("The voltage of battery is ");
    SerialUSB.print(v, 2);
    SerialUSB.println(" V");
    delay(1000);
}
```

### Use Grove modules to expand your application

To use 6 Grove connectors on WIO GPS Board, we have to use `digitalWrite(12, HIGH)` to open 3V3\_B to power the Grove modules(D12 defaults to LOW in order to reduce consumption).

The following code shows how to use Analog and Digital Grove modules:

Open your Arduino IDE and click on **File > Examples > MC20\_GPS\_Traker > Button\_LightSensor** to open the sketch or copy the blow code:

```c
#include "MC20_Arduino_Interface.h"

#define GrovePowerPin   12
#define LightSensorPin  A0
#define ButtonPin       2


void setup() {
  SerialUSB.begin(115200);
  pinMode(ButtonPin, INPUT);
  pinMode(GrovePowerPin, OUTPUT);
  // write high to grove power pin to enable all the Grove ports,
  // or only Grove D2 port is usable. 
  digitalWrite(GrovePowerPin, HIGH);   
}

void loop() {
    // print analog data when the button is pressed
    if (digitalRead(ButtonPin)) {
        SerialUSB.print("The value of light sensor:");
        SerialUSB.println(analogRead(LightSensorPin));
    }
    delay(500);
}   
```


### GNSS

MC20 module provides the AT Commands Set for MCU to communicate with its GSM/GPRS, GNSS and Bluetooth modules.

And we have installed a well written library for the WIO GPS Board, for simple applications you even don't need to know much about how to use the AT commands, which is complex and hard to read.

The following example shows how to read longitude and latitude and print them out in Serial Monitor. Please note that WIO GPS Board should be placed outdoor to get GPS signal.

Open your Arduino IDE and click on **File > Examples > MC20\_GPS\_Traker > MC20\_GNSS > GNSS\_Show\_Coordinate** to open the sketch or copy the blow code:

```c
#include "MC20_Common.h"
#include "MC20_Arduino_Interface.h"
#include "MC20_GNSS.h"


GNSS gnss = GNSS();

void setup() {
  SerialUSB.begin(115200);
  // while(!SerialUSB);

  gnss.Power_On();
  SerialUSB.println("\n\rPower On!");

  while(!gnss.open_GNSS()){
    delay(1000);
  }

  SerialUSB.println("Open GNSS OK.");
}

void loop() {
  // gnss.dataFlowMode();
  
  if(gnss.getCoordinate()){
    SerialUSB.print("GNSS: ");
    SerialUSB.print(gnss.longitude, 6); 
    SerialUSB.print(",");
    SerialUSB.println(gnss.latitude, 6);  
  } else{
    SerialUSB.println("Error!");
  }
  
  delay(1000);
}
```


### SMS Message Read

The following example shows how to read sms messages on WIO GPS Board. Nano SIM card is needed in this example.

Open your Arduino IDE and click on **File > Examples > MC20\_GPS\_Traker > MC20\_SMSRead** to open the sketch or copy the blow code:


```c
#include "MC20_Common.h"
#include "MC20_Arduino_Interface.h"

#define RGBPIN 10

char phone[32];
char dateTime[32];
char buffer[64];
int i = 0;
char *s = NULL;
int inComing = 0;

GPSTracker gpsTracker = GPSTracker();

void setup() {
  // MC20_init();
  pinMode(RGBPIN, OUTPUT);
  digitalWrite(RGBPIN, LOW);
  SerialUSB.begin(115200);
  // while(!SerialUSB);

  gpsTracker.Power_On();
  SerialUSB.println("Power On!");

  // gpsTracker.deleteSMS(1);

}

void loop() {

  if(MC20_check_readable()){
    inComing = 1;
  }else{
    delay(1000);
  }

  if(1 == inComing){
    MC20_read_buffer(buffer, 64);
    SerialUSB.println(buffer);

    if(NULL != (s = strstr(buffer,"+CMTI: \"SM\""))) { //SMS: $$+CMTI: "SM",24$$
        char message[128];
        int messageIndex = atoi(s+12);
        gpsTracker.readSMS(messageIndex, message,128);
        SerialUSB.print("Recv SMS: ");
        SerialUSB.println(message);
     }
     MC20_clean_buffer(buffer,64);  
     inComing = 0;
  }
  
}
```

### Bluetooth Connection

It is also very easy to connect to your Bluetooth device on WIO GPS Board. Open your Arduino IDE and click on **File > Examples > MC20\_GPS\_Traker > MC20\_BlueTooth > BT\_FastConnect** to open the sketch or copy the blow code.

Then change the deviceName of your Bluetooth device, WIO GPS Board will connect it automatically.

```c
#include "MC20_Common.h"
#include "MC20_BT.h"

// GPSTracker gpsTracker = GPSTracker();
BlueTooth bt = BlueTooth();
int bt_state = -1;
char *deviceName = "N-612";


void setup() {
  SerialUSB.begin(115200);
  while(!SerialUSB);

  bt.Power_On();
  SerialUSB.println("\n\rMC20 power On!");
  bt.BTPowerOn();
  SerialUSB.println("\n\rBT power On!");

  while(IDLE != (bt_state = bt.getBTState())){
    SerialUSB.print("State: ");
    SerialUSB.println(bt_state);
    delay(1000);
  }

  bt.BTFastConnect(deviceName, HFG_PROFILE);
}

void loop() {
  /* Debug */
  if(SerialUSB.available()){
    serialMC20.write(SerialUSB.read());
  }
  if(serialMC20.available()){     
    SerialUSB.write(serialMC20.read()); 
  }
}
```

## Resources

* [Schematics in Eagle](https://github.com/SeeedDocument/GPS_Tracker/blob/master/resources/GPS_Tracker_Eagle.zip)
* [Sketchup file(3D)](https://github.com/SeeedDocument/GPS_Tracker/blob/master/resources/GPS_Tracker.skp)
* [GSM Bluetooth Application Note](https://github.com/SeeedDocument/GPS_Tracker/blob/master/resources/Quectel_GSM_BT_Application_Note_V1.2(C).pdf)
* [GSM File AT Commands Manual](https://github.com/SeeedDocument/GPS_Tracker/blob/master/resources/Quectel_GSM_FILE_AT_Commands_Manual_V1.5.pdf)
* [MC20 AT Commands Manual](https://github.com/SeeedDocument/GPS_Tracker/blob/master/resources/Quectel_MC20_AT_Commands_Manual_V1.1.pdf)
* [MC20 GNSS AGPS ApplicationGuide](https://github.com/SeeedDocument/GPS_Tracker/blob/master/resources/Quectel_MC20_GNSS_AGPS_ApplicationGuide_V1.1.pdf)
* [MC20 GNSS AT Commands Manual](https://github.com/SeeedDocument/GPS_Tracker/blob/master/resources/Quectel_MC20_GNSS_AT_Commands_Manual_V1.1.pdf)
* [MC20 GNSS Protocol Specification](https://github.com/SeeedDocument/GPS_Tracker/blob/master/resources/Quectel_MC20_GNSS_Protocol_Specification_V1.0.pdf)
