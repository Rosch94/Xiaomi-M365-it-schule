//#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"
#include "System5x7mod.h"
#include <EEPROM.h>

//struct for recieve buffer
typedef struct
{
  uint8_t length;
  uint8_t buffer[36]; //0x24 or 36(DEC) maximum size of one packet inclunding cmd, arg, payload... more is not needed
} queuebuffer;

//reciever and sender queue
queuebuffer receiverbuffer;
queuebuffer sendbuffer;

typedef struct
{
  uint8_t throttle;
  uint8_t brake;
} throttlebrake;

//contain brake and throttlevalue
throttlebrake throttlebrakeValues;

//struct which contains the main information which is decoded
typedef struct
{
  uint16_t batterylevel;
  int16_t speed;         // /1000 in km/h, negative value = backwards...
  uint32_t odometer;     // /1000 in km
  uint16_t tripdistance; // /100
  //uint16_t ontime;     //power on time  not decoded jet
  uint16_t temperature; // /10 = temp in °C
  uint8_t headlightstate;
} maininfo;

maininfo mainformation;

//recieve buffer
uint8_t bufferreadindex = 0;
uint8_t bufferreceiverstate = 0;

uint16_t checksumcalculation = 0;
uint8_t checksum1 = 0;
uint8_t checksum2 = 0;

//M365 - serial sender
uint8_t sendecounter = 0;

#define bAddr 0
#define bcmd 1
#define bArg 2
#define bPayload 3

#define WKERSWEAK 1
#define WKERSMEDIUM 2
#define WKERSSTRONG 3
#define WTAILLIGHTOFF 4
#define WBACKLIGHTON 5
#define WCRUISEON 6
#define WCRUISEOFF 7

//flag if a package is prepared and ready to send -> 1 if prepared
uint8_t packagePreparedFlag = 0;
//flag if a package is copmletly recieved so we can renew the display
bool renewdispalyFlag;

//---------------------------------------------------------------------------
//packet wrapper

#define HEADER55 0x55
#define HEADERAA 0xAA
#define REQUESTTOESC 0x20
#define REPLYFROMESC 0x23
#define REQUESTTOBLE 0x21
#define packetunkown 0x02
#define CMDWRITE 0x03

const uint8_t READESCDATA[] = {0x06, REQUESTTOESC, 0x61};

//https://github.com/etransport/ninebot-docs/wiki/M365ESC
//R stands for Register look link above for explanation
//BACKLIGHT control
#define RBACKLIGHT 0x7D
#define BACKLIGHTON 0x02

//CRUISE control
#define RCRUISE 0x7C
#define CRUISEON 0x01

//OFF used for controll of BACKLIGHT and CRUISE
#define OFF 0x00

//KERS control
#define RKERS 0x7B
#define KERSWEAK 0x00
#define KERSMEDIUM 0x01
#define KERSSTRONG 0x02

//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
//display

//object of display
SSD1306AsciiAvrI2c ic2display;

//for display initialization
#define I2C_ADDRESS 0x3C

//main menu
static const int8_t d_main_battery[] PROGMEM = "Battery %";
static const int8_t d_main_temperature[] PROGMEM = "Temp. C.";
static const int8_t d_main_headtlight[] PROGMEM = "Headlight";

//more information menu
static const int8_t d_info_odometer[] PROGMEM = "Odometer in km:";
static const int8_t d_info_tripdistance[] PROGMEM = "Trip distance in km:";

//navigation menu
static const int8_t d_navi_batterywarning[] PROGMEM = "Bat. warning:";
static const int8_t d_navi_batteryinformation[] PROGMEM = "More information";
static const int8_t d_navi_kers[] PROGMEM = "KERS:";
static const int8_t d_navi_taillight[] PROGMEM = "Taillight:";
static const int8_t d_navi_cruise[] PROGMEM = "Cruise:";
static const int8_t d_navi_menu_exit[] PROGMEM = "Exit";

//other menu items
static const int8_t d_on[] PROGMEM = " ON";
static const int8_t d_off[] PROGMEM = "OFF";
static const int8_t d_weak[] PROGMEM = "WEAK";
static const int8_t d_middle[] PROGMEM = "MIDDLE";
static const int8_t d_strong[] PROGMEM = "STRONG";
static const int8_t d_line[] PROGMEM = "---------------------";
static const int8_t d_km[] PROGMEM = "km/h";
static const int8_t d_arrow[] PROGMEM = "=>";
static const int8_t d_b[] PROGMEM = "=>B<=";

//display control with brake and throttle
bool batterywarning = false;

//0 = speedmenu, 1 = navigationmenu 2 = mainmenu, 3 = infomenu
//defaul starting with main menu
int8_t display_option = 2;

//controlling menu position to change valuen
uint8_t menuposition = 0;

//to save and load actual config from EEPROM
bool cruise_state = true;
bool taillight_state = false;
uint8_t kers_state = 0; //0 = weak, 1 = medium, 2 = strong

//how often battery warning should appear on the display
uint8_t batterycounter;

//needed for display clear method
uint8_t olddisplayvalue = 0;

//timer to fix fast changing of button values
unsigned long buttontime;

//----------------------------------------------------------------------------

void setup()
{
  
  //second serial for debuggin
  //Serial.begin(115200);
  Serial.begin(115200);

  saveandloadConfig();

  //display configuration
  ic2display.setI2cClock(400000L);
  ic2display.begin(&Adafruit128x64, I2C_ADDRESS);
  ic2display.setFont(System5x7mod);
  ic2display.clear();
}

void displayclear(uint8_t newdisplayvalue)
{

  if (olddisplayvalue != newdisplayvalue)
  {
    ic2display.clear();
    olddisplayvalue = newdisplayvalue;
  }
}

void loop()
{

  recievedata();

  if (packagePreparedFlag == 0)
  {
    preparePacket(1);
  }

  if (renewdispalyFlag == true)
  {
    displaydata();
    renewdispalyFlag = false;
  }
}

void displaydata()
{
  maininfo *ptrmaininformation = &mainformation;
  uint16_t speed = abs(((*ptrmaininformation).speed / 1000)); //speed is positiv

  
  //display speed menu
  if (speed > 1)
  {
    display_option = 0;
  }

  if (speed <= 2)
  {
  int8_t controlbrakevalue = -1;
  int8_t controlthrottlevalue = -1;
    if (millis() > buttontime + 500)
    {
      if (throttlebrakeValues.brake >= 160)
      {
        buttontime = millis();
        controlbrakevalue = 1;
      }
      else if (throttlebrakeValues.brake <= 50)
      {

        controlbrakevalue = -1;
      }
      else
      {
        controlbrakevalue = 0;
      }

      if (throttlebrakeValues.throttle >= 160)
      {
        buttontime = millis();

        controlthrottlevalue = 1;
      }
      else if (throttlebrakeValues.throttle <= 50)
      {

        controlthrottlevalue = -1;
      }
      else
      {
        controlthrottlevalue = 0;
      }
    }

    if ((controlbrakevalue == 1) && (controlthrottlevalue == 1))
    {
      
      menuposition = 0; //menu position to change values
      display_option = 1; //open navigation menu = 1
      ic2display.clear();
    }

    switch (display_option)
    {
    case 1:
      if ((controlthrottlevalue == 1) && (controlbrakevalue == -1))
      {
        switch (menuposition)
        {
        case 0:
          if (batterywarning == true)
          {
            batterywarning = false;
          }
          else
          {
            batterywarning = true;
          }
          EEPROM.update(1, batterywarning);
          ic2display.clear();
          break;
        case 1:
          display_option = 3;
          ic2display.clear();
          break;
        case 2:
          if (cruise_state == true)
          {
            cruise_state = false;
            preparewritePacket(7);
          }
          else
          {
            cruise_state = true;
            preparewritePacket(6);
          }
          EEPROM.update(2, cruise_state);
          ic2display.clear();
          break;
        case 3:
          if (taillight_state == true)
          {
            taillight_state = false;
            preparewritePacket(4); //OFF
          }
          else
          {
            taillight_state = true;
            preparewritePacket(5); //ON
          }
          EEPROM.update(3, taillight_state);
          ic2display.clear();
          break;
        case 4:
          switch (kers_state)
          {
          case 1:
            preparewritePacket(2); //medium
            kers_state = 2;
            break;
          case 2:
            preparewritePacket(3); //strong
            kers_state = 0;
            break;
          default:
            preparewritePacket(1); //weak
            kers_state = 1;
            break;
          }
          EEPROM.update(4, kers_state);
          ic2display.clear();
          break;
        case 5:
          display_option = 2;
          displayclear(1);
          break;
        }
      }
      else
      {
        if ((controlbrakevalue == 1) && (controlthrottlevalue == -1))
        {
          if (menuposition < 5)
          {
            menuposition++;
          }
          else
          {
            menuposition = 0;
          }
        }
      }

      ic2display.set1X();
      ic2display.setCursor(15, 0);

      if (menuposition == 0)
      {
        ic2display.print((const __FlashStringHelper *)d_arrow);
      }
      else
      {
        ic2display.print("  ");
      }
      ic2display.print((const __FlashStringHelper *)d_navi_batterywarning);
      if (batterywarning == false)
      {
        ic2display.print((const __FlashStringHelper *)d_off);
      }
      else
      {
        ic2display.print((const __FlashStringHelper *)d_on);
      }
      ic2display.setCursor(15, 1);
      if (menuposition == 1)
      {
        ic2display.print((const __FlashStringHelper *)d_arrow);
      }
      else
      {
        ic2display.print("  ");
      }
      ic2display.print((const __FlashStringHelper *)d_navi_batteryinformation);

      ic2display.setCursor(15, 2);
      if (menuposition == 2)
      {
        ic2display.print((const __FlashStringHelper *)d_arrow);
      }
      else
      {
        ic2display.print("  ");
      }
      ic2display.print((const __FlashStringHelper *)d_navi_cruise);
      if (cruise_state == true)
      {
        ic2display.print((const __FlashStringHelper *)d_on);
      }
      else
      {
        ic2display.print((const __FlashStringHelper *)d_off);
      }
      ic2display.setCursor(15, 3);
      if (menuposition == 3)
      {
        ic2display.print((const __FlashStringHelper *)d_arrow);
      }
      else
      {
        ic2display.print("  ");
      }
      ic2display.print((const __FlashStringHelper *)d_navi_taillight);
      if (taillight_state == true)
      {
        ic2display.print((const __FlashStringHelper *)d_on);
      }
      else
      {
        ic2display.print((const __FlashStringHelper *)d_off);
      }
      ic2display.setCursor(15, 4);
      if (menuposition == 4)
      {
        ic2display.print((const __FlashStringHelper *)d_arrow);
      }
      else
      {
        ic2display.print("  ");
      }
      ic2display.print((const __FlashStringHelper *)d_navi_kers);
      if (kers_state == 1)
      {
        ic2display.print((const __FlashStringHelper *)d_weak);
      }
      if (kers_state == 2)
      {
        ic2display.print((const __FlashStringHelper *)d_middle);
      }
      if (kers_state == 0)
      {
        ic2display.print((const __FlashStringHelper *)d_strong);
      }
      ic2display.setCursor(15, 5);
      if (menuposition == 5)
      {
        ic2display.print((const __FlashStringHelper *)d_arrow);
      }
      else
      {
        ic2display.print("  ");
      }
      ic2display.print((const __FlashStringHelper *)d_navi_menu_exit);

      break;

    case 2:

      displayclear(0);
      ic2display.set1X();
      ic2display.setCursor(0, 0);
      ic2display.print((const __FlashStringHelper *)d_main_battery);
      ic2display.setCursor(70, 0);
      ic2display.print((const __FlashStringHelper *)d_main_headtlight);
      ic2display.setCursor(20, 1);
      ic2display.set2X();
      ic2display.print((*ptrmaininformation).batterylevel);
      ic2display.setCursor(76, 1);
      if ((*ptrmaininformation).headlightstate == 0)
      {

        ic2display.print((const __FlashStringHelper *)d_off);
      }
      else
      {
        ic2display.print((const __FlashStringHelper *)d_on);
      }
      ic2display.setCursor(70, 1);
      ic2display.set1X();
      ic2display.setCursor(0, 3);
      ic2display.print((const __FlashStringHelper *)d_line);
      ic2display.setCursor(0, 4);
      ic2display.print((const __FlashStringHelper *)d_main_temperature);
      ic2display.setCursor(10, 5);
      ic2display.set2X();
      ic2display.print((*ptrmaininformation).temperature / 10);
      ic2display.setCursor(70, 5);
      if (((*ptrmaininformation).batterylevel < 56) && (batterycounter == 50) && (batterywarning == true))
      {
        ic2display.print((const __FlashStringHelper *)d_b);
        batterycounter = 0;
      }
      else
      {
        batterycounter++;
        ic2display.print("     ");
      }

      break;

    case 3:
      displayclear(6);
      ic2display.setCursor(30, 0);
      ic2display.set1X();
      ic2display.print((const __FlashStringHelper *)d_info_odometer);
      ic2display.setCursor(60, 1);
      ic2display.set2X();
      ic2display.print((*ptrmaininformation).odometer / 1000);
      ic2display.setCursor(0, 3);
      ic2display.set1X();
      ic2display.print((const __FlashStringHelper *)d_line);
      ic2display.setCursor(10, 4);
      ic2display.print((const __FlashStringHelper *)d_info_tripdistance);
      ic2display.setCursor(60, 5);
      ic2display.set2X();
      ic2display.print((*ptrmaininformation).tripdistance / 100);
      if ((controlbrakevalue == 1) && (controlthrottlevalue == -1))
      {
        display_option = 1;
        ic2display.clear();
      }
      break;
    }
  }

  //show speed menu when no other menu option is used
  if ((display_option != 1) && (display_option != 2) && (display_option != 3))
  {
    displayclear(2);
    ic2display.set1X();
    ic2display.setCursor(89, 0);
    ic2display.print((const __FlashStringHelper *)d_km);
    ic2display.setCursor(58, 1);
    ic2display.set2X();
    ic2display.print(speed);
    ic2display.setCursor(0, 3);
    ic2display.set1X();
    ic2display.print((const __FlashStringHelper *)d_line);
    ic2display.setCursor(40, 4);
    ic2display.set1X();
    ic2display.print((const __FlashStringHelper *)d_main_battery);
    ic2display.setCursor(58, 5);
    ic2display.set2X();
    ic2display.print((*ptrmaininformation).batterylevel);
    display_option = 2;
  }
}

void recievedata()
{
  uint8_t recievedbyte;
  //Expect Packet  55 AA 07 20 65 00 04 28 27 00 00 20 FF
  while (Serial.available() > 0)
  {
    recievedbyte = Serial.read();

    switch (bufferreceiverstate)
    {
    case 0:                         
      if (recievedbyte == HEADER55) //  = 0x55
      {
        bufferreceiverstate = 1;
      }
      break;
    case 1:                         
      if (recievedbyte == HEADERAA) //  = 0xAA
      {
        bufferreceiverstate = 2;
      }
      break;
    case 2:                 
      if (recievedbyte > 35) //possibility that length is >35 which will overflow the buffer
      {
        bufferreceiverstate = 0;
      }
      bufferreadindex = 0;
      receiverbuffer.length = recievedbyte + 1; //packet length counts from second byte
      checksumcalculation = recievedbyte;
      bufferreceiverstate = 3;
      break;
    case 3: 
      receiverbuffer.buffer[bufferreadindex] = recievedbyte;
      bufferreadindex++;
      checksumcalculation = checksumcalculation + recievedbyte;
      if (bufferreadindex == receiverbuffer.length)
      {
        bufferreceiverstate = 4;
      }
      break;
    case 4: 
      checksum1 = recievedbyte;
      bufferreceiverstate = 5;
      break;
    case 5: 
      checksum2 = recievedbyte;
      checksumcalculation = checksumcalculation ^ 0xffff;
      if (checksumcalculation == ((uint16_t)(checksum2 << 8) + (uint16_t)checksum1))
      {
        processData();
      }
      if (receiverbuffer.buffer[bAddr] == 0x20 && receiverbuffer.buffer[bcmd] == 0x65 && receiverbuffer.buffer[bArg] == 0x00 && packagePreparedFlag == 1)
      {

        sendecounter++;
        //we will only send if we reciev 60x
        if (sendecounter > 60)
        {
          disableRX(); //disable reciever to not get our on messages

          Serial.write(sendbuffer.buffer, sendbuffer.length);

          enableRX();
          
          packagePreparedFlag = 0; //next package can be prepared
          sendecounter = 0;
        }
      }

      bufferreceiverstate = 0; //next packet can be read start from beginning of switch case 
      break;
    }
  }
}

void processData()
{
  switch (receiverbuffer.buffer[0])
  {
  case REQUESTTOESC: //0x20
    switch (receiverbuffer.buffer[1])
    {
    case 0x65:
      switch (receiverbuffer.buffer[2])
      {
      case 0x00:
        throttlebrakeValues.throttle = receiverbuffer.buffer[4];
        throttlebrakeValues.brake = receiverbuffer.buffer[5];
        break;
      default:
        break;
      }
    default:
      break;
    }
  case REPLYFROMESC: //0x23
    switch (receiverbuffer.buffer[1])
    {
    case 0x01: //read
      switch (receiverbuffer.buffer[2])
      {
      case 0xB0:
        mainformation.speed = ((int16_t)receiverbuffer.buffer[13]) | ((int16_t)receiverbuffer.buffer[14] << 8);
        mainformation.batterylevel = ((uint16_t)receiverbuffer.buffer[11]) | ((uint16_t)receiverbuffer.buffer[12] << 8);
        mainformation.odometer = ((uint32_t)receiverbuffer.buffer[17]) | ((uint32_t)receiverbuffer.buffer[18] << 8) | ((uint32_t)receiverbuffer.buffer[19] << 16) | ((uint32_t)receiverbuffer.buffer[20] << 24);
        mainformation.tripdistance = ((uint16_t)receiverbuffer.buffer[21]) | ((uint16_t)receiverbuffer.buffer[22] << 8);
        mainformation.temperature = ((uint16_t)receiverbuffer.buffer[25]) | ((uint16_t)receiverbuffer.buffer[26] << 8);

        break;

      default:
        break;
      }
      break;

    default:
      break;
    }
  case REQUESTTOBLE: //0x20
    switch (receiverbuffer.buffer[1])
    {
    case 0x64:
      switch (receiverbuffer.buffer[2])
      {
      case 0x00:
        if ((receiverbuffer.buffer[5] == 0 || receiverbuffer.buffer[5] == 100))
        {
          mainformation.headlightstate = receiverbuffer.buffer[5];
        }
        break;
      default:
        break;
      }
    default:
      break;
    }

  default:
    break;
  }
  renewdispalyFlag = true;
}

void preparePacket(uint8_t option)
{

  queuebuffer *ptsendequeue = &sendbuffer;
  checksumcalculation = 0x00;
  //every packet same
  (*ptsendequeue).buffer[0] = HEADER55;
  (*ptsendequeue).buffer[1] = HEADERAA;

  switch (option)
  {
  case 1:

    //55 AA 06 20 61 B0 20 02 28 27 57 FE
    (*ptsendequeue).buffer[2] = READESCDATA[0];
    (*ptsendequeue).buffer[3] = READESCDATA[1];
    (*ptsendequeue).buffer[4] = READESCDATA[2];
    (*ptsendequeue).buffer[5] = 0xB0;
    (*ptsendequeue).buffer[6] = 0x20;
    (*ptsendequeue).buffer[7] = packetunkown;
    (*ptsendequeue).buffer[8] = throttlebrakeValues.throttle;
    (*ptsendequeue).buffer[9] = throttlebrakeValues.brake;

    checksumcalculation += READESCDATA[0];
    checksumcalculation += READESCDATA[1];
    checksumcalculation += READESCDATA[2];
    checksumcalculation += 0xB0;
    checksumcalculation += 0x20;
    checksumcalculation += packetunkown;
    checksumcalculation += throttlebrakeValues.throttle;
    checksumcalculation += throttlebrakeValues.brake;
    checksumcalculation ^= 0xFFFF;

    (*ptsendequeue).buffer[10] = (uint8_t)(checksumcalculation & 0xFF);
    (*ptsendequeue).buffer[11] = (uint8_t)((checksumcalculation & 0xFF00) >> 8);
    (*ptsendequeue).length = 12;
    break;
  case 2:
    break;
  default:
    break;
  }

  packagePreparedFlag = 1;
}

void preparewritePacket(uint8_t writeoption)
{
  queuebuffer *ptsendequeue = &sendbuffer;
  checksumcalculation = 0x00;
  (*ptsendequeue).buffer[0] = HEADER55;
  (*ptsendequeue).buffer[1] = HEADERAA;
  (*ptsendequeue).length = 10; //write packages have all lenght of 10
  switch (writeoption)
  {
  case WKERSWEAK:                             //1 5AA42037B005DFF tested
    (*ptsendequeue).buffer[2] = 0x04;         //size of packet
    (*ptsendequeue).buffer[3] = REQUESTTOESC; //0x20
    (*ptsendequeue).buffer[4] = CMDWRITE;     //0x03
    (*ptsendequeue).buffer[5] = RKERS;        //register
    (*ptsendequeue).buffer[6] = KERSWEAK;
    (*ptsendequeue).buffer[7] = 0x00;

    checksumcalculation += 0x04;
    checksumcalculation += REQUESTTOESC;
    checksumcalculation += CMDWRITE;
    checksumcalculation += RKERS;
    checksumcalculation += KERSWEAK;
    checksumcalculation += 0x00;
    checksumcalculation ^= 0xFFFF;

    (*ptsendequeue).buffer[8] = (uint8_t)(checksumcalculation & 0xFF);
    (*ptsendequeue).buffer[9] = (uint8_t)((checksumcalculation & 0xFF00) >> 8);
    break;
  case WKERSMEDIUM: //2 55AA42037B105CFF tested
    (*ptsendequeue).buffer[2] = 0x04;
    (*ptsendequeue).buffer[3] = REQUESTTOESC;
    (*ptsendequeue).buffer[4] = CMDWRITE;
    (*ptsendequeue).buffer[5] = RKERS;
    (*ptsendequeue).buffer[6] = KERSMEDIUM;
    (*ptsendequeue).buffer[7] = 0x00;

    checksumcalculation += 0x04;
    checksumcalculation += REQUESTTOESC;
    checksumcalculation += CMDWRITE;
    checksumcalculation += RKERS;
    checksumcalculation += KERSMEDIUM;
    checksumcalculation += 0x00;
    checksumcalculation ^= 0xFFFF;

    (*ptsendequeue).buffer[8] = (uint8_t)(checksumcalculation & 0xFF);
    (*ptsendequeue).buffer[9] = (uint8_t)((checksumcalculation & 0xFF00) >> 8);
    break;
  case WKERSSTRONG: //3 55AA42037B205BFF tested
    (*ptsendequeue).buffer[2] = 0x04;
    (*ptsendequeue).buffer[3] = REQUESTTOESC;
    (*ptsendequeue).buffer[4] = CMDWRITE;
    (*ptsendequeue).buffer[5] = RKERS;
    (*ptsendequeue).buffer[6] = KERSSTRONG;
    (*ptsendequeue).buffer[7] = 0x00;

    checksumcalculation += 0x04;
    checksumcalculation += REQUESTTOESC;
    checksumcalculation += CMDWRITE;
    checksumcalculation += RKERS;
    checksumcalculation += KERSSTRONG;
    checksumcalculation += 0x00;
    checksumcalculation ^= 0xFFFF;

    (*ptsendequeue).buffer[8] = (uint8_t)(checksumcalculation & 0xFF);
    (*ptsendequeue).buffer[9] = (uint8_t)((checksumcalculation & 0xFF00) >> 8);
    break;
  case WTAILLIGHTOFF: //4 55AA42037D005BFF tested
    (*ptsendequeue).buffer[2] = 0x04;
    (*ptsendequeue).buffer[3] = REQUESTTOESC;
    (*ptsendequeue).buffer[4] = CMDWRITE;
    (*ptsendequeue).buffer[5] = RBACKLIGHT;
    (*ptsendequeue).buffer[6] = OFF;
    (*ptsendequeue).buffer[7] = 0x00;

    checksumcalculation += 0x04;
    checksumcalculation += REQUESTTOESC;
    checksumcalculation += CMDWRITE;
    checksumcalculation += RBACKLIGHT;
    checksumcalculation += OFF; //0x00
    checksumcalculation += 0x00;
    checksumcalculation ^= 0xFFFF;

    (*ptsendequeue).buffer[8] = (uint8_t)(checksumcalculation & 0xFF);
    (*ptsendequeue).buffer[9] = (uint8_t)((checksumcalculation & 0xFF00) >> 8);
    break;
  case WBACKLIGHTON: //5 55AA42037D2059FF tested
    (*ptsendequeue).buffer[2] = 0x04;
    (*ptsendequeue).buffer[3] = REQUESTTOESC;
    (*ptsendequeue).buffer[4] = CMDWRITE;
    (*ptsendequeue).buffer[5] = RBACKLIGHT;
    (*ptsendequeue).buffer[6] = BACKLIGHTON;
    (*ptsendequeue).buffer[7] = 0x00;

    checksumcalculation += 0x04;
    checksumcalculation += REQUESTTOESC;
    checksumcalculation += CMDWRITE;
    checksumcalculation += RBACKLIGHT;
    checksumcalculation += BACKLIGHTON;
    checksumcalculation += 0x00;
    checksumcalculation ^= 0xFFFF;

    (*ptsendequeue).buffer[8] = (uint8_t)(checksumcalculation & 0xFF);
    (*ptsendequeue).buffer[9] = (uint8_t)((checksumcalculation & 0xFF00) >> 8);
    break;
  case WCRUISEON: //6 55AA42037C105BFF tested
    (*ptsendequeue).buffer[2] = 0x04;
    (*ptsendequeue).buffer[3] = REQUESTTOESC;
    (*ptsendequeue).buffer[4] = CMDWRITE;
    (*ptsendequeue).buffer[5] = RCRUISE;
    (*ptsendequeue).buffer[6] = CRUISEON;
    (*ptsendequeue).buffer[7] = 0x00;

    checksumcalculation += 0x04;
    checksumcalculation += REQUESTTOESC;
    checksumcalculation += CMDWRITE;
    checksumcalculation += RCRUISE;
    checksumcalculation += CRUISEON;
    checksumcalculation += 0x00;
    checksumcalculation ^= 0xFFFF;

    (*ptsendequeue).buffer[8] = (uint8_t)(checksumcalculation & 0xFF);
    (*ptsendequeue).buffer[9] = (uint8_t)((checksumcalculation & 0xFF00) >> 8);
    break;
  case WCRUISEOFF: //7  55AA42037C005CFF tested
    (*ptsendequeue).buffer[2] = 0x04;
    (*ptsendequeue).buffer[3] = REQUESTTOESC;
    (*ptsendequeue).buffer[4] = CMDWRITE;
    (*ptsendequeue).buffer[5] = RCRUISE;
    (*ptsendequeue).buffer[6] = OFF;
    (*ptsendequeue).buffer[7] = 0x00;

    checksumcalculation += 0x04;
    checksumcalculation += REQUESTTOESC;
    checksumcalculation += CMDWRITE;
    checksumcalculation += RCRUISE;
    checksumcalculation += OFF;
    checksumcalculation += 0x00;
    checksumcalculation ^= 0xFFFF;

    (*ptsendequeue).buffer[8] = (uint8_t)(checksumcalculation & 0xFF);
    (*ptsendequeue).buffer[9] = (uint8_t)((checksumcalculation & 0xFF00) >> 8);
    break;
  default:
    break;
  }

  packagePreparedFlag = 1;
}

//saves previous config for display

void saveandloadConfig()
{
  bool configsaved = EEPROM.read(0);
  if (configsaved == true)
  {
    batterywarning = EEPROM.read(1);
    cruise_state = EEPROM.read(2);
    taillight_state = EEPROM.read(3);
    kers_state = EEPROM.read(4);
  }
  else
  {
    EEPROM.update(0, true);
    EEPROM.update(1, batterywarning);
    EEPROM.update(2, cruise_state);
    EEPROM.update(3, taillight_state);
    EEPROM.update(4, kers_state);
  }
}
void enableRX()
{
  UCSR0B &= ~((1 << RXEN0));
}

void disableRX()
{
  UCSR0B |= (1 << RXEN0);
}
