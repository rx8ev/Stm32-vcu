
#include "MAZDA_RX8.h"
#include "stm32_can.h"
#include "params.h"
#include "utils.h"
#include "my_math.h"
#include "hwinit.h"
#include <libopencm3/stm32/gpio.h>

//these messages go out on vehicle can and are specific to driving the RX8 instrument cluster etc.

// Variables for PCM, Only overrideable if PCM removed from CAN
bool checkEngineMIL = 0;
bool checkEngineBL = 0;
uint8_t engTemp = MAX(Param::GetInt(Param::tmpm), Param::GetInt(Param::tmphs))+100; //125;     //Roughly in the middle
uint8_t odo = 0;
bool oilPressure = 1;
bool lowWaterMIL = 0;
bool batChargeMIL = 0;
bool oilPressureMIL = 0;

// Variables for PCM, Only overrideable if PCM removed from CAN
uint16_t engineRPM = 1000;
uint16_t throttlePedal = 0;
uint16_t vehicleSpeed = 0x1388;  //0;
//uint8_t throttlePedal;

// Variables for ABS/DSC, Only overrideable if ABS/DSC removed from CAN
bool dscOff = 0;
bool absMIL = 0;
bool brakeFailMIL = 0;
bool etcActiveBL = 0;
bool etcDisabled = 0;

// Variables for Wheel Speed
uint16_t frontLeft = 0;
uint16_t frontRight = 0;
uint16_t rearLeft = 0;
uint16_t rearRight = 0;

bool MAZDA_RX8::Ready()
{
    return DigIo::t15_digi.Get();
}

bool MAZDA_RX8::Start()
{
    return Param::GetBool(Param::din_start);
}

void MAZDA_RX8::SetCanInterface(CanHardware* c)
{
    can = c;

    can->RegisterUserMessage(0x47);//Keyless Control & Immobiliser
    can->RegisterUserMessage(0x4B1);//Wheel speeds

}

/////////////////////////////////////////////////////////////////////////////////////////////////////
///////Handle incomming can messages from the car here
/////////////////////////////////////////////////////////////////////////////////////////////////////

void MAZDA_RX8::DecodeCAN(int id, uint32_t* data)
{

    switch (id)
    {
    case 0x47:
        MAZDA_RX8::handle47(data);
        break;

    case 0x4B1:
        MAZDA_RX8::handle4B1(data);
        break;

    default:
        break;
    }
}

void MAZDA_RX8::handle47(uint32_t data[2]) //Keyless Control Module and Immobiliser want to have a chat with the PCM, this deals with the conversation
{

    uint8_t* bytes = (uint8_t*)data;// arrgghhh this converts the two 32bit array into bytes. See comments are useful:)

   if(bytes[1] == 127 && bytes[2] == 2)
   {
       //KCM / Immobiliser chat replies
       // Reply to 47 first

       bytes[0]=0x07;    //Static values work fine here
       bytes[1]=0x7A; //0C;    //
       bytes[2]=0x79; //30;    //
       bytes[3]=0xA3; //F2;    //
       bytes[4]=0x43; //17;    //
       bytes[5]=0x00;    //
       bytes[6]=0x00;    //
       bytes[7]=0x00;    //
       
       can->Send(0x41A, (uint32_t*)bytes,8); //Send on CAN2
       // 0x41A = {7,12,48,242,23,0,0,0}
   }
   if(bytes[1] == 92 && bytes[2] == 244)
   {
       //KCM / Immobiliser chat replies
       // Reply to 47 second

       bytes[0]=0x81;    //Static values work fine here
       bytes[1]=0x7F;    //
       bytes[2]=0x00;    //
       bytes[3]=0x00;    //
       bytes[4]=0x00;    //
       bytes[5]=0x00;    //
       bytes[6]=0x00;    //
       bytes[7]=0x00;    //
       
       can->Send(0x41B, (uint32_t*)bytes,8); //Send on CAN2
       // 0x41B = {129,127,0,0,0,0,0,0}
   }
}

void MAZDA_RX8::handle4B1(uint32_t data[2]) //Read wheel speeds to update Dash, Check if it is 1200 dec
{

    uint8_t* bytes = (uint8_t*)data;// arrgghhh this converts the two 32bit array into bytes. See comments are useful:)

    frontLeft = (bytes[0] * 256) + bytes[1];
    frontRight = (bytes[2] * 256) + bytes[3];
    rearLeft = (bytes[4] * 256) + bytes[5];
    rearRight = (bytes[6] * 256) + bytes[7];

    vehicleSpeed = frontLeft;
    //Going to check front wheel speeds for any issues, ignoring the rears due to problems created by wheelspin
 //   if (frontLeft - frontRight > 500 || frontLeft - frontRight < -500)  //more than 5kph difference in wheel speed
 //   {
 //       checkEngineMIL = 1; //light up engine warning light and set speed to zero
 //       vehicleSpeed = 0;
 //   }
 //   else
 //   {
 //       vehicleSpeed = ((frontLeft + frontRight) / 2) / 100; //Get average of front two wheels.
 //   }
}

void MAZDA_RX8::Task100Ms()     // Send CAN messages on Various ID's
{
    engineRPM = Param::GetInt(Param::speed)*3.85;
    throttlePedal = Param::GetInt(Param::potnom)*2;

    uint8_t bytes[8];
    if(engineRPM < 3850)
    {
    bytes[0]=0x0F;    //RPM high byte Motor RPM*3.85
    bytes[1]=0x0A;    //RPM low byte
    }
    else {
    bytes[0]=engineRPM >> 8;
    bytes[1]=engineRPM;
    }
    bytes[2]=0xFF;    //
    bytes[3]=0xFF;    //
    bytes[4]= vehicleSpeed >> 8; //0x13;    //Vehicle speed high byte
    bytes[5]= vehicleSpeed;      //0x88;    //Vehicle speed low byte
    bytes[6]= throttlePedal;    //
    bytes[7]=0xFF;    //
    
    can->Send(0x201, (uint32_t*)bytes,8); //Send on CAN2
    // 0x201 = {0, 0, 255, 255, 0, 0, 0, 255}

    //engTemp = Param::GetInt(Param::tmpm)*2+75;
    engTemp = Param::GetInt(Param::tmphs)*2+75;


    bytes[0]=engTemp;       //
    bytes[1]=odo;           //
    bytes[2]=0x00;          //
    bytes[3]=0x00;          //
    bytes[4]=oilPressure;   //
    bytes[5]=0;
    if (checkEngineMIL == 1)
    {
        bytes[5] = bytes[5] | 0b01000000;
    }
    else
    {
        bytes[5] = bytes[5] & 0b10111111;
    }

    if (checkEngineBL == 1)
    {
        bytes[5] = bytes[5] | 0b10000000;
    }
    else
    {
        bytes[5] = bytes[5] & 0b01111111;
    }
    bytes[6]=0;
    if (lowWaterMIL == 1)
    {
        bytes[6] = bytes[6] | 0b00000010;
    }
    else
    {
        bytes[6] = bytes[6] & 0b11111101;
    }

    if (batChargeMIL == 1)
    {
        bytes[6] = bytes[6] | 0b01000000;
    }
    else
    {
        bytes[6] = bytes[6] & 0b10111111;
    }

    if (oilPressureMIL == 1)
    {
        bytes[6] = bytes[6] | 0b10000000;
    }
    else
    {
        bytes[6] = bytes[6] & 0b01111111;
    }
    bytes[7]=0x00;          //
    
    can->Send(0x420, (uint32_t*)bytes,7); //Send on CAN2
    // 0x420 = {0, 0, 0, 0, 0, 0, 0}

// control of ABS / DSC Lights

    bytes[0]=0x00;    //Static values work fine here
    bytes[1]=0x00;    //
    bytes[2]=0x00;    //
    bytes[3]=0x00;    //
    if (dscOff == 1)
    {
        bytes[3] = bytes[3] | 0b00000100;
    }
    else
    {
        bytes[3] = bytes[3] & 0b01111011;
    }
    bytes[4]=0x00;    //
    if (absMIL == 1)
    {
        bytes[4] = bytes[4] | 0b00001000;
    }
    else
    {
        bytes[4] = bytes[4] & 0b11110111;
    }
    if (brakeFailMIL == 1)
    {
        bytes[4] = bytes[4] | 0b01000000;
    }
    else
    {
        bytes[4] = bytes[4] & 0b10111111;
    }
    bytes[5]=0x00;    //
    if (etcActiveBL == 1)
    {
        bytes[5] = bytes[5] | 0b00100000;
    }
    else
    {
        bytes[5] = bytes[5] & 0b11011111;
    }

    if (etcDisabled == 1)
    {
        bytes[5] = bytes[5] | 0b00010000;
    }
    else
    {
        bytes[5] = bytes[5] & 0b11101111;
    }
    bytes[6]=0x00;    //
    
    can->Send(0x212, (uint32_t*)bytes,7); //Send on CAN2
    // 0x212 = {0, 0, 0, 0, 0, 0, 0}

//Setup PCM Status's required to fool all other CAN devices that everything is OK, just send these out continuously

    bytes[0]=0x13;    //Static values work fine here
    bytes[1]=0x13;    //
    bytes[2]=0x13;    //
    bytes[3]=0x13;    //
    bytes[4]=0xAF;    //
    bytes[5]=0x03;    //
    bytes[6]=0x13;    //
    
    can->Send(0x203, (uint32_t*)bytes,7); //Send on CAN2
    // 0x203 = {19,19,19,19,175,3,19}
    
//data to do with traction control

    bytes[0]=0x02;    //Static values work fine here
    bytes[1]=0x2D;    //
    bytes[2]=0x02;    //
    bytes[3]=0x2D;    //
    bytes[4]=0x02;    //
    bytes[5]=0x2A;    //
    bytes[6]=0x06;    //
    bytes[7]=0x81;    //
    
    can->Send(0x215, (uint32_t*)bytes,8); //Send on CAN2
    // 0x215 = {2, 45, 2, 45, 2, 42, 6, 129}

    bytes[0]=0x0F;    //Static values work fine here
    bytes[1]=0x00;    //
    bytes[2]=0xFF;    //
    bytes[3]=0xFF;    //
    bytes[4]=0x00;    //
    bytes[5]=0x00;    //
    bytes[6]=0x00;    //
    bytes[7]=0x00;    //
    
    can->Send(0x231, (uint32_t*)bytes,5); //Send on CAN2
    // 0x231 = {15,0,255,255,0}

    bytes[0]=0x04;    //Static values work fine here
    bytes[1]=0x00;    //
    bytes[2]=0x28;    //
    bytes[3]=0x00;    //
    bytes[4]=0x02;    //
    bytes[5]=0x37;    //
    bytes[6]=0x06;    //
    bytes[7]=0x81;    //
    
    can->Send(0x248, (uint32_t*)bytes,8); //Send on CAN2
    // 0x248 = {4,0,40,0,2,55,6,129}

// 0x620 needed for abs light to go off, byte 6 is different on different cars, sometimes 2,3 or 4

    bytes[0]=0x00;    //Static values work fine here
    bytes[1]=0x00;    //
    bytes[2]=0x00;    //
    bytes[3]=0x00;    //
    bytes[4]=0x10;    //
    bytes[5]=0x00;    //
    bytes[6]=0x02;    // 4?
    bytes[7]=0x00;    //
    
    can->Send(0x620, (uint32_t*)bytes,7); //Send on CAN2
    // 0x620 = {0,0,0,0,16,0,4}

// 0x630 needed for abs light to go off, AT/MT and Wheel Size

    bytes[0]=0x08;    //Static values work fine here
    bytes[1]=0x00;    //
    bytes[2]=0x00;    //
    bytes[3]=0x00;    //
    bytes[4]=0x00;    //
    bytes[5]=0x00;    //
    bytes[6]=0x6A;    //
    bytes[7]=0x6A;    //
    
    can->Send(0x630, (uint32_t*)bytes,8); //Send on CAN2
    // 0x620 = {8,0,0,0,0,0,106,106}

    bytes[0]=0x00;    //Static values work fine here
    
    can->Send(0x650, (uint32_t*)bytes,1); //Send on CAN2
    // 0x650 = {0}
}


