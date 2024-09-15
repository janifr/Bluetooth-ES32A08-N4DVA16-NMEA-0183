#define HC595_CLOCK 27   // SRCLK/SHCP  11
#define HC595_LATCH 14   // RCLK / STCP 12
#define HC595_DATA 13   // SER / DS    14
#define HC595_OUTPUT_ENABLE         4
#define RS485_RTS 22
#define POWER_LED 15

#define SERIAL_DEBUG_DISABLED

#include <ModbusMaster.h>
#include <ReactESP.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

//using namespace sensesp;

reactesp::ReactESP app;
uint8_t relay = 1;
uint8_t segment = 0xf7;
uint8_t segment_counter = 0;
uint8_t voltage_counter = 0;
uint8_t display[4];
uint16_t n4dva16_voltage[16];
uint32_t button_age[4];
const uint8_t button_pin[] = {18,19,21,23};
uint8_t channel = 0;
uint8_t channel_old = 0;
uint8_t channel_age = 0;
uint8_t outputs = 0;
uint8_t outputs_age = 0;

ModbusMaster node;
uint8_t modbus_ok= 0;

hw_timer_t *output_timer = NULL;

uint8_t SEG8Code[] = {0,0,0,0,0,0,0,0, //0-7 ASCII
                      0,0,0,0,0,0,0,0, //8-15
                      0,0,0,0,0,0,0,0, //16-23
                      0,0,0,0,0,0,0,0, //24-31
                      0,0,0,0,0,0,0,0, //32-39
                      0,0,0,0,0,0,0,0, //40-47
                      0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07, //48-55, numerals 0-7
                      0x7f,0x6f,0,0,0,0,0,0, //56-63, numerals 8 and 9
                      0,0x77,0x7c,0x39,0x5e,0x79,0x71,0, //A-F
                      0x76};//H

void preTransmission()
{
  digitalWrite(RS485_RTS, 1);
}

void postTransmission()
{
  digitalWrite(RS485_RTS, 0);
}

void Digital_Output_Byte(uint8_t dat)   //Send 1 byte
{
    uint8_t i;
    for(i=8;i>=1;i--)
    {
        if(dat & 0x80){
      digitalWrite(HC595_DATA, 1);
    } else {
      digitalWrite(HC595_DATA, 0);
    }       //Sends data bit by bit from high to low.
        dat <<= 1;
    digitalWrite(HC595_CLOCK, 0);
      digitalWrite(HC595_CLOCK, 1);
    }
}

void Output_ES32A08(uint8_t Num, uint8_t Seg, uint8_t out)
{
    Digital_Output_Byte(out);
    Digital_Output_Byte(Seg);
    Digital_Output_Byte(Num);
  digitalWrite(HC595_LATCH, 0);
  digitalWrite(HC595_LATCH, 1);
}

void IRAM_ATTR Update_Outputs()
{
   if (modbus_ok)
    display[1] |= 0x80;
  Output_ES32A08(display[3-segment_counter],segment >> segment_counter,outputs);
  segment_counter++;
  if(segment_counter>3)
    segment_counter=0;

  if(outputs)
  {
    outputs_age++;
    if(outputs_age>200)
    {
      outputs=0;
      outputs_age=0;
    }
  }

  for(int i=0;i<4;i++)
  {
    if (digitalRead(button_pin[i]))
      button_age[i]=0;
    else
      button_age[i]++;
  }

  if ((button_age[0] == 5 ) && (channel>0))
    channel--;

  if ((button_age[0] == 500 ) && (channel>0))
  {
    channel--;
    button_age[0] = 250;
  }

  if ((button_age[1] == 5 ) && (channel<15))
    channel++;

  if ((button_age[1] == 500 ) && (channel<15))
  {
    channel++;
    button_age[1] = 250;
  }
}

void Display_Update()
{
  char buffer[10];

  if (channel_age)
  {
    sprintf(buffer, "CH%2u",channel+1);
    channel_age--;
  } 
  else
    sprintf(buffer, "%4u",n4dva16_voltage[channel]);

  if (channel != channel_old)
  {
    channel_age=10;
    channel_old = channel;
  }

  display[0]=SEG8Code[buffer[0]];
  display[1]=SEG8Code[buffer[1]];
  display[2]=SEG8Code[buffer[2]];
  display[3]=SEG8Code[buffer[3]];

  voltage_counter++;
  if (voltage_counter>31)
    voltage_counter=0;
}

void Modbus_Read()
{
  uint8_t result;
  digitalWrite(POWER_LED, 1);
  modbus_ok = 0;
  char message_buffer[80];
  int message_length = 0;
  uint8_t crc = 0;
  uint8_t crc_incoming;

  // Read 16 registers starting at 0x0000)
  result = node.readInputRegisters(0x0000, 16);
  if (result == node.ku8MBSuccess)
  {
    modbus_ok=1;
    
    for(int i=0;i<16;i++)
    {
      n4dva16_voltage[i]=node.getResponseBuffer(i);
      
      message_length = 0;
      crc = 0;
      message_length += sprintf(message_buffer, "$ERAI%X,%u*",i,n4dva16_voltage[i]);
      for(int j=1;message_buffer[j]!='*';j++)
        crc = crc ^ message_buffer[j];
      message_length += sprintf(message_buffer + message_length, "%02x\r\n", crc);
      SerialBT.print(message_buffer);
    }
  }
}

void Bluetooth_Read()
{
  char input;
  char message_buffer[80];
  const char identifier[] = "$ERDOB,";
  
  int message_length = 0;

  char* identifier_position;
  char* crc_position;

  u_int8_t message_state = 0;
  u_int8_t message_crc = 0;
  u_int8_t crc = 0;

  while (SerialBT.available() && message_length < 80)
  {
    message_buffer[message_length]=SerialBT.read();
    message_length++;
  }

  identifier_position = strstr(message_buffer, identifier);
  if (identifier_position)
  {
    message_state = atoi(identifier_position+7);
    crc_position = strstr(identifier_position+8, "*");
    if (crc_position)
    {
      message_crc = strtol (crc_position+1,NULL,16);
      identifier_position++;
      for(;identifier_position<crc_position;identifier_position++)
        crc = crc ^ *identifier_position;
      if(message_crc == crc)
      {

          outputs = message_state;
          outputs_age = 0;
      }
    }
  }
}

void setup() {
  output_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(output_timer, &Update_Outputs, true);
  timerAlarmWrite(output_timer, 1000, true);
  timerAlarmEnable(output_timer);
  
  pinMode(HC595_CLOCK, OUTPUT);
  pinMode(HC595_LATCH, OUTPUT);
  pinMode(HC595_DATA, OUTPUT);
  pinMode(HC595_OUTPUT_ENABLE, OUTPUT);
  pinMode(POWER_LED, OUTPUT);
  for (int i=0;i<4;i++)
    pinMode(button_pin[i],INPUT_PULLUP);
  
  digitalWrite(HC595_OUTPUT_ENABLE,1);
  Output_ES32A08(0,0,0);
  digitalWrite(HC595_OUTPUT_ENABLE,0);

  pinMode(RS485_RTS, OUTPUT);
  digitalWrite(RS485_RTS, 0);

  Serial.begin(9600);
  SerialBT.begin("Linda"); //Bluetooth device name

  // Modbus slave ID 32
  node.begin(32, Serial);
  // Callbacks allow us to configure the RS485 transceiver correctly
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  app.onRepeat(10, [] ()
  {
    Bluetooth_Read();
  });

  app.onRepeat(100, [] ()
  {
    Display_Update();
  });

  app.onRepeat(1000, [] ()
  {
    Modbus_Read();
  });
}

void loop()
{
  app.tick();
}
