/*******************************************************************************
  Copyright 2016 ROBOTIS CO., LTD.

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
*******************************************************************************/

/* Authors: Yoonseok Pyo, Leon Jung, Darby Lim, HanCheol Cho */
#include <DynamixelSDK.h>
#include <stdarg.h>

// Protocol version
#define PROTOCOL_VERSION1               1.0                 // See which protocol version is used in the Dynamixel
#define PROTOCOL_VERSION2               2.0

// Default setting
#if defined(__OPENCR__) 
#define DEVICENAME                      "/dev/OpenCR"       // Device name not used on OpenCR
#elif defined(__OPENCM904__)
#define DEVICENAME                      "3"                 // Default to external (OpenCM485 expansion) on OpenCM9.04
#endif

// ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"
#define CMD_SERIAL                      Serial              // USB Serial



typedef union
{
  uint8_t  u8Data[4];
  uint16_t u16Data[2];
  uint32_t u32Data;

  int8_t   s8Data[4];
  int16_t  s16Data[2];
  int32_t  s32Data;
} dxl_ret_t;

char *dev_name = (char*)DEVICENAME;

// Initialize Packethandler2 instance
dynamixel::PacketHandler *packetHandler2;
dynamixel::PortHandler   *portHandler;


int tb3_id = -1;
int tb3_baud = -1;


void set_speed(int16_t left, int16_t right);


void      write(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, uint8_t id, uint16_t addr, uint16_t length, uint32_t value);
dxl_ret_t read(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, uint8_t id, uint16_t addr, uint16_t length);



void setup()
{
  CMD_SERIAL.begin(57600);
  while (!CMD_SERIAL);


  packetHandler2 = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION2);
  portHandler    = dynamixel::PortHandler::getPortHandler(dev_name);

  // Open port
  if (portHandler->openPort())
  {
    CMD_SERIAL.println("Succeeded to open the port!");
    CMD_SERIAL.printf(" - Device Name : %s\r\n", dev_name);
    CMD_SERIAL.printf(" - Baudrate    : %d\r\n", portHandler->getBaudRate());
    tb3_baud = portHandler->getBaudRate();
    // We run at 1000000
    portHandler->setBaudRate(1000000);
  }
  else
  {
    CMD_SERIAL.printf("Failed to open the port! [%s]\n", dev_name);
    CMD_SERIAL.printf("Press any key to terminate...\n");
    while (1);
  }

}

void loop()
{
  int32_t left, right;

  if (CMD_SERIAL.available())
  {
    left = CMD_SERIAL.parseInt();
    right = CMD_SERIAL.parseInt();
    set_speed(left, right);
  }
}

void set_speed(int16_t left, int16_t right)
{
    // Torque enable
    write(portHandler, packetHandler2, 1, 64, 1, 1);
    write(portHandler, packetHandler2, 2, 64, 1, 1);
    // Set speed 
    write(portHandler, packetHandler2, 1, 104, 4, left);
    write(portHandler, packetHandler2, 2, 104, 4, right);
}

void write(dynamixel::PortHandler *portHandler, 
    dynamixel::PacketHandler *packetHandler, 
    uint8_t id, uint16_t addr, uint16_t length, uint32_t value)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  if (length == 1)
  {
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, addr, (uint8_t)value, &dxl_error);
  }
  else if (length == 2)
  {
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, addr, (uint16_t)value, &dxl_error);
  }
  else if (length == 4)
  {
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, id, addr, (uint32_t)value, &dxl_error);
  }

  if (dxl_comm_result == COMM_SUCCESS)
  {
    if (dxl_error != 0) CMD_SERIAL.println(packetHandler->getRxPacketError(dxl_error));
  }
  else
  {
    CMD_SERIAL.println(packetHandler->getTxRxResult(dxl_error));
    CMD_SERIAL.println("Fail to write!");
  }
}

dxl_ret_t read(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, uint8_t id, uint16_t addr, uint16_t length)
{
  uint8_t dxl_error = 0;
  int     dxl_comm_result = COMM_TX_FAIL;
  dxl_ret_t ret;

  int8_t  value8    = 0;
  int16_t value16   = 0;
  int32_t value32   = 0;


  if (length == 1)
  {
    dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, id, addr, (uint8_t*)&value8, &dxl_error);
  }
  else if (length == 2)
  {
    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, id, addr, (uint16_t*)&value16, &dxl_error);
  }
  else if (length == 4)
  {
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, id, addr, (uint32_t*)&value32, &dxl_error);
  }

  if (dxl_comm_result == COMM_SUCCESS)
  {
    if (dxl_error != 0) CMD_SERIAL.println(packetHandler->getRxPacketError(dxl_error));

    if (length == 1)
    {
      ret.u32Data = value8;
    }
    else if (length == 2)
    {
      ret.u32Data = value16;
    }
    else if (length == 4)
    {
      ret.u32Data = value32;
    }
  }
  else
  {
    CMD_SERIAL.println(packetHandler->getTxRxResult(dxl_error));
    CMD_SERIAL.println("Fail to read! ");
  }

  return ret;
}
