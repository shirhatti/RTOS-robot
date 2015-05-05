// can0.c
// Runs on LM4F120/TM4C123
// Use CAN0 to communicate on CAN bus PE4 and PE5
// 

// Jonathan Valvano
// March 22, 2014

/* This example accompanies the books
   Embedded Systems: Real-Time Operating Systems for ARM Cortex-M Microcontrollers, Volume 3,  
   ISBN: 978-1466468863, Jonathan Valvano, copyright (c) 2014

   Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers, Volume 2
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2014

 Copyright 2014 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */
// MCP2551 Pin1 TXD  ---- CAN0Tx PE5 (8) O TTL CAN module 0 transmit
// MCP2551 Pin2 Vss  ---- ground
// MCP2551 Pin3 VDD  ---- +5V with 0.1uF cap to ground
// MCP2551 Pin4 RXD  ---- CAN0Rx PE4 (8) I TTL CAN module 0 receive
// MCP2551 Pin5 VREF ---- open (it will be 2.5V)
// MCP2551 Pin6 CANL ---- to other CANL on network 
// MCP2551 Pin7 CANH ---- to other CANH on network 
// MCP2551 Pin8 RS   ---- ground, Slope-Control Input (maximum slew rate)
// 120 ohm across CANH, CANL on both ends of network
#include <stdint.h>
#include "hw_can.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_types.h"
#include "can.h"
#include "debug.h"
#include "interrupt.h"

#include "can0.h"
#include "tm4c123gh6pm.h"



#define NULL 0

int32_t StartCritical(void);
void EndCritical(int32_t primask);



// reverse these IDs on the other microcontroller

// Mailbox linkage from background to foreground
uint8_t static RCVData[4];
int static MailFlag;
uint32_t button1Pressed = 0;
uint32_t button2Pressed = 0;
uint32_t Mailbox = 4000000, Mailbox2 = 9900000, Mailbox3 = 9900000, Mailbox4 = 9900000;
uint32_t Mailbox5 = 9900000, Mailbox6 = 9900000;

//*****************************************************************************
//
// The CAN controller interrupt handler.
//
//*****************************************************************************
void CAN0_Handler(void){ uint8_t data[4];
  uint32_t ulIntStatus, ulIDStatus;
	unsigned long temp;
  int i;
  tCANMsgObject xTempMsgObject;
  xTempMsgObject.pucMsgData = data;
  ulIntStatus = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE); // cause?
  if(ulIntStatus & CAN_INT_INTID_STATUS){  // receive?
    ulIDStatus = CANStatusGet(CAN0_BASE, CAN_STS_NEWDAT);
    for(i = 0; i < 32; i++){    //test every bit of the mask
      if( (0x1 << i) & ulIDStatus){  // if active, get data
        CANMessageGet(CAN0_BASE, (i+1), &xTempMsgObject, true);
        if(xTempMsgObject.ulMsgID == Ping1_ID){
          //temp = (data[0]<<24)&0xFF000000 + (data[1]<<16)&0x00FF0000 + (data[2]<<8)&0x0000FF00 + data[3]&0x000000FF;
					temp = (data[0]<<24) + (data[1]<<16) + (data[2]<<8) + data[3];
					Mailbox = temp;
        }else if(xTempMsgObject.ulMsgID == Ping2_ID){
					temp = (data[0]<<24) + (data[1]<<16) + (data[2]<<8) + data[3];
					Mailbox2 = temp;
				}else if(xTempMsgObject.ulMsgID == Ping3_ID){
					temp = (data[0]<<24) + (data[1]<<16) + (data[2]<<8) + data[3];
					Mailbox3 = temp;
				}else if(xTempMsgObject.ulMsgID == Ping4_ID){
					temp = (data[0]<<24) + (data[1]<<16) + (data[2]<<8) + data[3];
					Mailbox4 = temp;
				}else if(xTempMsgObject.ulMsgID == Button1_ID){
					button1Pressed = 1;
					//add thread that compensates for wall hit
				}else if(xTempMsgObject.ulMsgID == Button2_ID){
					button2Pressed = 1;
					//add thread that compensates for wall hit
				}else if(xTempMsgObject.ulMsgID == IR_ID){
					temp = (data[0]<<24) + (data[1]<<16) + (data[2]<<8) + data[3];
					Mailbox5 = temp;
				}
      }
    }
  }
  CANIntClear(CAN0_BASE, ulIntStatus);  // acknowledge
}

//Set up a message object.  Can be a TX object or an RX object.
void static CAN0_Setup_Message_Object( uint32_t MessageID, \
                                uint32_t MessageFlags, \
                                uint32_t MessageLength, \
                                uint8_t * MessageData, \
                                uint32_t ObjectID, \
                                tMsgObjType eMsgType){
  tCANMsgObject xTempObject;
  xTempObject.ulMsgID = MessageID;          // 11 or 29 bit ID
  xTempObject.ulMsgLen = MessageLength;
  xTempObject.pucMsgData = MessageData;
  xTempObject.ulFlags = MessageFlags;
  CANMessageSet(CAN0_BASE, ObjectID, &xTempObject, eMsgType);
}
// Initialize CAN port
void CAN0_Open(void){
	uint32_t volatile delay;
	int32_t sr;
	
	sr = StartCritical();
  MailFlag = false;

  SYSCTL_RCGCCAN_R |= 0x00000001;  // CAN0 enable bit 0
  SYSCTL_RCGCGPIO_R |= 0x00000010;  // RCGC2 portE bit 4
  for(delay=0; delay<10; delay++){};
  GPIO_PORTE_AFSEL_R |= 0x30; //PORTE AFSEL bits 5,4
// PORTE PCTL 88 into fields for pins 5,4
  GPIO_PORTE_PCTL_R = (GPIO_PORTE_PCTL_R&0xFF00FFFF)|0x00880000;
  GPIO_PORTE_DEN_R |= 0x30;
  GPIO_PORTE_DIR_R |= 0x20;
      
  CANInit(CAN0_BASE);
  CANBitRateSet(CAN0_BASE, 80000000, CAN_BITRATE);
  CANEnable(CAN0_BASE);
// make sure to enable STATUS interrupts
  CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);
// Set up filter to receive these IDs
// in this case there is just one type, but you could accept multiple ID types
  CAN0_Setup_Message_Object(Ping1_ID, MSG_OBJ_RX_INT_ENABLE, 4, NULL, Ping1_ID, MSG_OBJ_TYPE_RX);
	CAN0_Setup_Message_Object(Ping2_ID, MSG_OBJ_RX_INT_ENABLE, 4, NULL, Ping2_ID, MSG_OBJ_TYPE_RX);
	CAN0_Setup_Message_Object(Ping3_ID, MSG_OBJ_RX_INT_ENABLE, 4, NULL, Ping3_ID, MSG_OBJ_TYPE_RX);
	CAN0_Setup_Message_Object(Ping4_ID, MSG_OBJ_RX_INT_ENABLE, 4, NULL, Ping4_ID, MSG_OBJ_TYPE_RX);
	CAN0_Setup_Message_Object(Button1_ID, MSG_OBJ_RX_INT_ENABLE, 4, NULL, Button1_ID, MSG_OBJ_TYPE_RX);
	CAN0_Setup_Message_Object(Button2_ID, MSG_OBJ_RX_INT_ENABLE, 4, NULL, Button2_ID, MSG_OBJ_TYPE_RX);
	CAN0_Setup_Message_Object(IR_ID, MSG_OBJ_RX_INT_ENABLE, 4, NULL, IR_ID, MSG_OBJ_TYPE_RX);
  NVIC_EN1_R = (1 << (INT_CAN0 - 48)); //IntEnable(INT_CAN0);
		
	EndCritical(sr);
  return;
}

// send 4 bytes of data to other microcontroller 
void CAN0_SendData(uint8_t data[4]){
// in this case there is just one type, but you could accept multiple ID types
 // CAN0_Setup_Message_Object(XMT_ID, NULL, 4, data, XMT_ID, MSG_OBJ_TYPE_TX);
}

// Returns true if receive data is available
//         false if no receive data ready
int CAN0_CheckMail(void){
  return MailFlag;
}
// if receive data is ready, gets the data and returns true
// if no receive data is ready, returns false
int CAN0_GetMailNonBlock(uint8_t data[4]){
	unsigned long temp;
  if(MailFlag){
		temp = (data[0]<<24) + (data[1]<<16) + (data[2]<<8) + data[3];
    return true;
  }
  return false;
}
// if receive data is ready, gets the data 
// if no receive data is ready, it waits until it is ready
void CAN0_GetMail(uint8_t data[4]){
	
  while(MailFlag==false){};
	
	// need to protect this data write
  data[0] = RCVData[0];
  data[1] = RCVData[1];
  data[2] = RCVData[2];
  data[3] = RCVData[3];
  MailFlag = false;	
}

