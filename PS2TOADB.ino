/*
 * Copyright (c) 1999 Apple Computer, Inc. All rights reserved.
 *
 * @APPLE_LICENSE_HEADER_START@
 * 
 * Copyright (c) 1999-2003 Apple Computer, Inc.  All Rights Reserved.
 * 
 * This file contains Original Code and/or Modifications of Original Code
 * as defined in and that are subject to the Apple Public Source License
 * Version 2.0 (the 'License'). You may not use this file except in
 * compliance with the License. Please obtain a copy of the License at
 * http://www.opensource.apple.com/apsl/ and read it before using this
 * file.
 * 
 * The Original Code and all software distributed under the License are
 * distributed on an 'AS IS' basis, WITHOUT WARRANTY OF ANY KIND, EITHER
 * EXPRESS OR IMPLIED, AND APPLE HEREBY DISCLAIMS ALL SUCH WARRANTIES,
 * INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE, QUIET ENJOYMENT OR NON-INFRINGEMENT.
 * Please see the License for the specific language governing rights and
 * limitations under the License.
 * 
 * @APPLE_LICENSE_HEADER_END@
 */
/*   Copyright (c) 1991 NeXT Computer, Inc.  All rights reserved. 
 *      Copyright (c) 2013 Michel Gallant. All rights reserved. 
 *
 * adb_bus.h - Architecture-indpendent ADB bus defintions.
 *
 * HISTORY
 * 5-Jan-2013   Michel Gallant at Foulab
 *      Created. 
 */

#include "PS2Mouse.h"
#include "PS2KeyAdvanced.h"

#include "ADBKeyTable.h"


/* Keyboard constants  Change to suit your Arduino
   define pins used for data and clock from keyboard */
#define DATAPIN 3
#define IRQPIN  2

// Ps2 Mouse
#define PS2DATA 6
#define PS2CLOCK 7

// Adb bus
#define ADB 1

//State machine states
#define INIT 1
#define ATTENTION 2
#define SYNC 3
#define GOTSTOPBIT 4

// Input compare interrupt on
#define ENABLE_CAP_INT TIMSK1 |= 0x20;
#define DISABLE_CAP_INT TIMSK1 &= ~0x20;

//ADB commands
#define TALK 3
#define LISTEN 2
#define FLUSH 0

// PS2 Mouse button butmasks
#define LBUTTON 1
#define RBUTTON 2

//Raw logic level of input.
volatile byte level;

//Time since last level change
volatile unsigned int time;

// By default, a mouse has address 3.
// This can be changed by the computer.
// It is stored in register 3.
#define INIT_DEVICE_ADDRESS 3
#define INIT_HANDLER_ID 1

// By default, a keyboard has address 2.
// This can be changed by the computer.
// It is stored in register 3.
#define INIT_KB_DEVICE_ADDRESS 2
#define INIT_KB_HANDLER_ID 1


// Data registers. Register 0 is mouse movement data.
// Register 3 is device address and status bits.
// Registers 1 and 2 are unused on most devices.
volatile byte reg0[2];
//byte reg3[2];

volatile byte kb_reg0[2];
volatile byte kb_reg2[2];

volatile byte state = INIT;

//TODO: Why does this seem backwards from the cmdBits struct below?
typedef struct {

  byte reserved0:1,     // bit 15, always 0
  exceptionalEvent:1,   // bit 14
  serviceRequestEnable:1, // bit 13
  reserved1:1,      // bit 12, always 0
  deviceAddress:4;    // bits 11..8
  byte handlerId:8;     // bits 7..0
  
} adbRegister3bits;

typedef struct {
  byte b[2];
} adbRegister3byte;

typedef union {
  adbRegister3bits  bits;
  adbRegister3byte  bytes;
} adbRegister3;

typedef struct{
  byte reg:2,            // Bits 0..1
  cmd:2,            // Bits 2..3
  address:4;   // Bits 4..7
}cmdBits;

typedef struct{
  byte cmdByte;
}cmdByte;

typedef union{
  cmdBits bits;
  cmdByte bytes;
}cmd;

//typedef struct{
  

adbRegister3 reg3;
adbRegister3 kb_reg3;

#define TLEN 256
//volatile unsigned int t[TLEN];
//volatile unsigned int l[TLEN];
//volatile unsigned int a[TLEN];
//volatile unsigned int* volatile times; 
//volatile unsigned int* endtimes; 
//volatile unsigned int* volatile levels;
//volatile unsigned int* endlevels;

volatile int working;

volatile cmd command;

volatile unsigned int syncs = 0;
volatile unsigned int attens = 0;
volatile unsigned long int acalls = 0;
volatile unsigned long int aloops = 0;

volatile byte pending = 0;
volatile byte kb_pending = 0;

PS2Mouse mouse( PS2CLOCK, PS2DATA );
PS2KeyAdvanced keyboard;


void setup(){
  /*times = t;
  endtimes = times + TLEN;
  levels = l;
  endlevels = levels + TLEN;*/
  Serial.begin(115200);
  // mouse.initialize();

  // Configure the keyboard library
  keyboard.begin( DATAPIN, IRQPIN );

  reg3.bytes.b[0] = 0;
  reg3.bytes.b[1] = 0;
  reg3.bits.deviceAddress = INIT_DEVICE_ADDRESS;
  reg3.bits.handlerId = INIT_HANDLER_ID;
//  reg3.bits.exceptionalEvent = 1;
//  reg3.bits.serviceRequestEnable = 1;
  Serial.print("Starting ADB. Device address: ");
  Serial.print( reg3.bits.deviceAddress );
  Serial.print(" Handler ID: " );
  Serial.println( reg3.bits.handlerId );
  kb_reg3.bytes.b[0] = 0;
  kb_reg3.bytes.b[1] = 0;
  kb_reg3.bits.deviceAddress = INIT_KB_DEVICE_ADDRESS;
  kb_reg3.bits.handlerId = INIT_KB_HANDLER_ID;
//  kb_reg3.bits.exceptionalEvent = 1;
//  kb_reg3.bits.serviceRequestEnable = 1;
  Serial.print("Starting KB ADB. Device address: ");
  Serial.print( kb_reg3.bits.deviceAddress );
  Serial.print(" KB Handler ID: " );
  Serial.println( kb_reg3.bits.handlerId );
  // Set data direction in
  DDRB &= ~ADB;
  // Enable weak pullup
  PORTB |= ADB;
  // Set up timer-counter #1 
  TCCR1A = 0x00;
  // Input compare noise canceller on,
  // Falling edge input compare interrupt, 
  // clock prescaler = 8
  TCCR1B = 0x82;
  TCCR1C = 0x00;
  TCNT1= 0x00;
  // Input compare interrupt on
  ENABLE_CAP_INT
  
  // If ADB input is high, set falling-edge interrupt.
  // Otherwise, set rising-edge input 
  //(level = PORTB & ADB);//? TCCR1B &= 0xBF: TCCR1B |= 0x40;
  TCCR1B |= 0x40;
  working = 1;
  
  Serial.println( "Setup done.");
}

void loop(){

  //int data[3];
  //printMouseReport(data);
  //delay(200);
  //Serial.println( "Waiting for HIGH" );
  if( !working ){
    Serial.println( "DONE WORKING");
    /*for( int i =0; i < TLEN; i++){
      Serial.print( " Level : " );
      Serial.print( l[i] ? "H" : "L" );
      Serial.print( " Time: " );
      Serial.println( t[i] );
  
    }*/
    Serial.print( "Syncs, attentions, acalls, aloops:: " );
    Serial.print( syncs );
    Serial.print(", ");
    Serial.print( attens );
    Serial.print(", ");
    Serial.print( acalls );
    Serial.print(", ");
    Serial.println( aloops );
    while(1);
  }

  getHigh();
  getAttentionSignal();
  if( state != ATTENTION ) return;
  getSyncSignal();
  if( state != SYNC ) return;
  // Get command byte

  for( int i = 0; i < 8; i++ ){
    byte b = getBit();
    if ( b > 1 ) return;
    command.bytes.cmdByte  = ( command.bytes.cmdByte << 1 ) + b;
  }

  /*command.bits.reg = 2;
  Serial.println( command.bits.address );
  Serial.println( command.bits.cmd );
  Serial.println( command.bits.reg );
  Serial.println( command.bytes.cmdByte, HEX );
  Serial.println( reg3.bits.deviceAddress );
  Serial.print( reg3.bytes.b[0], HEX );
  Serial.println( reg3.bytes.b[1], HEX );*/
  // mouse
  if( command.bits.address == reg3.bits.deviceAddress ){
    static bool mo_first_time = true;
    if( command.bits.cmd == TALK && command.bits.reg == 0 && pending ){
      getStopAndTlt();
      sendMouse();
      // Serial.println("MO: reg0");
    }
    else if( command.bits.cmd == TALK && command.bits.reg == 3 && mo_first_time )
    {
      mo_first_time = false;
      getStopAndTlt();
      sendMouseReg3();
      Serial.println("MO: reg3");
    }
    else if(command.bits.cmd == LISTEN)
    {
      uint8_t l_data;

      getHigh();
      getAttentionSignal();
      if( state != ATTENTION ) return;
      getSyncSignal();
      if( state != SYNC ) return;
      // Get command byte
      for( int i = 0; i < 8; i++ ){
        byte b = getBit();
        if ( b > 1 ) return;
        l_data  = ( l_data << 1 ) + b;
      }      
      
      Serial.print("MO: LISTEN reg: ");
      Serial.print( command.bits.reg, HEX );
      Serial.print(" ,cmd: ");
      Serial.print( command.bits.cmd, HEX );
      Serial.print(" ,addr: ");
      Serial.print( command.bits.address, HEX );
      Serial.print(" ,data: ");
      Serial.println( l_data, HEX );
        
    }
    else if(command.bits.cmd == FLUSH)
    {
      uint8_t l_data;

      Serial.println("MO: FLUSH");

      getHigh();
      getAttentionSignal();
      if( state != ATTENTION ) return;
      getSyncSignal();
      if( state != SYNC ) return;
      // Get command byte
      for( int i = 0; i < 8; i++ ){
        byte b = getBit();
        if ( b > 1 ) return;
        l_data  = ( l_data << 1 ) + b;
      }      
      
      Serial.print("MO: FLUSH reg: ");
      Serial.print( command.bits.reg, HEX );
      Serial.print(" ,cmd: ");
      Serial.print( command.bits.cmd, HEX );
      Serial.print(" ,addr: ");
      Serial.print( command.bits.address, HEX );
      Serial.print(" ,data: ");
      Serial.println( l_data, HEX );
        
    }
    else if( command.bits.reg != 0)
    {
      Serial.print("MO: other reg: ");
      Serial.print( command.bits.reg, HEX );
      Serial.print(" ,cmd: ");
      Serial.print( command.bits.cmd, HEX );
      Serial.print(" ,addr: ");
      Serial.println( command.bits.address, HEX );
    }
    
    //TODO: Handle LISTEN and FLUSH here
    
  }

  // keyboard
  if( command.bits.address == kb_reg3.bits.deviceAddress ){
    static bool kb_first_time = true;
    if( command.bits.cmd == TALK && command.bits.reg == 0 && kb_pending ){
      getStopAndTlt();
      sendKeybReg0();
      Serial.println("KB: reg0");
    }
    else if( command.bits.cmd == TALK && command.bits.reg == 2 && kb_pending ){
      getStopAndTlt();
      sendKeybReg2();
      Serial.println("Kb: reg2");
    }
    else if( command.bits.cmd == TALK && command.bits.reg == 3 && kb_first_time ){
      kb_first_time = false;
      getStopAndTlt();
      sendKeybReg3();
      Serial.println("KB: reg3");
    }    
    else if(command.bits.cmd == LISTEN)
    {
      uint8_t l_data;

      getHigh();
      getAttentionSignal();
      if( state != ATTENTION ) return;
      getSyncSignal();
      if( state != SYNC ) return;
      // Get command byte
      for( int i = 0; i < 8; i++ ){
        byte b = getBit();
        if ( b > 1 ) return;
        l_data  = ( l_data << 1 ) + b;
      }  
            
      Serial.print("KB: LISTEN reg: ");
      Serial.print( command.bits.reg, HEX );
      Serial.print(" ,cmd: ");
      Serial.print( command.bits.cmd, HEX );
      Serial.print(" ,addr: ");
      Serial.print( command.bits.address, HEX );
      Serial.print(" ,data: ");
      Serial.println( l_data, HEX );
    }
    else /* if( command.bits.reg != 0) */
    {
      Serial.print("KB: reg: ");
      Serial.print( command.bits.reg, HEX );
      Serial.print(" ,cmd: ");
      Serial.print( command.bits.cmd, HEX );
      Serial.print(" ,addr: ");
      Serial.println( command.bits.address, HEX );
    }
    
    //TODO: Handle LISTEN and FLUSH here
    
  }

  
  //TODO: Handle global commands here
  
  
  // Check for new movement/ buttons from the mouse.
  // If there is data pending, don't check, because that would
  // overwrite the currently pending data. The mouse
  // should accumulate any extra movement that happens
  // during this time.
  if( !pending ) checkMouse();


  if( !kb_pending && keyboard.available( ) )
  {
    // read the next key
    uint16_t c = keyboard.read( );
    if( c > 0 )
    {
      uint8_t mac_kb = ADB_KEY_IGNORE;
      int length = sizeof( adb_key ) / sizeof( adb_key[ 0 ] );
      for( int index = 0; index < length; index++ )      
      {
        if( adb_key[index][1] == (c & 0xFF) )
        {
          mac_kb = adb_key[index][0];
          index = length;
        }
      }

      if( ADB_KEY_IGNORE != mac_kb )
      {
        kb_reg0[0] = 0xFF;
        kb_reg0[1] = mac_kb | ((c&PS2_BREAK)>>8);

        kb_reg2[0] = 0; // to modify
        uint8_t kb_led = keyboard.getLock();
        kb_reg2[1] = 0xFF;
        if(!(kb_led & PS2_LOCK_CAPS)) kb_reg2[1] &= ~2;
        if(!(kb_led & PS2_LOCK_SCROLL)) kb_reg2[1] &= ~4;
        if(!(kb_led & PS2_LOCK_NUM)) kb_reg2[1] &= ~1;
        kb_pending = 1;
      }
      
      Serial.print( "Value " );
      Serial.print( c, HEX );
      Serial.print( " - Status Bits " );
      Serial.print( c >> 8, HEX );
      Serial.print( "  Code " );
      Serial.print( c & 0xFF, HEX );
      Serial.print( ", Mac Code " );
      Serial.println( kb_reg0[1], HEX );
    }
  }  
  /*Serial.print( "cb " );
  Serial.println( command.bytes.cmdByte, HEX );*/
  
}

void printMouseReport(int data[]){
  mouse.report(data);
  Serial.print( "RAW: ");
  Serial.print( data[0], HEX );
  Serial.print(" Status: X - ");
  Serial.print( data[0]& 1<<4 ? data[1] : -data[1] );
  Serial.print(" Y - ");
  Serial.print( data[0] & 1<<5 ? data[2] : -data[2] );
  Serial.print(" L - ");
  Serial.print( (data[0] & 1<<0) ? "D" : "U" );
  Serial.print(" R - ");
  Serial.println( data[0] & 1<<1 ? "D" : "U" );
}

void getHigh(){
  
  while( !level );
  return;

  //asm("cli");
}

void getAttentionSignal(){
  while( !level ){
    aloops++;
  };
  acalls++;

  //TODO: Attention time on the IIgs seems longer.
  //Increase to 827 or more... maybe 850?
  if( time > 776 && time < 850 ){
    state = ATTENTION;
    attens++;
  }
}

void getSyncSignal(){
  while( level );
  if( time > 60/*67*/ && time < 80 ){
    state = SYNC;
    syncs++;
  }
}

byte getBit(){
  // b is the candidate - what we suspect the bit will be.
  // If both the high and low times match the protocol spec,
  // return b, otherwise return error.
  byte b;
  while( !level );
  if( time < 50 ){
    b = 1;
  }else if( time >= 50 && time < 72 ){
    b = 0;
  }else{
    return 2;
  }
  
  while( level );
  if( b == 1 && time >= 50 && time < 72 ){
    return 1;
  }else if( b == 0 && time < 50 ){
    return 0;
  }else{
    return 2;
  }
  
}

ISR(TIMER1_CAPT_vect){
  // Disable interrupts
  asm("cli");
  // Clear counter register
  TCNT1 = 0;
  // Save line level to level value
  //level = PORTB & ADB;
  // Save match register to time value
  // The timer ticks twice per microsecond,
  // so dividing by two gives us actual microseconds
  time = ICR1 >> 1;
  // Set next interrupt to happen on appropriate
  // rising or falling edge
  //level ? TCCR1B &= 0xBF: TCCR1B |= 0x40;
  level = (TCCR1B & 0x40);
  TCCR1B ^= 0x40;
  //*(times++) = time;
  //*(levels++) = !level;
  /*if( levels == endlevels ){
    working = 0;
    TIMSK1&= ~0x20;
    //times = t;
    //levels = l;
  }*/
  // Enable interrupts
  asm("sei");
  //Serial.println( time );
  
  
}

void getStopAndTlt(){

  while( !level );
  //If we have a 0, this is the stop bit
  if( time > 50 && time < 65 ){
    state = GOTSTOPBIT;
  // If time is around 300 uS low, then some other device
  // has requested service and we can't transmit
  }else if( time > 280 && time < 330 ){
    state = INIT;
  }else{
    //TODO: probably should be COLLISION not INIT
    state = INIT;
  }
  
  // Delay until Tlt is reached
  // 200mS, which is 400 counter counts
  while( level && TCNT1 < 400 );

}

void sendMouse(){
  // Disable timer interrupts
  DISABLE_CAP_INT
  //start bit
  sendBit( 1 );
  sendByte( reg0[0] );
  sendByte( reg0[1] );
  //stop bit
  sendBit( 0 );
  pending = 0;
  //re-enable timer interrupt
  ENABLE_CAP_INT
}

void sendMouseReg3(){
  // Disable timer interrupts
  DISABLE_CAP_INT
  //start bit
  sendBit( 1 );
  sendByte( 0x01 /*reg3.bytes.b[0]*/ );
  sendByte( 0x03 /*reg3.bytes.b[1]*/ );
  //stop bit
  sendBit( 0 );
  //re-enable timer interrupt
  ENABLE_CAP_INT
}

void sendKeybReg0(){
  // Disable timer interrupts
  DISABLE_CAP_INT
  //start bit
  sendBit( 1 );
  sendByte( kb_reg0[0] );
  sendByte( kb_reg0[1] );
  //stop bit
  sendBit( 0 );
  kb_pending = 0;
  //Serial.println("M");
  //re-enable timer interrupt
  ENABLE_CAP_INT
}

void sendKeybReg2(){
  // Disable timer interrupts
  DISABLE_CAP_INT
  //start bit
  sendBit( 1 );
  sendByte( kb_reg2[0] );
  sendByte( kb_reg2[1] );
  //stop bit
  sendBit( 0 );
  kb_pending = 0;
  //Serial.println("M");
  //re-enable timer interrupt
  ENABLE_CAP_INT
}

void sendKeybReg3(){
  // Disable timer interrupts
  DISABLE_CAP_INT
  //start bit
  sendBit( 1 );
  sendByte( 0x01 /*kb_reg3.bytes.b[0]*/ );
  sendByte( 0x02 /*kb_reg3.bytes.b[1]*/ );
  //stop bit
  sendBit( 0 );
  //re-enable timer interrupt
  ENABLE_CAP_INT
}

//TODO: ADD COLLISION DETECTION HERE!
inline void sendBit( byte b ){
  if(b){
    goLow();
    delayMicroseconds(33);
    goHigh();
    delayMicroseconds(61);
  }else{
    goLow();
    delayMicroseconds(61);
    goHigh();
    delayMicroseconds(33);
  }    
}

void sendByte( byte b ){
  // Send a byte MSB first.
  for( int i = 0; i < 8; i ++ ){
    sendBit( b & 0x80  );
    b = b << 1;
  }
}

inline void goLow(){
    //Turn off pullup
    PORTB &= ~ADB;
    // Set direction output
    DDRB |= ADB;
}

inline void goHigh(){
  // Set direction input
  DDRB &= ~ADB;
  //Turn on pullup
  PORTB |= ADB;
}

// Check for new movement/ buttons from the mouse.
// Transfer to to register 0 and set the 
// pending flag if there is new data.
void checkMouse(){
  static byte oldbuttons = 0;
  static byte buttons = 0;
  int data[3];

  // Ignore the ADB bus while we read the PS2 mouse.
  // We don't care what's happening there.
  DISABLE_CAP_INT
  //2 mS
  mouse.report(data);
  // Looks like the mouse library already sign-extends x and y values
  // Now we just downconvert them to 7 bit values,
  // capping them if they would otherwise overflow 7 bits
  if( data [1] < -64 ) data[1]=-64;
  else if( data[1] > 63 ) data[1] = 63;
  // Vertical axis seems reversed from PS2 mouse
  data[2] = -data[2];
  if( data [2] < -64 ) data[2]=-64;
  else if( data[2] > 63 ) data[2] = 63;  
  // Move Y value into reg 0 byte 1
  reg0[0] = data[2] & 0x7F;
  // Move x value into reg 0 byte 2
  reg0[1] = data[1] & 0x7F;
  // TODO: check for button changes here
  buttons = data[0] & 0x03;
  if( reg0[0] || reg0[1] || buttons != oldbuttons ){
    pending = 1;
  }
  if ( !(buttons & LBUTTON) ) reg0[0] |= 0x80;
  if ( !(buttons & RBUTTON) ) reg0[1] |= 0x80;
  
  oldbuttons = buttons;
  ENABLE_CAP_INT
}
