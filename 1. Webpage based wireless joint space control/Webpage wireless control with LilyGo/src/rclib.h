#ifndef __RCLIB_H__
#define __RCLIB_H__
#include <Arduino.h>
#include "driver/twai.h"

#include "config.h" //configure pinout
#include <HardwareSerial.h>
#include <SPI.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <WiFi.h>

extern int num1;
extern int num2;
extern int Speed;
extern int Position;
extern int panduan;
extern int panduan1;
extern int yanshi;
extern int ii, j;
extern int c[99];
extern float a1, a2, a3, a4, a5, a6, a7, a8, p, q;
extern float b1[99], b2[99], b3[99], b4[99], b5[99], b6[99], b7[99], b8[99];
extern int txid;
#define LILYGO

#ifdef LILYGO
#define TX_PIN 27
#define RX_PIN 26
#else
#define TX_PIN 17
#define RX_PIN 18
#endif

bool rc_init();
void rc_estop(int);                     // set estop
void rc_set_status(int, uint8_t);       // set motor status
int rc_read_status(int);                // read motor status
void rc_set_mode(int, uint8_t);         // set motor mode
int rc_read_mode(int);                  // read motor mode
void rc_set_zp(int, uint8_t);           // set current position as zero point
void rc_set_PID(int, uint8_t, float);   // set PID parameters
float rc_read_PID(int, uint8_t);        // read PID paramaters
void rc_set_lim(int, uint8_t, float);   // set limitations
float rc_read_lim(int, uint8_t);        // read limitations
void rc_spr(int, float);                // single point run
void rc_spt(int, float, float);         // single point trajectory
void rc_set_ctp(int, uint16_t, float);  // set continuous traj position data
void rc_set_ctv(int, uint16_t, float);  // set continuous traj speed data
void rc_set_ctmf(int, uint16_t, float); // set continuous traj current data
void rc_tdr(int, uint16_t);             // set to run as specific trajectory
void rc_record(int, uint16_t);          // record data to the assigned index position
float rc_read_rd(int, uint8_t);         // read running data
void rc_set_CAN_ID(int, uint8_t);       // set up CAN ID
void rc_reset(int);                     // reset all settings

#endif