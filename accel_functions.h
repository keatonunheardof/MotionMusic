// accel_functions.h 

#include "twi_master.h"  //my TWCR_START, STOP, NACK, RNACK, SEND

#define ACCEL_ADDRESS 0x6A                    //ACCEL-0, address pin floating
#define ACCEL_WRITE (ACCEL_ADDRESS | TW_WRITE) //LSB is a zero to write
#define ACCEL_READ  (ACCEL_ADDRESS | TW_READ)  //LSB is a one to read
#define ACCEL_PTR_TEMP          0x00          //ACCEL temperature address
#define ACCEL_PTR_CONFIG        0x01          //ACCEL configuration address
#define ACCEL_PTR_CTRL_STATUS   0x04          //ACCEL control and status register
#define ACCEL_CONFIG_VALUE0     0x60          //no pwr dwn, disbl alert, no one shot: config reg
#define ACCEL_CONFIG_VALUE1     0xE0          //no timeout, max resolution: for ctl/status reg

//special functions for accel temperature sensor

void init_accel();
uint16_t read_accel(uint8_t celsius);
uint16_t get_remote_temp(uint8_t celsius);
uint16_t get_local_temp(uint8_t celsius);
  
