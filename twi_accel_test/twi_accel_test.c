#define F_CPU       8000000UL
#define BAUD        19200UL 
#define MY_UBRR     F_CPU/(16*BAUD)-1

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "uart_interface.h"
#include "twi_interface.h"
#include "LSM6DS3.h"

#define ACCEL_RANGE 8
#define GYRO_RANGE  1000
#define IDENTIFIER 	0xAA
#define NUM_AVGS 	50


//*****************************************************************************
// Main Program
//*****************************************************************************

int main(){

    volatile int16_t X_XL, Y_XL, Z_XL;
    volatile int16_t X_G, Y_G, Z_G;

    volatile double  x_accel_dbl, y_accel_dbl, z_accel_dbl;
    volatile int16_t x_accel, y_accel, z_accel;
    volatile int16_t x_gyro, y_gyro, z_gyro;
    volatile int16_t roll, pitch;

    char str[30];
    uint8_t rd_buf[2];
    
    // Initialize UART
    UART_Init(MY_UBRR);

    // Initialize TWI
    init_twi();

    // Enable interrupts
    sei();
    
    // Initialize LSM6DS3
    init_LSM6DS3();

    // Initialize Accelerometer 
    init_accel();

    // Initialize Gyroscope 
    init_gyro();

    while(1){

        // Wait while data is not available
        while(!gyro_data_avail()){}

        // Read X axis of accelerometer
        read_LSM6DS3(OUTX_L_XL, rd_buf);    // Read low byte                
        X_XL = rd_buf[0];                   // Save low byte                
        read_LSM6DS3(OUTX_H_XL, rd_buf);    // Read high byte               
        X_XL |= (rd_buf[0]<<8);             // Concatenate high and low bytes

        // Read Y axis of accelerometer
        read_LSM6DS3(OUTY_L_XL, rd_buf);    // Read low byte                                        
        Y_XL = rd_buf[0];                   // Save low byte                           
        read_LSM6DS3(OUTY_H_XL, rd_buf);    // Read high byte                                        
        Y_XL |= (rd_buf[0]<<8);             // Concatenate high and low bytes                       

        // Read Z axis of accelerometer     // Read low byte                  
        read_LSM6DS3(OUTZ_L_XL, rd_buf);    // Save low byte                                         
        Z_XL = rd_buf[0];                   // Read high byte                          
        read_LSM6DS3(OUTZ_H_XL, rd_buf);    // Concatenate high and low bytes                         
        Z_XL |= (rd_buf[0]<<8);                                      

        // Read X axis of gyroscope         // Read low byte                 
        read_LSM6DS3(OUTX_L_G, rd_buf);     // Save low byte                                           
        X_G = rd_buf[0];                    // Read high byte                            
        read_LSM6DS3(OUTX_H_G, rd_buf);     // Concatenate high and low bytes                         
        X_G |= (rd_buf[0]<<8);                                        

        // Read Y axis of gyroscope         // Read low byte                 
        read_LSM6DS3(OUTY_L_G, rd_buf);     // Save low byte                                           
        Y_G = rd_buf[0];                    // Read high byte                       
        read_LSM6DS3(OUTY_H_G, rd_buf);     // Concatenate high and low bytes       
        Y_G |= (rd_buf[0]<<8);        

        // Read Z axis of gyroscope         // Read low byte                 
        read_LSM6DS3(OUTZ_L_G, rd_buf);     // Save low byte                 
        Z_G = rd_buf[0];                    // Read high byte                
        read_LSM6DS3(OUTZ_H_G, rd_buf);     // Concatenate high and low bytes
        Z_G |= (rd_buf[0]<<8);     

        // Calculate current acceleration for pitch and roll calculations
        x_accel_dbl = X_XL * ACCEL_RANGE / pow(2,15);
        y_accel_dbl = Y_XL * ACCEL_RANGE / pow(2,15);
        z_accel_dbl = Z_XL * ACCEL_RANGE / pow(2,15);

        // Calculate pitch and roll
        pitch = (int16_t)(atan(x_accel_dbl / sqrt(pow(y_accel_dbl,2) + pow(z_accel_dbl,2))) * 180 / M_PI);
        roll = (int16_t)(atan( y_accel_dbl / sqrt(pow(x_accel_dbl,2) + pow(z_accel_dbl,2))) * 180 / M_PI);
        
        // Calculate current acceleration and scale by 100 to display to terminal
        x_accel = X_XL * (100 * ACCEL_RANGE / pow(2,15));
        y_accel = Y_XL * (100 * ACCEL_RANGE / pow(2,15));
        z_accel = Z_XL * (100 * ACCEL_RANGE / pow(2,15));

        // Calculate current angular rate
        x_gyro = (X_G / pow(2,15)) * GYRO_RANGE;
        y_gyro = (Y_G / pow(2,15)) * GYRO_RANGE;
        z_gyro = (Z_G / pow(2,15)) * GYRO_RANGE;		

        // Print Orientation and Motion data
        sprintf(str,"%i\t%i\t\t%i\t%i\t%i\t\t%i\t%i\t%i\t\r\n", pitch, roll, x_accel, y_accel, z_accel, x_gyro, y_gyro, z_gyro);
        
        UART_Transmit_String(str);
        
    }

    return 0;
}

