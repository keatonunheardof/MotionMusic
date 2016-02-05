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
#include "spi_interface.h"
#include "LSM6DS3.h"

#define ACCEL_RANGE 8
#define GYRO_RANGE  1000
#define IDENTIFIER 	0xAA
#define WINDOW_SIZE 50
#define NUM_AVGS 	50

uint8_t timer_ovf_cnt = 0;

ISR(TIMER1_OVF_vect){
		timer_ovf_cnt++;
}

//*****************************************************************************
// Main Program
//*****************************************************************************

int main(){

		// Initialize UART
		UART_Init(MY_UBRR);

		// Initialize SPI
		SPI_Init();

		// Initialize LSM6DS3
		init_LSM6DS3();

		// Initialize Accelerometer 
		init_accel();

		// Initialize Gyroscope 
		init_gyro();

		// Initialize timer/counter1 for normal mode, no prescaller
		TCCR1B = (1<<CS10);

		// Enable overflow interrupt
		TIMSK1 |= (1<<TOIE1);

		// Enable interrupts
		sei();

		volatile int16_t X_XL[WINDOW_SIZE], Y_XL[WINDOW_SIZE], Z_XL[WINDOW_SIZE];
		volatile int16_t X_G[WINDOW_SIZE], Y_G[WINDOW_SIZE], Z_G[WINDOW_SIZE];
		volatile int16_t diff_X_XL[WINDOW_SIZE], diff_Y_XL[WINDOW_SIZE], diff_Z_XL[WINDOW_SIZE];
		volatile int16_t avg_diff_X_XL, avg_diff_Y_XL, avg_diff_Z_XL;
		volatile int16_t avg_X_G, avg_Y_G, avg_Z_G;

		volatile double x_accel[WINDOW_SIZE], y_accel[WINDOW_SIZE], z_accel[WINDOW_SIZE];
		volatile int16_t x_gyro[WINDOW_SIZE], y_gyro[WINDOW_SIZE], z_gyro[WINDOW_SIZE];
		volatile int16_t roll, pitch;

		volatile uint16_t time[2];
		volatile uint16_t diff_time;

		char str[30];

		uint16_t cnt = 0;	
		uint8_t i;

		time[0] = TCNT1;
		time[1] = TCNT1;
		
		while(1){

				time[1] = time[0];
				time[0] = TCNT1;

				if(time[0] < time[1])
						diff_time = (uint16_t)((uint32_t)time[0]+65536 - (uint32_t)time[1]);
				else
						diff_time = (time[0] - time[1]);


				// Wait while data is not available
				while(!gyro_data_avail()){}

				// Get new accelerometer data 
				X_XL[cnt%WINDOW_SIZE] = SPI_Receive(OUTX_L_XL);
				X_XL[cnt%WINDOW_SIZE] |= (SPI_Receive(OUTX_H_XL)<<8);
				Y_XL[cnt%WINDOW_SIZE] = SPI_Receive(OUTY_L_XL);
				Y_XL[cnt%WINDOW_SIZE] |= (SPI_Receive(OUTY_H_XL)<<8);
				Z_XL[cnt%WINDOW_SIZE] = SPI_Receive(OUTZ_L_XL);
				Z_XL[cnt%WINDOW_SIZE] |= (SPI_Receive(OUTZ_H_XL)<<8);

				// Get new gyroscope data 
				X_G[cnt%WINDOW_SIZE] = SPI_Receive(OUTX_L_G);
				X_G[cnt%WINDOW_SIZE] |= (SPI_Receive(OUTX_H_G)<<8);
				Y_G[cnt%WINDOW_SIZE] = SPI_Receive(OUTY_L_G);
				Y_G[cnt%WINDOW_SIZE] |= (SPI_Receive(OUTY_H_G)<<8);
				Z_G[cnt%WINDOW_SIZE] = SPI_Receive(OUTZ_L_G);
				Z_G[cnt%WINDOW_SIZE] |= (SPI_Receive(OUTZ_H_G)<<8);

				// Get change in acceleration 
				diff_X_XL[cnt%WINDOW_SIZE] = (X_XL[(cnt-1)%WINDOW_SIZE] -  X_XL[cnt%WINDOW_SIZE]) * ACCEL_RANGE / pow(2,6);
				diff_Y_XL[cnt%WINDOW_SIZE] = (Y_XL[(cnt-1)%WINDOW_SIZE] -  Y_XL[cnt%WINDOW_SIZE]) * ACCEL_RANGE / pow(2,6);
				diff_Z_XL[cnt%WINDOW_SIZE] = (Z_XL[(cnt-1)%WINDOW_SIZE] -  Z_XL[cnt%WINDOW_SIZE]) * ACCEL_RANGE / pow(2,6);		

				// Calculate current acceleration
				x_accel[cnt%WINDOW_SIZE] = X_XL[cnt%WINDOW_SIZE] * ACCEL_RANGE / pow(2,15);
				y_accel[cnt%WINDOW_SIZE] = Y_XL[cnt%WINDOW_SIZE] * ACCEL_RANGE / pow(2,15);
				z_accel[cnt%WINDOW_SIZE] = Z_XL[cnt%WINDOW_SIZE] * ACCEL_RANGE / pow(2,15);

				pitch = (int16_t)(atan(x_accel[cnt%WINDOW_SIZE] / sqrt(pow(y_accel[cnt%WINDOW_SIZE],2) + pow(z_accel[cnt%WINDOW_SIZE],2))) * 180 / M_PI);
				roll = (int16_t)(atan( y_accel[cnt%WINDOW_SIZE] / sqrt(pow(x_accel[cnt%WINDOW_SIZE],2) + pow(z_accel[cnt%WINDOW_SIZE],2))) * 180 / M_PI);

				// Calculate current angular rate
				x_gyro[cnt%WINDOW_SIZE] = (X_G[cnt%WINDOW_SIZE] / pow(2,15)) * GYRO_RANGE;
				y_gyro[cnt%WINDOW_SIZE] = (Y_G[cnt%WINDOW_SIZE] / pow(2,15)) * GYRO_RANGE;
				z_gyro[cnt%WINDOW_SIZE] = (Z_G[cnt%WINDOW_SIZE] / pow(2,15)) * GYRO_RANGE;		

				// Clear last average
				avg_diff_X_XL = 0;
				avg_diff_Y_XL = 0;
				avg_diff_Z_XL = 0;

				// Get sum of last NUM_AVGS accelerations
				for(i = 0; i < NUM_AVGS; i++){

						avg_diff_X_XL += diff_X_XL[(cnt%WINDOW_SIZE) - i];
						avg_diff_Y_XL += diff_Y_XL[(cnt%WINDOW_SIZE) - i];
						avg_diff_Z_XL += diff_Z_XL[(cnt%WINDOW_SIZE) - i];

				}

				// Take average of last NUM_AVGS accelerations and pitch and roll calculations
				avg_diff_X_XL /= NUM_AVGS;
				avg_diff_Y_XL /= NUM_AVGS;
				avg_diff_Z_XL /= NUM_AVGS; 				


				// Print Orientation and Motion data
				sprintf(str,"%02X\t\t%i\t%i\t\t%i\t%i\t%i\t\t%i\t%i\t%i\t\t%u\r\n", IDENTIFIER,\
								pitch, roll,\ 
								avg_diff_X_XL, avg_diff_Y_XL, avg_diff_Z_XL,\
								x_gyro[cnt%WINDOW_SIZE], y_gyro[cnt%WINDOW_SIZE], z_gyro[cnt%WINDOW_SIZE],\
								diff_time);
				UART_Transmit_String(str);

				cnt++;
				timer_ovf_cnt = 0;
		}

		return 0;
}

