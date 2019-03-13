
/* 
 * MPU6050 Interfacing with PIC18F4550
 * http://www.electronicwings.com
 */
 
#include <stdio.h>
#include <stdlib.h>
#include "I2C_Master_File.h"
#include "MPU6050_res_define.h"
#include "configBits.h"

void MPU6050_Init()										/* Gyro initialization function */
{
	MSdelay(150);		/* Power up time >100ms */
	I2C_Start_Wait(0xD0);	/* Start with device write address */
	I2C_Write(SMPLRT_DIV);	/* Write to sample rate register */
	I2C_Write(0x07);	/* 1KHz sample rate */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(PWR_MGMT_1);	/* Write to power management register */
	I2C_Write(0x01);	/* X axis gyroscope reference frequency */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(CONFIG);	/* Write to Configuration register */
	I2C_Write(0x00);	/* Fs = 8KHz */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(GYRO_CONFIG);	/* Write to Gyro configuration register */
	I2C_Write(0x18);	/* Full scale range +/- 2000 degree/C */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(INT_ENABLE);	/* Write to interrupt enable register */
	I2C_Write(0x01);
	I2C_Stop();

}

void MPU_Start_Loc()

{
	I2C_Start_Wait(0xD0);								/* I2C start with device write address */
	I2C_Write(ACCEL_XOUT_H);							/* Write start location address from where to read */ 
	I2C_Repeated_Start(0xD1);							/* I2C start with device read address */
}

void main()

{
	char buffer[20];
	int Ax,Ay,Az,T,Gx,Gy,Gz;
	float Xa,Ya,Za,t,Xg,Yg,Zg;
    OSCCON = 0x72;
    I2C_Init();											/* Initialize I2C */	
	MPU6050_Init();										/* Initialize Gyro */
	USART_Init(9600);									/* Initialize USART with 9600 baud rate */
	
	while(1)
	{
		MPU_Start_Loc();								/* Read Gyro values continuously and send to terminal over USART */
		Ax = (((int)I2C_Read(0)<<8) | (int)I2C_Read(0));
		Ay = (((int)I2C_Read(0)<<8) | (int)I2C_Read(0));
		Az = (((int)I2C_Read(0)<<8) | (int)I2C_Read(0));
		T = (((int)I2C_Read(0)<<8) | (int)I2C_Read(0));
		Gx = (((int)I2C_Read(0)<<8) | (int)I2C_Read(0));
		Gy = (((int)I2C_Read(0)<<8) | (int)I2C_Read(0));
		Gz = (((int)I2C_Read(0)<<8) | (int)I2C_Read(1));
		I2C_Stop();
        
		
		Xa = (float)Ax/16384.0;							/* Divide raw value by sensitivity scale factor to get real values */
		Ya = (float)Ay/16384.0;
		Za = (float)Az/16384.0;
		Xg = (float)Gx/131.0;
		Yg = (float)Gy/131.0;
		Zg = (float)Gz/131.0;
		t = ((float)T/340.00)+36.53;					/* Convert temperature in °/c using formula */


		sprintf(buffer," Ax = %.2f g\t",Xa);			/* Take values in buffer to send all parameters over USART */
		USART_SendString(buffer);

		sprintf(buffer," Ay = %.2f g\t",Ya);
		USART_SendString(buffer);
		
		sprintf(buffer," Az = %.2f g\t",Za);
		USART_SendString(buffer);

		sprintf(buffer," T = %.2f%cC\t",t,0xF8);           /* 0xF8 Ascii value of degree '°' on serial */
		USART_SendString(buffer);

		sprintf(buffer," Gx = %.2f%c/s\t",Xg,0xF8);
		USART_SendString(buffer);

		sprintf(buffer," Gy = %.2f%c/s\t",Yg,0xF8);
		USART_SendString(buffer);
		
		sprintf(buffer," Gz = %.2f%c/s\r\n",Zg,0xF8);
		USART_SendString(buffer);
	}
}

