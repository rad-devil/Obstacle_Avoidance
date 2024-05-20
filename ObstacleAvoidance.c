/*
 * Copyright 2016-2023 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    Project5.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL46Z4.h"
#include "fsl_debug_console.h"
/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */

/*
 * @brief   Application entry point.
 */


#define FWD 1
#define REV 2
#define LEFT 3
#define RIGHT 4
#define BOTH 5

#define PWM_LEFT 0
#define PWM_RIGHT 1

volatile char end_color = ' ';


volatile unsigned char light_val_left = 0;
volatile unsigned char light_val_right = 0;

volatile unsigned int time_now_ms = 0;
//volatile unsigned int start_time = 0;

//volatile unsigned int end_time = 0;
volatile unsigned int over_flow_num = 0;

volatile  float distance = 0.0;


char Detect_color();

void delay_ms(unsigned short delay_t) {
    SIM->SCGC6 |= (1 << 24); // Clock Enable TPM0
    SIM->SOPT2 |= (0x2 << 24); // Set TPMSRC to OSCERCLK
    TPM0->CONF |= (0x1 << 17); // Stop on Overflow
    TPM0->SC = (0x1 << 7) | (0x07); // Reset Timer Overflow Flag, Set Prescaler 128
    TPM0->MOD = delay_t * 61 + delay_t/2; //

    TPM0->SC |= 0x01 << 3; // Start the clock!

    while(!(TPM0->SC & 0x80)){} // Wait until Overflow Flag
    return;
}


void MotorSetup()
{


	SIM->SCGC5 |= 1<<10;
	PORTB->PCR[0] &= ~0x700;
	PORTB->PCR[0] |= 0x700 & (1 << 8);
	GPIOB->PDDR |= (1 << 0);

	PORTB->PCR[1] &= ~0x700;
	PORTB->PCR[1] |= 0x700 & (1 << 8);
	GPIOB->PDDR |= (1 << 1);



	SIM->SCGC6 |= (1 << 26); // Clock Enable TPM2
	SIM->SOPT2 |= (0x2 << 24); // Set TPMSRC to OSCERCLK

	PORTB->PCR[2] &= ~0x700;
	PORTB->PCR[2] |= 0x700 & (1 << 8); // set ptb 2 as tpm2_CH0
	GPIOB->PDDR |= (1 << 2);

	// init TPM2 for PWM
	// TPM2->CONTROLS[0].CnSC |= (0x2 << 2) | (0x2 << 4);  // set 1 in bit 2 and set 1 in bit 4



	//TPM2->MOD = 99; // number of steps available for pwm control





	PORTB->PCR[3] &= ~0x700;
	PORTB->PCR[3] |= 0x700 & (1 << 8); // set ptb 3 as tpm2_CH1
	GPIOB->PDDR |= (1 << 3);

	// TPM2->CONTROLS[1].CnSC |= (0x2 << 2) | (0x2 << 4);


	SIM->SCGC5 |= 1<<11;


	PORTC->PCR[1] &= ~0x700;
	PORTC->PCR[1] |= 0x700 & (1 << 8);
	GPIOC->PDDR |= (1 << 1);
	PORTC->PCR[2] &= ~0x700;
	PORTC->PCR[2] |= 0x700 & (1 << 8);
	GPIOC->PDDR |= (1 << 2);

	//TPM2->SC |= 0x01 << 3;



	// set up PTB2 ,3 as PWM Output


}

void straight(int direction, float duration)
{


	TPM2->CONTROLS[PWM_LEFT].CnV = 99;
	TPM2->CONTROLS[PWM_RIGHT].CnV = 99;




	if (direction == FWD)
	{
						GPIOB->PDOR |= (1<<0);
							GPIOB->PDOR &= ~(1<<1);
							GPIOC->PDOR |= (1<<1);
							GPIOC->PDOR &=  ~(1<<2);


							 delay_sec(2);

		// account for delay

	}

	else if (direction == REV)
	{
		GPIOB->PDOR &= ~(1<<0);
				GPIOB->PDOR |= (1<<1);


				GPIOC->PDOR &= ~(1<<1);
				GPIOC->PDOR |= (1<<2);

	}

	else
	{
		GPIOB->PDOR |= (1<<1);
		GPIOB->PDOR &= ~(1<<0);
		GPIOC->PDOR &= ~(1<<2);
		GPIOC->PDOR |= (1<<1);
		//delay_ms(2000);

	}



	/*duration = duration*1000;


	delay_ms((int)duration);
*/





}


void SwitchSetup()
{
	SIM->SCGC5 |= 1<<11;

	PORTC->PCR[3] &= ~0x703; // Clear mux and PE/PS bits
	PORTC->PCR[3] |= 0x703 & ((1 << 8) | 0x03); // Set MUX bits to GPIO, Set pullup and pull enable.
	GPIOC->PDDR &= ~(1 << 3); // Clear Data direction (input)
}

bool sw1Pressed()
{
	if(! (GPIOC->PDIR & 0x8))
	{
		return true;
	}

	else
	{
		return false;
	}
}

void ArcCW(float duration)
{

	TPM2->CONTROLS[PWM_LEFT].CnV = 99;
	TPM2->CONTROLS[PWM_RIGHT].CnV = 82;

								GPIOB->PDOR |= (1<<0);
								GPIOB->PDOR &= ~(1<<1);
								GPIOC->PDOR |= (1<<1);
								GPIOC->PDOR &=  ~(1<<2);

								duration = duration*1000;


								delay_ms((int)duration);
								//StopMotor(BOTH);



}

void ArcCCW(float duration)
{
	TPM2->CONTROLS[PWM_LEFT].CnV = 74;
		TPM2->CONTROLS[PWM_RIGHT].CnV = 99;

									GPIOB->PDOR |= (1<<0);
									GPIOB->PDOR &= ~(1<<1);
									GPIOC->PDOR |= (1<<1);
									GPIOC->PDOR &=  ~(1<<2);

									duration = duration*1000;


									//delay_ms((int)duration);
									//StopMotor(BOTH);



}


void Turn(int direction, int duration)
{
	if (direction == RIGHT)
	{


				StopMotor(RIGHT);
				delay_ms(750);
				StopMotor(LEFT);


	}

	else if (direction == LEFT)
	{


				StopMotor(LEFT);
				delay_ms(200);
				StopMotor(RIGHT);



	}

	//delay_sec(duration);


}

void StopMotor(int motor)
{



	if (motor == LEFT)
	{

		GPIOB->PDOR &= ~(1<<0);
		GPIOB->PDOR &= ~(1<<1);

	}

	if (motor == RIGHT)
	{

		GPIOC->PDOR &= ~(1<<1);
		GPIOC->PDOR &= ~(1<<2);

	}


	if (motor == BOTH)
	{
		GPIOB->PDOR &= ~(1<<0);
		GPIOB->PDOR &= ~(1<<1);

		GPIOC->PDOR &= ~(1<<1);
		GPIOC->PDOR &= ~(1<<2);
	}



}




void initI2C() {

	SIM->SCGC4 |= (1<<6);
	SIM->SCGC5 |= (1<<11);
	PORTC->PCR[9] &= ~0x700;
	PORTC->PCR[9] |= (1<<9);
	//GPIOC->PDDR &= ~(1 << 9);
	PORTC->PCR[8] &= ~0x700;
	PORTC->PCR[8] |= (1<<9);
	//GPIOC->PDDR &= ~(1 << 8);


	// set up pin mode for PTC9 and PTC8

	//set pins to alt4

	//
	I2C0->A1 = 0;
	I2C0->F = 0;
	I2C0->C1 = 0;
	I2C0->S = 0;
	I2C0->D = 0;
	I2C0->C2 = 0;
	I2C0->FLT = 0;
	I2C0->RA = 0;
	I2C0->SMB = 0;
	I2C0->A2 = 0;
	I2C0->SLTH = 0;
	I2C0->SLTL = 0;

	I2C0->FLT |= 0x50;

	I2C0->S |= (1<<4);
	I2C0->S |= (1<<1);

	I2C0->F |=  0x27;
	I2C0->F &= ~(0x3<<6);




    // Enable Clock Gating for I2C module and Port

    // Setup Pin Mode for I2C

    // Write 0 to all I2C registers

    // Write 0x50 to FLT register (clears all bits)

    // Clear status flags

    // Set I2C Divider Register (Choose a clock frequency less than 100 KHz)

    // Set bit to enable I2C Module

	//
	//&  ~(0x3<<3)

	I2C0->C1 |= (1<<7);
	I2C_WriteByte(128,1);
	delay_sec(1);
	I2C_WriteByte(0x01 | 0x80,0xFF);
	delay_sec(1);
}


void clearStatusFlags() {

	I2C0->FLT |= (1<<6);
	I2C0->FLT |= (1<<4);
	I2C0->S |= (1<<4);
	I2C0->S |= (1<<1);

   // Clear STOPF and Undocumented STARTF bit 4 in Filter Register

    // Clear ARBL and IICIF bits in Status Register

}

void TCFWait() {
	while(!((I2C0->S) & (1 << 7)))
	{

	}

    // Wait for TCF bit to Set in Status Register

}


void IICIFWait() {

	//printf("%d",( (I2C0->S)&(1 << 1)));
	while(!((I2C0->S) & (1 << 1)))
	{


	}

    // Wait for IICIF bit to Set in Status Register

}


void SendStart() {
	I2C0->C1 |= (1 << 5);

	I2C0->C1 |= (1 << 4);

    // Set MST and TX bits in Control 1 Register


}


void RepeatStart() {
	I2C0->C1 |= (1 << 5);

	I2C0->C1 |= (1 << 4);

	I2C0->C1 |= (1 << 2);


	// Set MST, TX and RSTA bits in Control 1 Register

    // Wait 6 cycles      // Dirty delay of 90 microseconds most probably
	for(int i = 0; i < 6; i ++)
	{
		__asm volatile ("nop");
	}

}


void SendStop() {

	I2C0->C1 &= ~(1 << 5);

	I2C0->C1 &= ~(1 << 4);

	I2C0->C1 &= ~(1 << 3);

	while((I2C0->S & (1 << 5)))
	{

	}



    // Clear MST, TX and TXAK bits in Control 1 Register

    // Wait for BUSY bit to go low in Status Register

}


void clearIICIF() {

	I2C0->S |= (1<<1);
    // Clear IICIF bit in Status Register

}


int RxAK() {

	if (!(I2C0->S & (1 << 0 )))
	{
		return 1;
	}

	else
	{
		return 0;
	}

    // Return 1 if byte has been ACK'd. (See RXAK in Status Register)

}



void I2C_WriteByte (int register_address,int register_data) {

    clearStatusFlags();

    TCFWait();

    SendStart();

    I2C0->D = (0x29<<1);

    // TODO: Write Device Address, R/W = 0 to Data Register


    IICIFWait();


    if (!RxAK()){

        printf("NO RESPONSE - Address");

        SendStop();

        return;

    }


    clearIICIF();


    I2C0->D = (register_address);
    // TODO: Write Register address to Data Register

    IICIFWait();

    if (!RxAK()){

        printf("NO RESPONSE - Register");

        SendStop();

        return;

    }


    TCFWait();

    clearIICIF();



    // Write Data byte to Data Register
    I2C0->D = (register_data);

    IICIFWait();

    if (!RxAK()){

        printf("Incorrect ACK - Data");

    }


    clearIICIF();

    SendStop();

}

void delay_sec_m(unsigned int n)
{
	for (int i = 0;i < n*1000; i++)
	{
		delay_ms(1);
	}
}


void delay_sec_n(unsigned int n)
{
	for (int i = 0;i < n*1000; i++)
	{
		delay_ms(1);
	}
}

void Read_Block(int register_address, int *destination_Data_Byte_Array, int Length) {

    unsigned char dummy = 0;

    clearStatusFlags();

    TCFWait();

    SendStart();


    dummy++;  // Do something to suppress the warning.

    I2C0->D = (0x29<<1);

    //TODO: Write Device Address, R/W = 0 to Data Register


    IICIFWait();


    if (!RxAK()){

        printf("NO RESPONSE - Address");

        SendStop();

        return 0;

    }


    clearIICIF();


    I2C0->D = (register_address);

    // Write Register address to Data Register

    IICIFWait();

    if (!RxAK()){

        printf("NO RESPONSE - Register");

        SendStop();

        return 0;

    }


    clearIICIF();

    RepeatStart();


    I2C0->D = ((0x29<<1) | (1<<0));
    // Write device address again, R/W = 1 to Data Register

    IICIFWait();

    if (!RxAK()){

        printf("NO RESPONSE - Restart");

        SendStop();

        return 0;

    }


    TCFWait();

    clearIICIF();

    I2C0->C1 &= ~(1 << 4);

    I2C0->C1 &= ~(1 << 3);



    // Switch to RX by clearing TX and TXAK bits in Control 1 register


    if(Length==1){
    	I2C0->C1 |= (1 << 3);




        // Set TXAK to NACK in Control 1 - No more data!

    }


   dummy = I2C0->D; // Dummy Read
  // printf("%d",dummy);

    for(int index=0; index<Length; index++){

        IICIFWait();

        clearIICIF();


        if(index == Length-2){


        	I2C0->C1 |= (1 << 3);
            // Set TXAK to NACK in Control 1 - No more data!

        }


       if(index == Length-1){

            SendStop();

        }
       /*printf("Values \n: %d ", red_val[i]);*/
       destination_Data_Byte_Array[index] = I2C0->D;

        // Read Byte from Data Register into Array

    }

}


void ServoSetup()
{
		SIM->SOPT2 |= (0x2 << 24);
		//TPM1->SC |= 0x06;
		SIM->SCGC5 |= 1<<9;
		SIM->SCGC6 |= (1 << 25);
		PORTA->PCR[12] &= ~0x700;
		PORTA->PCR[12] |= 0x350;

		TPM1->CONTROLS[0].CnSC |= (0x2 << 2) | (0x2 << 4);


		TPM1->MOD = 1250;
		TPM1->SC |=0x06;
		TPM1->SC |= 0x01 << 3;
		TPM1->CONTROLS[0].CnV = 187;
}


void SonarSetup()
{
	SIM->SCGC5 |= 1<<12;
	PORTD->PCR[2] &= ~0x700;
	PORTD->PCR[2] |= (1 << 8);
	GPIOD->PDDR |= (1 << 2);
	GPIOD->PDOR &= ~(1 << 2);

	// Setup PTA13 as interrupt source both rising and falling edge
	SIM->SCGC5 |= 1<<9;
	PORTA->PCR[13] &= ~0x700;
	PORTA->PCR[13] |= (1 << 8);

	GPIOA->PDDR &= ~(1 << 13);
	//NVIC_EnableIRQ(30);


	SIM->SCGC6 |= 1<<26;
	SIM->SOPT2 |= 2<<24;
	TPM2->MOD = 39999;
	TPM2->CONF |= 0x3<<6;
	TPM2->SC |= (1 << 6) | (1 << 7);
	TPM2->SC |= (0x01 << 3) ;
	//NVIC_EnableIRQ(19);


}

void TPM2_IRQHandler(void){
	if(TPM2->SC & (1 << 7))
	{
		TPM2->SC |= (1 << 7);
		over_flow_num++;
	}
 // Reset Timer Interrupt Flag
		// What happens on overflow?
	}



bool check()
{
	bool x = false;
	 unsigned int start_time = 0;
	    unsigned int end_time = 0;
	    int save_overflow = 0;

	    unsigned int diff = 0;

	    unsigned int dist = 0;
	   // MotorSetup();


	    //init_timer();




	    	//TPM1->CONTROLS[0].CnV = 125;

	    //straight(FWD,0);


	    //int save_overflow = 0;
	    TPM1->CONTROLS[0].CnV = 250;


	    				GPIOD->PDOR |= (1<<2);

	    		    	    delay_ten_macro();

	    		    	    GPIOD->PDOR &= ~(1<<2);



	    		    	    over_flow_num = 0;


	    		    	    while(!(GPIOA->PDIR & 1 << 13))
	    		    	    {
	    		    	    	__asm volatile ("nop");
	    		    	    }

	    		    	    start_time = TPM2->CNT;

	    		    	    while((GPIOA->PDIR & 1 << 13))
	    		    	    {
	    		    	    	__asm volatile ("nop");
	    		    	    }

	    		    	    end_time = TPM2->CNT;

	    		    	    save_overflow = over_flow_num;

	    		    	    if(end_time < start_time || save_overflow > 1)
	    		    	    {
	    		    	    	diff = ((40000*save_overflow) + end_time) - start_time;
	    		    	    }
	    		    	    else
	    		    	    {
	    		    	    	diff = end_time - start_time;
	    		    	    }

	    		    	    dist = diff/464;

	    		    	   // echo_ready = false;
	    		    	   // distance =  (float)(1.0/290.0)*((float) diff);
	    		    	        	    	//distance = distance/27;
	    		    	    //printf("%d \n",dist);


	    		    	    delay_ms(100);

	    		    	    if(dist < 16)
	    		    	    {
	    		    	    	x = false;
	    		    	    }

	    		    	    else
	    		    	    {
	    		    	    	x = true;
	    		    	    }



	return x;

}



void delay_sec(unsigned int n)
{
	for (int i = 0;i < n*1000; i++)
	{

	}
}


void park()
{
    straight(10,0);
    delay_ms(650);
    StopMotor(BOTH);
    straight(REV,0);
    delay_sec_m(1);
    StopMotor(BOTH);
}


void obstacle_avoidance()
{
	TPM1->CONTROLS[0].CnV = 125;
	while(!check())
	{
		if(light_val_left < 240)
		{
			Turn(LEFT,1);
		}
		ArcCCW(1000);

		if(light_val_right > 240)
		{
			Turn(RIGHT,1);
		}

		ArcCCW(1000);
	}





}




void follow()
{

	straight(FWD,0);
	    	       	   delay_sec_n(1);
	    	       	   Turn(RIGHT,0);
	bool inner_side = 0;
	ArcCCW(1000);

	 while(1) {



	    ADC0->SC1[0] = 0x01; // Set Channel, starts conversion.  0x1 left 0x5 for right

	    //ADC0->SC1[0] |= (1<<5);
	    while(!(ADC0->SC1[0] & 0x80)){ }
	    //delay(1000);
	    light_val_left = ADC0->R[0]; // Resets COCO


	   // printf("%d ",light_val_left);


	    ADC0->SC1[0] = 0x05; // Set Channel, starts conversion.  0x1 left 0x5 for right

	    //ADC0->SC1[0] |= (1<<5);
	    while(!(ADC0->SC1[0] & 0x80)){ }
	    //delay(1000);
	    light_val_right = ADC0->R[0]; // Resets COCO


	   //printf("%d \n",light_val_right); //sweet spot is 240

	 /*  if(!check)
	    {
	    	obstacle_avoidance();
	    	TPM1->CONTROLS[0].CnV = 187;
	    }*/

	    if(light_val_right > 220)
	    {
	    	Turn(LEFT,1);
	    	//inner_side = 0;
	    	ArcCCW(1000);
	    }

	    //printf("%d \n", light_val_right);

	    if(light_val_left > 220)
	    {
	    	Turn(RIGHT,1);
	    	//inner_side = 1;
	    	ArcCCW(1000);
	    }



	    //straight(FWD,0);
	    //ArcCCW(1000);




	        	//printf(" %d %d %d \n ", red, green,blue);








	        	if (Detect_color() == 'r' && end_color == 'r')
	        	{
	        		delay_ms(800);
	        		park();
	        		break;


	        	}

	        	if (Detect_color() == 'b' && end_color == 'b')
	        	{
	        		delay_ms(800);
	        		park();
	        		break;



	        	}

	        	if(Detect_color() == 'g' && end_color == 'g')
	        	{
	        		delay_ms(800);
	        		park();
	        		break;


	        	}

	        	if(Detect_color() == 'y' && end_color == 'y')
	        	{
	        		delay_ms(800);
	        		park();

	        		break;

	        	}



	    }
}



char Detect_color()
{
	//SonarSetup();
	  	  int red_val[2] = {0,0};
	       int green_val[2] = {0,0};
	       int blue_val[2] = {0,0};

	       int red = 0;
	       int blue = 0;
	       int green = 0;
	Read_Block(182,red_val,2);
	 //delay_sec(1);
	Read_Block(184,green_val,2);
	 //delay_sec(1);
	Read_Block(186,blue_val,2);
	 //delay_sec(1);
	//Read_Block(182,red_val,2);
	red = red_val[0] | (red_val[1]<<8);
	green = green_val[0] | (green_val[1]<<8);
	blue = blue_val[0] | (blue_val[1] << 8);




	//printf(" %d %d %d \n ", red, green,blue);







	    // if one number is less than 220


				if ((red > 290) && (green < 290) && (blue < 290))
				{
					return  'r';
					//printf("red \n");
					/*StopMotor(BOTH);
					   break;*/


				}

				if ((blue > 220) && (red < 320) && (green < 320) )
				   {
						//printf("blue \n");
					return 'b';

				    }

				if((green > 310) && (blue < 500) && (red < 500) )
				{
					//printf("green \n");
					return 'g';

					//break;
				}



				if((green > 100+blue)  && (red > 100 + blue) )
				{
					//delay_sec_m(1);
					//StopMotor(BOTH);
					//printf("yellow \n");
					return 'y';


				}




			return 'n';

}





int main(void) {

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif

    //PRINTF("Hello World\n");

     //Force the counter to be placed into memory.

    MotorSetup();
    SwitchSetup();




   unsigned short cal_v = 0;


    while(!sw1Pressed())
       			{


       			}


    initI2C();


        I2C_WriteByte(128,3);
        delay_sec(1);
        I2C_WriteByte(129,246);
        delay_sec(1);

   //straight(FWD,0);
     //Enter an infinite loop, just incrementing a counter.
    // Clock Gating
    SIM->SCGC5 |= (1<<13); // Enable Light Sensor I/O Port
    SIM->SCGC5 |= (1<<12);
    SIM->SCGC6 |= (1<<27); // Enable ADC Module


    // Setup Analog Input - Default is analog (PTE22), No need to do anything.

    //ADC0->SC1 |= (1<<0);
    // Setup LED Pin for GPIO
    PORTD->PCR[5] &= ~0x700; // Clear First
    PORTD->PCR[5] |= 0x700 & (1 << 8); // Set MUX bits
    // Setup Pin 5 as output
    GPIOD->PDDR |= (1 << 5);
    // Setup ADC Clock ( < 4 MHz)
    ADC0->CFG1 = 0; // Default everything.
    // Analog Calibrate


    ADC0->SC3 = 0x07; // Enable Maximum Hardware Averaging
    ADC0->SC3 |= 0x80; // Start Calibration
    // Wait for Calibration to Complete (either COCO or CALF)
    while(!(ADC0->SC1[0] & 0x80)){ }
    // Calibration Complete, write calibration registers.
    cal_v = ADC0->CLP0 + ADC0->CLP1 + ADC0->CLP2 + ADC0->CLP3 + ADC0->CLP4 + ADC0->CLPS;
    cal_v = cal_v >> 1 | 0x8000;
    ADC0->PG = cal_v;
    cal_v = 0;
    cal_v = ADC0->CLM0 + ADC0->CLM1 + ADC0->CLM2 + ADC0->CLM3 + ADC0->CLM4 +
    ADC0->CLMS;
    cal_v = cal_v >> 1 | 0x8000;
    ADC0->MG = cal_v;
    ADC0->SC3 = 0; // Turn off Hardware Averaging

    //SonarSetup();
    //ServoSetup();


    //ArcCW(1000);



    while(1)
    {

    	  if (Detect_color() == 'r')
    	    	   		   {
    	    	   			   end_color = 'y';
    	    	   			   follow();
    	    	   			   break;
    	    	   		   }

    	    	   		   if (Detect_color() == 'b')
    	    	   		   {
    	    	   			   end_color = 'g';
    	    	   			   follow();
    	    	   			  break;

    	    	   		   }

    	    	   		   if (Detect_color() == 'g')
    	    	   		   {
    	    	   			   end_color = 'b';
    	    	   			   follow();
    	    	   			   break;
    	    	   		   }

    	    	   		   if(Detect_color() == 'y')
    	    	   		   {
    	    	   			   end_color = 'r';
    	    	   			   follow();
    	    	   			   break;
    	    	   		   }




    }






    return 0 ;
}
