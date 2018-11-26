/*
 * Copyright 2016-2018 NXP Semiconductor, Inc.
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
 * @file    Examen2_Project.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "UART.h"
#include "NVIC.h"
#include "Delay.h"
#include "stdio.h"

#define SYSTEM_CLOCK 21000000
/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */

/*
 * @brief   Application entry point.
 */
/**This is mail box to received the information from the serial port*/
extern UART_MailBoxType UART0_MailBox;

int main(void)
{
	uint8 flag = TRUE;
	uint8 value = FALSE;
	uint8 position_x = FALSE;
	uint8 position_y = 30;
	sint8 arr[8] = {'0','3','3','[','0',position_y,position_x,'H'};
	sint8 i = FALSE;

	/**Enables the clock of PortB in order to configures TX and RX of UART peripheral*/
	SIM->SCGC5 = SIM_SCGC5_PORTB_MASK;
	/**Configures the pin control register of pin16 in PortB as UART RX*/
	PORTB->PCR[16] = PORT_PCR_MUX(3);
	/**Configures the pin control register of pin16 in PortB as UART TX*/
	PORTB->PCR[17]= PORT_PCR_MUX(3);
	/**Configures UART 0 to transmit/receive at 11520 bauds with a 21 MHz of clock core*/
	UART_init (UART_0,  21000000, BD_115200);
	printf("UART is configured");
	//printf("UART is configured");
	/**Enables the UART 0 interrupt*/
	UART0_interruptEnable(UART_0);
	/**Enables the UART 0 interrupt in the NVIC*/
	NVIC_enableInterruptAndPriotity(UART0_IRQ, PRIORITY_10);

	/**The following sentences send strings to PC using the UART_putString function. Also, the string
	 * is coded with terminal code*/
	/** VT100 command for text in white and background in black*/
	UART_putString(UART_0,"\033[0;30;47m");
	/*VT100 command for clearing the screen*/
	UART_putString(UART_0,"\033[2J");
	/** VT100 command for text in white and background in black*/
	UART_putString(UART_0,"\033[0;30;47m");
	/** VT100 command for positioning the cursor in x and y position*/
	UART_putString(UART_0,"\033[30;00H");

	/**Enables interrupts*/
	EnableInterrupts;

	for (;;) {
		if (UART0_MailBox.flag)
		{
			value = UART0_MailBox.mailBox;
			do
			{
				if (UART0_MailBox.flag)
				{
					if((UART0_MailBox.mailBox == 119) || (UART0_MailBox.mailBox == 87))
					{
						position_y--;
						arr[5] = position_y;
						UART_putChar(UART_0,92);
						while(*arr)
						{
							UART_putChar(UART_0,arr[i]);
							i++;
						}
						/**Sends to the PCA the received data in the mailbox*/
						UART_putChar(UART_0, value);
					}
					if((UART0_MailBox.mailBox == 61) || (UART0_MailBox.mailBox == 41))
					{
						position_x--;
						arr[6] = position_y;
						UART_putChar(UART_0,92);
						while(*arr)
						{
							UART_putChar(UART_0,arr[i]);
							i++;
						}
						/**Sends to the PCA the received data in the mailbox*/
						UART_putChar(UART_0, value);
					}
					if((UART0_MailBox.mailBox == 83) || (UART0_MailBox.mailBox == 115))
					{
						position_y++;
						arr[5] = position_y;
						UART_putChar(UART_0,92);
						while(*arr)
						{
							UART_putChar(UART_0,arr[i]);
							i++;
						}
						/**Sends to the PCA the received data in the mailbox*/
						UART_putChar(UART_0, value);
					}
					if((UART0_MailBox.mailBox == 68) || (UART0_MailBox.mailBox == 100))
					{
						position_x++;
						arr[6] = position_y;
						UART_putChar(UART_0,92);
						while(*arr)
						{
							UART_putChar(UART_0,arr[i]);
							i++;
						}
						/**Sends to the PCA the received data in the mailbox*/
						UART_putChar(UART_0, value);
					}
					if(UART0_MailBox.mailBox == 13)
					{
						flag = FALSE;
					}
					/**clear the reception flag*/
					UART0_MailBox.flag = 0;
				}
			}while(flag);
			/**clear the reception flag*/
			UART0_MailBox.flag = 0;
		}
	}

	return 0;
}
