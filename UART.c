/*
 * UART.c
 *
 *  Created on: Oct 22, 2018
 *      Author: LuisFernando & Tsipini
 */

#include "UART.h"

UART_MailBoxType UART0_MailBox;
UART_MailBoxType UART1_MailBox;
UART_MailBoxType UART2_MailBox;
UART_MailBoxType UART3_MailBox;
UART_MailBoxType UART4_MailBox;
UART_MailBoxType UART5_MailBox;

/*UART PCR*/
static const GPIO_pinControlRegisterType UART_PCR = GPIO_MUX3;

void UART0_RX_TX_IRQHandler(void)
{
	/*First is verified if the serial port finished to transmit*/
	while(!(UART0->S1 && UART_S1_RDRF_SHIFT));
	/*The info is saved in Data Register*/
	UART0_MailBox.mailBox = UART0->D;
	/*There are new data*/
	UART0_MailBox.flag = TRUE;
}

void UART_init(UART_ChannelType uartChannel, uint32 systemClk, UART_BaudRateType baudRate)
{
	uint16 brfa;
	uint8 temp;

	/**This is mail box to received the information from the serial port*/
	UART_MailBoxType UART0_MailBox;

	/*Set the configuration for UART_PCR_Tx in pin16/PortB*/
	GPIO_pin_control_register(GPIO_B, BIT16, &UART_PCR);
	GPIO_data_direction_pin(GPIO_B, GPIO_OUTPUT, BIT16);
	/*Set the configuration for UART_PCR_Rx in pin17/PortB*/
	GPIO_pin_control_register(GPIO_B, BIT17, &UART_PCR);
	GPIO_data_direction_pin(GPIO_B, GPIO_INPUT, BIT17);

	switch(uartChannel)
	{
	case UART_0:
		SIM->SCGC4 = SIM_SCGC4_UART0_MASK;
		/*Disable Tx/Rx*/
		UART0->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK );
		/*Calculate baud settings*/
		brfa = (uint16) ((systemClk)/(16*baudRate));
		/*Save the value of UART0->BDH, for exception of the SBR*/
		temp = UART0->BDH & ~(UART_BDH_SBR(BDH_SBR_MASK));
		UART0->BDH = temp | (((brfa & UART_BDH_SBR_MASK) >> SHIFT8));
		UART0->BDL = (uint8_t) (brfa & UART_BDL_SBR_MASK);
		/*Copy brfa to Reg UART0->C4*/
		UART0->C4 |= UART_C4_BRFA_MASK;
		UART0->C4 = (brfa);

		UART0->C2 |= (UART_C2_TE_MASK | UART_C2_RE_MASK);
		break;
	case UART_1:
		SIM->SCGC4 = SIM_SCGC4_UART1_MASK;
		/*Disable Tx/Rx*/
		UART1->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);
		/*Calculate baud settings*/
		brfa = (uint16) ((systemClk) / (16 * baudRate));
		/*Save the value of UART0->BDH, for exception of the SBR*/
		temp = UART1->BDH & ~(UART_BDH_SBR(BDH_SBR_MASK));
		UART1->BDH = temp | (((brfa & UART_BDH_SBR_MASK) >> SHIFT8));
		UART1->BDL = (uint8_t) (brfa & UART_BDL_SBR_MASK);
		/*Copy brfa to Reg UART0->C4*/
		UART1->C4 |= UART_C4_BRFA_MASK;
		UART1->C4 = (brfa);

		UART1->C2 |= (UART_C2_TE_MASK | UART_C2_RE_MASK);
		break;
	case UART_2:
		SIM->SCGC4 = SIM_SCGC4_UART2_MASK;
		/*Disable Tx/Rx*/
		UART2->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);
		/*Calculate baud settings*/
		brfa = (uint16) ((systemClk) / (16 * baudRate));
		/*Save the value of UART0->BDH, for exception of the SBR*/
		temp = UART2->BDH & ~(UART_BDH_SBR(BDH_SBR_MASK));
		UART2->BDH = temp | (((brfa & UART_BDH_SBR_MASK) >> SHIFT8));
		UART2->BDL = (uint8_t) (brfa & UART_BDL_SBR_MASK);
		/*Copy brfa to Reg UART0->C4*/
		UART2->C4 |= UART_C4_BRFA_MASK;
		UART2->C4 = (brfa);

		UART2->C2 |= (UART_C2_TE_MASK | UART_C2_RE_MASK);
		break;
	case UART_3:
		SIM->SCGC4 = SIM_SCGC4_UART3_MASK;
		/*Disable Tx/Rx*/
		UART3->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);
		/*Calculate baud settings*/
		brfa = (uint16) ((systemClk) / (16 * baudRate));
		/*Save the value of UART0->BDH, for exception of the SBR*/
		temp = UART3->BDH & ~(UART_BDH_SBR(BDH_SBR_MASK));
		UART3->BDH = temp | (((brfa & UART_BDH_SBR_MASK) >> SHIFT8));
		UART3->BDL = (uint8_t) (brfa & UART_BDL_SBR_MASK);
		/*Copy brfa to Reg UART0->C4*/
		UART3->C4 |= UART_C4_BRFA_MASK;
		UART3->C4 = (brfa);

		UART3->C2 |= (UART_C2_TE_MASK | UART_C2_RE_MASK);
		break;
	case UART_4:
		SIM->SCGC1 = SIM_SCGC1_UART4_MASK;
		/*Disable Tx/Rx*/
		UART4->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);
		/*Calculate baud settings*/
		brfa = (uint16) ((systemClk) / (16 * baudRate));
		/*Save the value of UART0->BDH, for exception of the SBR*/
		temp = UART4->BDH & ~(UART_BDH_SBR(BDH_SBR_MASK));
		UART4->BDH = temp | (((brfa & UART_BDH_SBR_MASK) >> SHIFT8));
		UART4->BDL = (uint8_t) (brfa & UART_BDL_SBR_MASK);
		/*Copy brfa to Reg UART0->C4*/
		UART4->C4 |= UART_C4_BRFA_MASK;
		UART4->C4 = (brfa);

		UART4->C2 |= (UART_C2_TE_MASK | UART_C2_RE_MASK);
		break;
	case UART_5:
		SIM->SCGC1 = SIM_SCGC1_UART5_MASK;
		/*Disable Tx/Rx*/
		UART5->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);
		/*Calculate baud settings*/
		brfa = (uint16) ((systemClk) / (16 * baudRate));
		/*Save the value of UART0->BDH, for exception of the SBR*/
		temp = UART5->BDH & ~(UART_BDH_SBR(BDH_SBR_MASK));
		UART5->BDH = temp | (((brfa & UART_BDH_SBR_MASK) >> SHIFT8));
		UART5->BDL = (uint8_t) (brfa & UART_BDL_SBR_MASK);
		/*Copy brfa to Reg UART0->C4*/
		UART5->C4 |= UART_C4_BRFA_MASK;
		UART5->C4 = (brfa);

		UART5->C2 |= (UART_C2_TE_MASK | UART_C2_RE_MASK);
		break;
	}
}

void UART0_interruptEnable(UART_ChannelType uartChannel)
{
	switch(uartChannel)
	{
	case UART_0:
		UART0->C2 |= UART_C2_RIE_MASK;
		break;
	case UART_1:
		UART1->C2 |= UART_C2_RIE_MASK;
		break;
	case UART_2:
		UART2->C2 |= UART_C2_RIE_MASK;
		break;
	case UART_3:
		UART3->C2 |= UART_C2_RIE_MASK;
		break;
	case UART_4:
		UART4->C2 |= UART_C2_RIE_MASK;
		break;
	case UART_5:
		UART5->C2 |= UART_C2_RIE_MASK;
		break;
	}
}

void UART_putChar (UART_ChannelType uartChannel, sint8 character)
{
	switch (uartChannel)
	{
	case UART_0:
		/*Check if there isn't any data transmition*/
		while(!(UART0->S1 && UART_S1_TC_MASK));
		/*Send char to the Data Reg*/
		UART0->D = (uint8_t) character;
		break;
	case UART_1:
		/*Check if there isn't any data transmition*/
		while(!(UART1->S1 && UART_S1_TC_MASK));
		/*Send char to the Data Reg*/
		UART1->D = (uint8_t) character;
		break;
	case UART_2:
		/*Check if there isn't any data transmition*/
		while(!(UART2->S1 && UART_S1_TC_MASK));
		/*Send char to the Data Reg*/
		UART2->D = (uint8_t) character;
		break;
	case UART_3:
		/*Check if there isn't any data transmition*/
		while(!(UART3->S1 && UART_S1_TC_MASK));
		/*Send char to the Data Reg*/
		UART3->D = (uint8_t) character;
		break;
	case UART_4:
		/*Check if there isn't any data transmition*/
		while(!(UART4->S1 && UART_S1_TC_MASK));
		/*Send char to the Data Reg*/
		UART4->D = (uint8_t) character;
		break;
	case UART_5:
		/*Check if there isn't any data transmition*/
		while (!(UART5->S1 && UART_S1_TC_MASK));
		/*Send char to the Data Reg*/
		UART5->D = (uint8_t) character;
		break;

	}
}

void UART_putString(UART_ChannelType uartChannel, sint8* string)
{
	switch(uartChannel)
	{
		case UART_0:
			/*Transmit data*/
			/*Preguntar si se puede funcion null*/
			while(*string)
			{
				/*Keeps sending char until ptr is null*/
				UART_putChar(UART_0,*string++);
			}
			break;
		case UART_1:
			/*Transmit data*/
			while(*string)
			{
				/*Keeps sending char until ptr is null*/
				UART_putChar(UART_1,*string++);
			}
			break;
		case UART_2:
			/*Transmit data*/
			while(*string)
			{
				/*Keeps sending char until ptr is null*/
				UART_putChar(UART_2,*string++);
			}
			break;
		case UART_3:
			/*Transmit data*/
			while(*string)
			{
				/*Keeps sending char until ptr is null*/
				UART_putChar(UART_3,*string++);
			}
			break;
		case UART_4:
			/*Transmit data*/
			while(*string)
			{
				/*Keeps sending char until ptr is null*/
				UART_putChar(UART_4,*string++);
			}
			break;
		case UART_5:
			/*Transmit data*/
			while(*string)
			{
				/*Keeps sending char until ptr is null*/
				UART_putChar(UART_5,*string++);
			}
			break;
		default:
			break;

	}
}

uint8 UART_getChar(UART_ChannelType uartChannel)
{
	switch (uartChannel)
	{
	case UART_0:
		/* Wait until space is available in the FIFO */
		while (!(UART0->S1 & UART_S1_RDRF_MASK));
		/* Send the character */
		return UART0->D;
		break;
	case UART_1:
		/* Wait until space is available in the FIFO */
		while (!(UART1->S1 & UART_S1_RDRF_MASK));
		/* Send the character */
		return UART1->D;
		break;
	case UART_2:
		/* Wait until space is available in the FIFO */
		while (!(UART2->S1 & UART_S1_RDRF_MASK));
		/* Send the character */
		return UART2->D;
		break;
	case UART_3:
		/* Wait until space is available in the FIFO */
		while (!(UART3->S1 & UART_S1_RDRF_MASK));
		/* Send the character */
		return UART3->D;
		break;
	case UART_4:
		/* Wait until space is available in the FIFO */
		while (!(UART4->S1 & UART_S1_RDRF_MASK));
		/* Send the character */
		return UART4->D;
		break;
	case UART_5:
		/* Wait until space is available in the FIFO */
		while (!(UART5->S1 & UART_S1_RDRF_MASK));
		/* Send the character */
		return UART5->D;
		break;
	}
}
