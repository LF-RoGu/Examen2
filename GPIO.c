/*
 * GPIO.c
 *
 *  Created on: Sep 11, 2018
 *      Author: LuisFernando
 */

#include "MK64F12.h"
#include "GPIO.h"

uint8_t GPIO_clock_gating(gpio_port_name_t portName)
{
	switch(portName)/** Selecting the GPIO for clock enabling*/
			{
				case GPIO_A: /** GPIO A is selected*/
					SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTA; /** Bit 9 of SIM_SCGC5 is  set*/
					break;
				case GPIO_B: /** GPIO B is selected*/
					SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTB; /** Bit 10 of SIM_SCGC5 is set*/
					break;
				case GPIO_C: /** GPIO C is selected*/
					SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTC; /** Bit 11 of SIM_SCGC5 is set*/
					break;
				case GPIO_D: /** GPIO D is selected*/
					SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTD; /** Bit 12 of SIM_SCGC5 is set*/
					break;
				case GPIO_E: /** GPIO E is selected*/
					SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTE; /** Bit 13 of SIM_SCGC5 is set*/
					break;
				default: /**If doesn't exist the option*/
					return(FALSE);
			}// end switch
	/**Successful configuration*/
	return(TRUE);
}// end function

uint8_t GPIO_pin_control_register(gpio_port_name_t portName, uint8_t pin, GPIO_pinControlRegisterType* pinControlRegister)
{

	switch(portName)
		{
		case GPIO_A:/** GPIO A is selected*/
			PORTA->PCR[pin] = *pinControlRegister;
			break;
		case GPIO_B:/** GPIO B is selected*/
			PORTB->PCR[pin] = *pinControlRegister;
			break;
		case GPIO_C:/** GPIO C is selected*/
			PORTC->PCR[pin] = *pinControlRegister;
			break;
		case GPIO_D:/** GPIO D is selected*/
			PORTD->PCR[pin] = *pinControlRegister;
			break;
		case GPIO_E: /** GPIO E is selected*/
			PORTE->PCR[pin]= *pinControlRegister;
		default:/**If doesn't exist the option*/
			return(FALSE);
		break;
		}
	/**Successful configuration*/
	return(TRUE);
}

void GPIO_write_port(gpio_port_name_t portName, uint32 Data )
{
	switch(portName)
		{
			case GPIO_A:/** GPIOA is selected*/
				GPIOA->PDOR = Data;
				break;
			case GPIO_B:/** GPIOB is selected*/
				GPIOB->PDOR = Data;
				break;
			case GPIO_C:/** GPIOC is selected*/
				GPIOC->PDOR = Data;
				break;
			case GPIO_D:/** GPIOD is selected*/
				GPIOD->PDOR = Data;
				break;
			case GPIO_E: /** GPIOE is selected*/
				GPIOE->PDOR = Data;
			default:/**If doesn't exist the option*/
				break;
		}
}
uint32 GPIO_read_port(gpio_port_name_t portName)
{
	switch(portName)
	{
		case GPIO_A:/** GPIOA is selected*/
			return(GPIOA->PDIR);
			break;
		case GPIO_B:/** GPIOB is selected*/
			return(GPIOB->PDIR);
			break;
		case GPIO_C:/** GPIOC is selected*/
			return(GPIOC->PDIR);
			break;
		case GPIO_D:/** GPIOD is selected*/
			return(GPIOD->PDIR);
			break;
		case GPIO_E: /** GPIOE is selected*/
			return(GPIOE->PDIR);
		default:/**If doesn't exist the option*/
			return(FALSE);
			break;
	}
}
uint8_t GPIO_read_pin(gpio_port_name_t portName, uint8 pin)
{

	switch(portName)
	{
		case GPIO_A:/** GPIO A is selected*/
			return((GPIOA->PDIR >> pin) & 0x01);
			break;
		case GPIO_B:/** GPIO B is selected*/
			return((GPIOB->PDIR >> pin) & 0x01);
			break;
		case GPIO_C:/** GPIO C is selected*/
			return((GPIOC->PDIR >> pin) & 0x01);
			break;
		case GPIO_D:/** GPIO D is selected*/
			return((GPIOD->PDIR >> pin) & 0x01);
			break;
		case GPIO_E: /** GPIO E is selected*/
			return((GPIOE->PDIR >> pin) & 0x01);

		default:/**If doesn't exist the option*/
			return(FALSE);
			break;
	}
}
void GPIO_set_pin(gpio_port_name_t portName, uint8 pin)
{
	switch(portName)
	{
		case GPIO_A:/** GPIO A is selected*/
			GPIOA->PSOR |= TRUE << pin;
			break;
		case GPIO_B:/** GPIO B is selected*/
			GPIOB->PSOR |= TRUE << pin;
			break;
		case GPIO_C:/** GPIO C is selected*/
			GPIOC->PSOR |= TRUE << pin;
			break;
		case GPIO_D:/** GPIO D is selected*/
			GPIOD->PSOR |= TRUE << pin;
			break;
		case GPIO_E:/** GPIO E is selected*/
			GPIOE->PSOR |= TRUE << pin;
			break;
		default:
			break;
	}
}
void GPIO_clear_pin(gpio_port_name_t portName, uint8 pin)
{
	switch(portName)
	{
		case GPIO_A:/** GPIO A is selected*/
			GPIOA->PCOR |= TRUE << pin;
			break;
		case GPIO_B:/** GPIO B is selected*/
			GPIOB->PCOR |= TRUE << pin;
			break;
		case GPIO_C:/** GPIO C is selected*/
			GPIOC->PCOR |= TRUE  << pin;
			break;
		case GPIO_D:/** GPIO D is selected*/
			GPIOD->PCOR |= TRUE  << pin;
			break;
		case GPIO_E:/** GPIO E is selected*/
			GPIOE->PCOR |= TRUE  << pin;
			break;
		default:
			break;
	}
}
void GPIO_toogle_pin(gpio_port_name_t portName, uint8 pin)
{
	switch(portName)
	{
		case GPIO_A:/** GPIO A is selected*/
			GPIOA->PTOR |= TRUE << pin;
			break;
		case GPIO_B:/** GPIO B is selected*/
			GPIOB->PTOR |= TRUE << pin;
			break;
		case GPIO_C:/** GPIO C is selected*/
			GPIOC->PTOR |= TRUE << pin;
			break;
		case GPIO_D:/** GPIO D is selected*/
			GPIOD->PTOR |= TRUE << pin;
			break;
		case GPIO_E:/** GPIO E is selected*/
			GPIOE->PTOR |= TRUE << pin;
			break;
		default:
			break;
	}
}
void GPIO_dataDirectionPORT(gpio_port_name_t portName ,uint32 direction)
{
	switch(portName)
	{
		case GPIO_A:/** GPIO A is selected*/
			GPIOA->PDDR = direction;
			break;
		case GPIO_B:/** GPIO B is selected*/
			GPIOB->PDDR = direction;
			break;
		case GPIO_C:/** GPIO C is selected*/
			GPIOC->PDDR = direction;
			break;
		case GPIO_D:/** GPIO D is selected*/
			GPIOD->PDDR = direction;
			break;
		case GPIO_E:/** GPIO E is selected*/
			GPIOE->PDDR = direction;
			break;
		default:
			break;
	}

}
void GPIO_data_direction_pin(gpio_port_name_t portName, uint8 State, uint8 pin)
{
	switch(portName)
	{
		case GPIO_A:/** GPIO A is selected*/
			GPIOA->PDDR &= ~(TRUE << pin);
			GPIOA->PDDR |= State << pin;
			break;

		case GPIO_B:/** GPIO B is selected*/
			GPIOB->PDDR &= ~(TRUE << pin);
			GPIOB->PDDR |= State << pin;
			break;

		case GPIO_C:/** GPIO C is selected*/
			GPIOC->PDDR &= ~(TRUE << pin);
			GPIOC->PDDR |= State << pin;
			break;

		case GPIO_D:/** GPIO D is selected*/
			GPIOD->PDDR &= ~(TRUE << pin);
			GPIOD->PDDR |= State << pin;
			break;

		case GPIO_E:/** GPIO E is selected*/
			GPIOE->PDDR &= ~(TRUE << pin);
			GPIOE->PDDR |= State << pin;
			break;

		default:
			break;
	}
}
