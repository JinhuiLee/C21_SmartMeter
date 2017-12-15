/*
 * main.c
 *
 * Created: 12/20/2016 1:38:29 PM
 *  Author: xu
 */ 

/**
 * \file
 *
 * \brief SAM CAN basic Quick Start
 *
 * Copyright (C) 2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel micro-controller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
//#include "battery.h"
#include <asf.h>
#include <string.h>
#include <conf_can.h>
#include "sys_event_interrupt_hook.h"
#include <stdbool.h>

//! [module_inst]
static struct usart_module cdc_instance;
static struct usart_module usart_instance;


#define MAX_RX_BUFFER_LENGTH 20
#define MAX_CMD_LENGTH 100
volatile uint8_t rx_buffer[MAX_RX_BUFFER_LENGTH];
void usart_read_callback(struct usart_module *const usart_module);
void usart_write_callback(struct usart_module *const usart_module);
void configure_usart_callbacks(void);
void configure_usart(void);


uint8_t buffer_index = 0;
uint8_t cmd_length = 0;
uint8_t command_buffer[MAX_CMD_LENGTH] = {0};
uint8_t cmd_start = 0;
uint8_t cmd_received = 0;

void clear_rx_buffer(){
	memset(rx_buffer, 0, MAX_RX_BUFFER_LENGTH);
}

void clear_cmd_buffer(){
	memset(command_buffer, 0, MAX_CMD_LENGTH);
}


void usart_read_callback(struct usart_module *const usart_module)
{	for (int i = 0; i < MAX_RX_BUFFER_LENGTH; i++) {		uint8_t data = rx_buffer[i];		printf("_%X_", data);		if (data == 0x68 && buffer_index < 9) {
			cmd_start = true; 
			cmd_received = false; 
			clear_cmd_buffer();
			buffer_index=0;
		}
		if (cmd_start) { 
			command_buffer[buffer_index++] = data;
			if (data == 0x16) {
				cmd_start = false;
				cmd_received = true;
				cmd_length = buffer_index;
				buffer_index=0;
				
			}
		}				if (cmd_received == true) {
			printf("\n\n");
			for (int i = 0; i < cmd_length; i++) {
				printf("%X ", command_buffer[i]);
			}
			printf("\n\n");
			cmd_received = false;
			clear_cmd_buffer();
		}	}	clear_rx_buffer();			
}


void usart_write_callback(struct usart_module *const usart_module)
{
	//port_pin_toggle_output_level(LED_0_PIN);
}


//#define EXT1_UART_MODULE              SERCOM3
//#define EXT1_UART_SERCOM_MUX_SETTING  USART_RX_1_TX_0_XCK_1
//#define EXT1_UART_SERCOM_PINMUX_PAD0  PINMUX_PA22C_SERCOM3_PAD0
//#define EXT1_UART_SERCOM_PINMUX_PAD1  PINMUX_PA23C_SERCOM3_PAD1
//#define EXT1_UART_SERCOM_PINMUX_PAD2  PINMUX_UNUSED
//#define EXT1_UART_SERCOM_PINMUX_PAD3  PINMUX_UNUSED
//#define EXT1_UART_SERCOM_DMAC_ID_TX   SERCOM3_DMAC_ID_TX
//#define EXT1_UART_SERCOM_DMAC_ID_RX   SERCOM3_DMAC_ID_RX

void configure_usart_callbacks(void)
{
	usart_register_callback(&usart_instance,
	usart_write_callback, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_register_callback(&usart_instance,
	usart_read_callback, USART_CALLBACK_BUFFER_RECEIVED);
	usart_enable_callback(&usart_instance, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_enable_callback(&usart_instance, USART_CALLBACK_BUFFER_RECEIVED);
}


void configure_usart(void)
{
	struct usart_config config_usart;
	usart_get_config_defaults(&config_usart);
	config_usart.baudrate = 19200;
	config_usart.mux_setting = EXT1_UART_SERCOM_MUX_SETTING;
	config_usart.pinmux_pad0 = EXT1_UART_SERCOM_PINMUX_PAD0;
	config_usart.pinmux_pad1 = EXT1_UART_SERCOM_PINMUX_PAD1;
	config_usart.pinmux_pad2 = EXT1_UART_SERCOM_PINMUX_PAD2;
	config_usart.pinmux_pad3 = EXT1_UART_SERCOM_PINMUX_PAD3;
	while (usart_init(&usart_instance, EXT1_UART_MODULE, &config_usart) != STATUS_OK) {
	}
	usart_enable(&usart_instance);
	configure_usart_callbacks();
}






//******************************************** UART CONFIG *******************************************
//! [cdc_setup]
static void configure_usart_cdc(void)
{

	struct usart_config config_cdc;
	usart_get_config_defaults(&config_cdc);
	config_cdc.baudrate	 = 9600;
	config_cdc.mux_setting = EDBG_CDC_SERCOM_MUX_SETTING;
	config_cdc.pinmux_pad0 = EDBG_CDC_SERCOM_PINMUX_PAD0;
	config_cdc.pinmux_pad1 = EDBG_CDC_SERCOM_PINMUX_PAD1;
	config_cdc.pinmux_pad2 = EDBG_CDC_SERCOM_PINMUX_PAD2;
	config_cdc.pinmux_pad3 = EDBG_CDC_SERCOM_PINMUX_PAD3;
	stdio_serial_init(&cdc_instance, EDBG_CDC_MODULE, &config_cdc);
	usart_enable(&cdc_instance);
}
//! [cdc_setup]
//******************************************* UART CONFIG END *******************************************

int main(void)
{

//! [setup_init]
	system_init();
	system_interrupt_enable_global();
	
//! setup USART
	configure_usart_cdc();
	configure_usart();
//! [setup_init]

	uint8_t string[] = "Hello World!\r\n";
	//usart_write_buffer_wait(&usart_instance, string, sizeof(string));
	
//! [main_loop]
	while(1) {
		port_pin_set_output_level(LED_0_PIN, cmd_received);
		usart_read_buffer_job(&usart_instance,(uint8_t *)rx_buffer, MAX_RX_BUFFER_LENGTH);
		
		
	}
//! [main_loop]
}

