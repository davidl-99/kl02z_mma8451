/*
 * Copyright 2016-2021 NXP
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
 * @file    MKL02Z32xxx4_Project_mma8451.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL02Z4.h"
#include "fsl_debug_console.h"

#include "sdk_hal_gpio.h"
#include "sdk_hal_uart0.h"
#include "sdk_hal_i2c0.h"

#define MMA851_I2C_DEVICE_ADDRESS    0X1D
#define MMA8451_WHO_AM_I_MEMORY_ADDRESS    0X0D

//DEFINICION DE CADA DIRECCION DE MEMORIA DE CADA EJE TANTO DEL MAS SIGNIFICATIVO COMO EL MENOS SIGNIFICATIVO

#define OUT_X_MSB	0x01
#define OUT_X_LSB	0x02
#define OUT_Y_MSB	0x03
#define OUT_Y_LSB	0x04
#define OUT_Z_MSB	0x05
#define OUT_Z_LSB	0x06

#define CTRL_REG1 	0x2A

int main(void) {

	status_t status;
	status_t status_del_acelerometro;
	uint8_t nuevo_byte_uart;
	uint8_t nuevo_dato_i2c;

	//variables del eje x
	uint16_t outx_msb;
	uint16_t outx_lsb;
	uint16_t outx_final;

	//variables del eje y
	uint16_t outy_msb;
	uint16_t outy_lsb;
	uint16_t outy_final;

	//variables del eje z
	uint16_t outz_msb;
	uint16_t outz_lsb;
	uint16_t outz_final;

  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif

    (void)uart0Inicializar(115200); //115200 bps
    (void)i2c0MasterInit(100000);   //100 kbps
    status_del_acelerometro=i2c0MasterWriteByte(MMA851_I2C_DEVICE_ADDRESS, CTRL_REG1, 1);

    PRINTF("Usar teclado para controlar LEDS\r\n");
    PRINTF("r-R led ROJO\r\n");
    PRINTF("v-V led VERDE\r\n");
    PRINTF("a-A led AZUL\r\n");
    PRINTF("M buscar acelerometro\r\n");
    PRINTF("X-x posicion del eje X del  acelerometro\r\n");
    PRINTF("Y-y posicion del eje Y del acelerometro\r\n");
    PRINTF("Z-z posicion del eje Z del acelerometro\r\n");

	while (1) {
		if (uart0CuantosDatosHayEnBuffer() > 0) {
			status = uart0LeerByteDesdeBuffer(&nuevo_byte_uart);
			if (status == kStatus_Success) {
				printf("dato:%c\r\n", nuevo_byte_uart);
				switch (nuevo_byte_uart) {
				case 'a':
				case 'A':
					gpioPutToggle(KPTB10);
					break;

				case 'v':
					gpioPutHigh(KPTB7);
					break;
				case 'V':
					gpioPutLow(KPTB7);
					break;

				case 'r':
					gpioPutValue(KPTB6, 1);
					break;
				case 'R':
					gpioPutValue(KPTB6, 0);
					break;

				case 'M':
					i2c0MasterReadByte(&nuevo_dato_i2c, MMA851_I2C_DEVICE_ADDRESS, MMA8451_WHO_AM_I_MEMORY_ADDRESS);

					if(nuevo_dato_i2c==0x1A)
						printf("MMA8451 encontrado!!\r\n");
					else
						printf("MMA8451 error\r\n");
					break;

				case 'X':
				case 'x':
					i2c0MasterReadByte(&outx_msb, MMA851_I2C_DEVICE_ADDRESS,
							OUT_X_MSB);
					i2c0MasterReadByte(&outx_lsb, MMA851_I2C_DEVICE_ADDRESS,
							OUT_X_LSB);

					outx_msb <<= 8;
					outx_final = outx_msb | outx_lsb;
					outx_final >>= 2;
					printf("Posicion en el eje X es: %d\r\n", outx_final);
					break;

				case 'Y':
				case 'y':
					i2c0MasterReadByte(&outy_msb, MMA851_I2C_DEVICE_ADDRESS,
					OUT_Y_MSB);
					i2c0MasterReadByte(&outy_lsb, MMA851_I2C_DEVICE_ADDRESS,
					OUT_Y_LSB);

					outy_msb <<= 8;
					outy_final = outy_msb | outy_lsb;
					outy_final >>= 2;
					printf("Posicion en el eje Y es: %d\r\n", outy_final);
					break;

				case 'Z':
				case 'z':
					i2c0MasterReadByte(&outz_msb, MMA851_I2C_DEVICE_ADDRESS,
					OUT_Z_MSB);
					i2c0MasterReadByte(&outz_lsb, MMA851_I2C_DEVICE_ADDRESS,
					OUT_Z_LSB);

					outz_msb <<= 8;
					outz_final = outz_msb | outz_lsb;
					outz_final >>= 2;
					printf("Posicion en el eje Z es: %d\r\n", outz_final);
					break;
				}
			} else {
				printf("error\r\n");
			}
		}
	}
    return 0 ;
}
/*----------------------------------------/
 * comentarios de este procedimiento:
 * outx_msb <<= 8;
   outx_final = outx_msb | outx_lsb;
   outx_final >>= 2;

   la variable outx_msb (pero puede ser y o z) contiene los valores mas significativos de la posición en cualquier eje
   este dato el acelerometro lo entrega en 8 bits pero se guarda en una variable de 16 bits entonces si el acelerometro
   enrega por ejemplo 10111001 la variable lo guarda en 16 bits 0000000010111001. Luego con outx_msb <<= 8; se le hace un
   corriemiento de 8 veces a esta variable de 16 bits quedando asi: 1011100100000000.

   la variable outx_lsb (pero funciona tanto para y o z) contiene los valores menos significativos de la posicion del eje
   tambien los entrega el acelerometro de 8 bits pero en realidad se toman 6 porque los dos primeros bits son 00
   ejemplo 11100100 al guardarse en una variable de 16 bits queda 0000000011100100.

   Luego se realiza una or entre la variable outx_msb corrida 8 veces a la izquierda y la variable outx_lsb
   outx_final = outx_msb | outx_lsb;

   or   1011100100000000
        0000000011100100
        ----------------
        1011100111100100

   Mediante la or es posible unir los valores mas significativos y los menos significativos de cada eje

   outx_final >>= 2; simplemte corre los valores completos de la posicion del eje dos posiciones ya que los dos primeros
   valores siempre van a ser cero
 */
