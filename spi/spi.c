/*
 * spi.c
 * 
 * Copyright 2013 Shimon <shimon@monistit.com>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 * 
 * 
 */
#include "spi.h"

#if SPI_CONFIG_AS_MASTER
void spi_init()
{	
	/* Setup SPI I/O pins. */
	/* Set SCK low. */
	SPI_PORT &= ~(_BV(SPI_SCK));
	/* Set SCK as output, MOSI as output, SS must be output for Master mode to work. */
	SPI_DDR |= (_BV(SPI_SCK) | _BV(SPI_MOSI) | _BV(ENC28J60_SPI_SS) | _BV(ENC28J60_SPI_SS) | _BV(RFID_SPI_SS));
	/* Set MISO as input. */
	SPI_DDR &= ~(_BV(SPI_MISO));
	/* Enable SPI, master mode. */
	SPCR = _BV(SPE) | _BV(MSTR) | _BV(SPR0);
}


uint8_t spi_transmit(uint8_t data)
{
	SPDR = data;
	while(!(SPSR & (1<<SPIF)));
	
	return SPDR;
}

#else
void spi_init()
{
	SPI_DDR = (1<<SPI_MISO);
	SPCR = (1<<SPE);
}

uint8_t spi_transmit(uint8_t data)
{
	while(!(SPSR & (1<<SPIF)));
	return SPDR;
}
#endif
