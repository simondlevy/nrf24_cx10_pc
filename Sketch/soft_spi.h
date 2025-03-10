/*
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License.
 If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#define NOP() __asm__ __volatile__("nop")

// SPI outputs
#define MOSI_on PORTD |= _BV(3)  // PD3
#define MOSI_off PORTD &= ~_BV(3)// PD3
#define SCK_on PORTD |= _BV(4)   // PD4
#define SCK_off PORTD &= ~_BV(4) // PD4
#define CE_on PORTD |= _BV(5)    // PD5
#define CE_off PORTD &= ~_BV(5)  // PD5
#define CS_on PORTC |= _BV(1)    // PC1
#define CS_off PORTC &= ~_BV(1)  // PC1

// SPI input
#define  MISO_on (PINC & _BV(0)) // PC0


static uint8_t spi_write(uint8_t command) 
{
    uint8_t result=0;
    uint8_t n=8;
    SCK_off;
    MOSI_off;
    while(n--) {
        if(command & 0x80)
            MOSI_on;
        else
            MOSI_off;
        if(MISO_on)
            result |= 0x01;
        SCK_on;
        NOP();
        SCK_off;
        command = command << 1;
        result = result << 1;
    }
    MOSI_on;
    return result;
}

// read one byte from MISO
static uint8_t spi_read()
{
    uint8_t result=0;
    uint8_t i;
    MOSI_off;
    NOP();
    for(i=0;i<8;i++) {
        if(MISO_on) // if MISO is HIGH
        result = (result<<1)|0x01;
        else
        result = result<<1;
        SCK_on;
        NOP();
        SCK_off;
        NOP();
    }
    return result;
}

static void spi_write_address(uint8_t address, uint8_t data) 
{
    CS_off;
    spi_write(address);
    NOP();
    spi_write(data);
    CS_on;
}

static uint8_t spi_read_address(uint8_t address) 
{
    uint8_t result;
    CS_off;
    spi_write(address);
    result = spi_read();
    CS_on;
    return(result);
}
