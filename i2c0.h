/**
 *Created in Fall 2019
 *Author:
 *  Samnang Chay
 */
//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL with LCD/Keyboard Interface
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// I2C devices on I2C bus 0 with 2kohm pullups on SDA and SCL

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#ifndef I2C0_H_
#define I2C0_H_

#include <stdint.h>
#include <stdbool.h>
#include "wait.h"

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void initI2c0();
void writeI2c0Register(uint8_t add, uint8_t reg, uint8_t data);
void writeI2c0Registers(uint8_t add, uint8_t reg, uint8_t data[], uint8_t size);
uint8_t readI2c0Register(uint8_t add, uint8_t reg);
void readI2c0Registers(uint8_t add, uint8_t reg, uint8_t *data, uint8_t size);

bool pollI2c0Address(uint8_t add);
bool isI2c0Error();


#endif
