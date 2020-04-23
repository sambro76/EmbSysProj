/**
 *Created in Fall 2019
 *Author:
 *  Samnang Chay
 *  Reference: Dr. Jason Losh
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

#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "i2c0.h"

// PortB masks
#define SDA_MASK 8
#define SCL_MASK 4

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void initI2c0()
{
    // Enable clocks
    SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;
    SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOB | SYSCTL_RCGC2_GPIOF;

    // Configure I2C
    GPIO_PORTB_DIR_R |= SDA_MASK | SCL_MASK;        // make bits 2 and 3 outputs
    GPIO_PORTB_DR2R_R |= SDA_MASK | SCL_MASK;       // set drive strength to 2mA
    GPIO_PORTB_DEN_R |= SDA_MASK | SCL_MASK;            // enable digital
    GPIO_PORTB_ODR_R |= SDA_MASK;                   // enable open drain outputs
    GPIO_PORTB_AFSEL_R |= SDA_MASK | SCL_MASK;         // configure auxiliary fn
    GPIO_PORTB_PCTL_R &= ~(GPIO_PCTL_PB2_M | GPIO_PCTL_PB3_M);
    GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB2_I2C0SCL | GPIO_PCTL_PB3_I2C0SDA;

    // Configure I2C0 peripheral
    I2C0_MCR_R = I2C_MCR_MFE;                    // master
    I2C0_MTPR_R = 19;                            // (40MHz/2) / (19+1) = 100kbps

    // Configure LED pins
    GPIO_PORTF_DIR_R = 0b01111; // bits 1, 2 and 3 are outputs, other pins are inputs
    GPIO_PORTF_DR2R_R = 0b01111; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = 0b11111;  // digital enable

    GPIO_PORTF_PUR_R = (1<<4 | 1<<0); //enable push-up PF0 & PF4
}

uint8_t readI2c0Register(uint8_t add, uint8_t reg)
{
    I2C0_MSA_R = add * 2;
    I2C0_MDR_R = reg;
    I2C0_MICR_R = I2C_MICR_IC;
    I2C0_MCS_R = I2C_MCS_START | I2C_MCS_RUN;
    while ((I2C0_MRIS_R & I2C_MRIS_RIS) == 0)
        ;
    I2C0_MSA_R = add * 2 + 1;
    I2C0_MICR_R = I2C_MICR_IC;
    I2C0_MCS_R = I2C_MCS_STOP | I2C_MCS_START | I2C_MCS_RUN;
    while ((I2C0_MRIS_R & I2C_MRIS_RIS) == 0)
        ;
    return I2C0_MDR_R;
}

void readI2c0Registers(uint8_t add, uint8_t reg, uint8_t *data, uint8_t size)
{
    uint8_t i;
    I2C0_MSA_R = add * 2;
    I2C0_MDR_R = reg;
    I2C0_MICR_R = I2C_MICR_IC;
    I2C0_MCS_R = I2C_MCS_START | I2C_MCS_RUN;
    while (!(I2C0_MRIS_R & I2C_MRIS_RIS))
        ;
    I2C0_MSA_R = add * 2 + 1;
    I2C0_MICR_R = I2C_MICR_IC;
    I2C0_MCS_R = I2C_MCS_DATACK | I2C_MCS_START | I2C_MCS_RUN;
    while (!(I2C0_MRIS_R & I2C_MRIS_RIS))
        ;
    data[0] = I2C0_MDR_R;
    for (i = 1; i < size - 1; i++)
    {
        I2C0_MICR_R = I2C_MICR_IC;
        I2C0_MCS_R = I2C_MCS_DATACK | I2C_MCS_RUN;
        while (!(I2C0_MRIS_R & I2C_MRIS_RIS))
            ;
        data[i] = I2C0_MDR_R;
    }
    I2C0_MICR_R = I2C_MICR_IC;
    I2C0_MCS_R = I2C_MCS_ADRACK | I2C_MCS_RUN;
    while (!(I2C0_MRIS_R & I2C_MRIS_RIS))
        ;
    data[size - 1] = I2C0_MDR_R;
}

void writeI2c0Register(uint8_t add, uint8_t reg, uint8_t data)
{
    I2C0_MSA_R = add * 2;
    I2C0_MDR_R = reg;
    I2C0_MICR_R = I2C_MICR_IC;
    I2C0_MCS_R = I2C_MCS_START | I2C_MCS_RUN;
    while ((I2C0_MRIS_R & I2C_MRIS_RIS) == 0)
        ;
    I2C0_MDR_R = data;
    I2C0_MICR_R = I2C_MICR_IC;
    I2C0_MCS_R = I2C_MCS_STOP | I2C_MCS_RUN;
    while (!(I2C0_MRIS_R & I2C_MRIS_RIS))
        ;
}

void writeI2c0Registers(uint8_t add, uint8_t reg, uint8_t data[], uint8_t size)
{
    uint8_t i;
    I2C0_MSA_R = add * 2;
    I2C0_MDR_R = reg;
    I2C0_MICR_R = I2C_MICR_IC;
    if (size == 0)
    {
        I2C0_MCS_R = I2C_MCS_STOP | I2C_MCS_START | I2C_MCS_RUN;
        while ((I2C0_MRIS_R & I2C_MRIS_RIS) == 0)
            ;
    }
    else
    {
        I2C0_MCS_R = I2C_MCS_START | I2C_MCS_RUN;
        while ((I2C0_MRIS_R & I2C_MRIS_RIS) == 0)
            ;
        for (i = 0; i < size - 1; i++)
        {
            I2C0_MDR_R = data[i];
            I2C0_MICR_R = I2C_MICR_IC;
            I2C0_MCS_R = I2C_MCS_RUN;
            while ((I2C0_MRIS_R & I2C_MRIS_RIS) == 0)
                ;
        }
        I2C0_MDR_R = data[size - 1];
        I2C0_MICR_R = I2C_MICR_IC;
        I2C0_MCS_R = I2C_MCS_STOP | I2C_MCS_RUN;
        while ((I2C0_MRIS_R & I2C_MRIS_RIS) == 0)
            ;
    }
}

bool pollI2c0Address(uint8_t add)
{
    I2C0_MSA_R = add * 2 + 1;
    I2C0_MICR_R = I2C_MICR_IC;
    I2C0_MCS_R = I2C_MCS_STOP | I2C_MCS_START | I2C_MCS_RUN;
    while ((I2C0_MRIS_R & I2C_MRIS_RIS) == 0)
        ;
    return !(I2C0_MCS_R & I2C_MCS_ERROR);
}

bool isI2c0Error()
{
    return !(I2C0_MCS_R & I2C_MCS_ERROR);
}

