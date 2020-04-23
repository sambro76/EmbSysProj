/**
 *Created in Fall 2019
 *Author:
 *  Samnang Chay
 *  Reference: Dr. Jason Losh, github.com/kriswiner/MPU6050 and freescale.com
 *
 */
//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------
// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz
//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include "wait.h"
#include "tm4c123gh6pm.h"
#include "uart0.h"
#include "i2c0.h"
#include "mpu9250.h"
#include <miscFunction.h>

#define MAX_CHARS 80
char str[MAX_CHARS];

// Initialize Hardware
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);
    SYSCTL_GPIOHBCTL_R = 0;

    SYSCTL_RCGCHIB_R |= SYSCTL_RCGCHIB_R0;
    HIB_CTL_R |= (HIB_CTL_CLK32EN | HIB_CTL_RTCEN);  //enable clock & the module  | HIB_CTL_RTCEN
    while (!(HIB_CTL_R & HIB_CTL_WRC)) ;  //spin until write complete bit is set
    HIB_CTL_R |= HIB_CTL_VDD3ON;
    while(!(HIB_CTL_R & HIB_CTL_WRC)) ;
    flashEnable();
}

void getsUart0(char str[], uint8_t size)
{
    uint8_t count = 0;
    bool end = false;
    char c;
    while(!end)
    {
        c = getcUart0();
        end = (c == 13) || (count == size);
        if (!end)
        {
            if ((c == 8 || c == 127) && count > 0)
                count--;
            if (c >= ' ' && c < 127)
                str[count++] = c;
        }
    }
    str[count] = '\0';
}

bool doCommand(){
    bool ok = true;
    uint8_t i;
    uint8_t add=0, reg, dat;

    if (isCommand("poll",0)){
        //putsUart0("Devices found: ");
        for (i = 4; i < 119; i++){
            if (pollI2c0Address(i)){
                add++;
                sprintf(str, "0x%02x ", i);
                if(add==1){
                    putsUart0("Devices found: ");
                    putsUart0(str);
                }
                else
                    putsUart0(str);
            }
        }
        if(add==0)
            putsUart0("No device found!");
        putsUart0("\r\n");
    }
    else if (isCommand("write",3)){
        if (isValueArr(getString(0)) && isValueArr(getString(1)) && isValueArr(getString(2))) {
            add = getValue(0);
            reg = getValue(1);
            dat = getValue(2);
            writeI2c0Register(add, reg, dat);
            sprintf(str, "Writing 0x%02hhx to address 0x%02hhx, register 0x%02hhx\r\n", dat, add, reg);
            putsUart0(str);
        }
        else
            putsUart0("Error in write command arguments\r\n");
    }
    else if (isCommand("read",2)){

        if (isValueArr(getString(0)) && isValueArr(getString(1))){
            add = getValue(0);
            reg = getValue(1);
            dat = readI2c0Register(add, reg);
            sprintf(str, "Read 0x%02hhx from address 0x%02hhx, register 0x%02hhx\r\n", dat, add, reg);
            putsUart0(str);
        }
        else
            putsUart0("Error in read command arguments\r\n");
    }
    else if (isCommand("help",0)){
        putsUart0("Usage:\t ...\r\n");


    }
    else if(isCommand("time", 0)){  //step 5
        if(isCommand("time", 4)){
            ok = false;
            putsUart0("\r\nNote: Input time format is HH MM SS\r\n");
            return ok;
        }
        else if(isCommand("time", 3)){
            if (getValue(0) <= 23 && getValue(1) <= 59 && getValue(2) <= 59)
            {
                setTime(getValue(0), getValue(1), getValue(2));
            }
            else
            {
                ok = false;
                putsUart0("\r\nNote: Input time format is HH MM SS\r\n");
                return ok;
            }
        }
        else if (isCommand("time", 1) || isCommand("time", 2))  {
            ok = false;
            putsUart0("\r\nNote: Input time format is HH MM SS\r\n");
            return ok;
        }
        else if (isCommand("time", 0)) {   //step 5
            struct tm ts = currentTimeDate();
            if(ts.tm_hour==0) putsUart0("0"); else putsUart0(myItoa(ts.tm_hour));
            putcUart0(':');
            if(ts.tm_min==0) putsUart0("0");else putsUart0(myItoa(ts.tm_min));
            putcUart0(':');
            if(ts.tm_sec==0) putsUart0("0"); else putsUart0(myItoa(ts.tm_sec));
        }
    }

    else if(isCommand("date", 0)){  //step 5
        if(isCommand("date", 4)){
            ok = false;
            putsUart0("\r\nNote: Input date format is MM DD YYYY\r\n");
            return ok;
        }
        else if(isCommand("date", 3)){
            if(getValue(0)>=1 && getValue(0) <=12 && getValue(1) >= 1 && getValue(1) <= 31 && getValue(2)>=1900 && getValue(2) <=2020)
            {
                setDate( getValue(0), getValue(1), getValue(2) );
            }
            else
            {
                ok=false;
                putsUart0("\r\nNote: Input date format is MM DD YYYY\r\n");
                return ok;
            }
        }
        else if (isCommand("date", 1) || isCommand("date", 2))  {
            ok = false;
            putsUart0("\r\nNote: Input date format is MM DD YYYY\r\n");
            return ok;
        }
        else if (isCommand("date", 0)) {
            struct tm ts = currentTimeDate();
            putsUart0(myItoa(ts.tm_mon+1));
            putcUart0('/');
            if(ts.tm_mday==0) putsUart0("0"); else putsUart0(myItoa(ts.tm_mday));
            putcUart0('/');
            putsUart0(myItoa(ts.tm_year+1900));
        }
    }
    else if (isCommand("reset", 0)){  //step 6
        reset();
    }
    else if (isCommand("temp",0)){  //step 7
        readTempData();
    }
    else if (isCommand("accel",0)){  //step 8
        callAccel();
    }
    else if (isCommand("gyro",0)){  //step 8
        callGyro();
    }
    else if (isCommand("compass",0)){  //step 8
        while(1) {
            callCompass();
            waitMicrosecond(500000);
        }
    }
    else if (isCommand("periodic",1)){  //step 9: continuously sample the data within 's' seconds
        if(isValueArr(getString(0)) ){
            if( (HWREG(HIB_DATA + (6 * 4 ))==1) || (HWREG(HIB_DATA + (7 * 4 ))==1) || (HWREG(HIB_DATA + (8 * 4 ))==1) || (HWREG(HIB_DATA + (9 * 4 ))==1) ){
                waitMicrosecond(10000);
                if( (HWREG(HIB_DATA + (14 * 4 ))==1) ) {
                    if(getValue(0)<2) {
                        putsUart0("\r\nTo get stable periodic sleep, time requires at least 2s.");
                    }
                    else {
                        HWREG(HIB_DATA + (15 * 4)) = 's'; //to identify the wakeup is from sleep/hibernation & periodic mode
                        while (!(HIB_CTL_R & HIB_CTL_WRC)) ;

                        initHibSleep( getValue(0) );   //Using RTC with wakeup pin & external interrupt for Hibernation
                    }
                }
                else {
                    putsUart0("\r\nType 'stop' to exit periodic sampling\r\n");
                    waitMicrosecond(100000);
                    initHibRTC1(0, getValue(0));  //Using RTC Match Functionality with No Hibernation with interrupt handler
                }
            }
            else {
                putsUart0("Note: there is nothing to record. Please check log mask.");
            }
        }
        else ok=false;
    }
    else if (isCommand("sample",2)){  //step 10: to sample only N samples within 's' seconds
        if(isValueArr(getString(0)) && isValueArr(getString(1))){
            if( (HWREG(HIB_DATA + (6 * 4 ))==1) || (HWREG(HIB_DATA + (7 * 4 ))==1) || (HWREG(HIB_DATA + (8 * 4 ))==1) || (HWREG(HIB_DATA + (9 * 4 ))==1) ){
                waitMicrosecond(10000);
                if( (HWREG(HIB_DATA + (14 * 4 ))==1) ) {
                    if(getValue(1)<2) {
                        putsUart0("\r\nTo get stable sleep, time requires at least 2s.");
                    }
                    else {
                        HWREG(HIB_DATA + (15 * 4)) = 's'; //to identify the wakeup is from sleep/hibernation & periodic mode
                        while (!(HIB_CTL_R & HIB_CTL_WRC)) ;

                        initHibSleep( getValue(0) );   //Using RTC with wakeup pin & external interrupt for Hibernation
                    }
                }
                else {
                    initHibRTC1(getValue(0), getValue(1));
                }
            }
            else {
                putsUart0("Note: there is nothing to record. Please check log mask.");
            }
        }
        else ok=false;
    }

    else if (isCommand("gating",3)){  //step 11
        if ( (strcmp(getString(0), "ax") == 0) || (strcmp(getString(0), "ay") == 0) || (strcmp(getString(0), "az") == 0) ) {
            if ( ((strcmp(getString(1), "gt") == 0) || (strcmp(getString(1), "lt") == 0)) && isValueArr(getString(2)) ) {

                gating(getString(0), getString(1), getsValue(2));
            }
            else
                ok = false;
        }
        else if ( (strcmp(getString(0), "temp") == 0) ) {
            if ( ((strcmp(getString(1), "gt") == 0) || (strcmp(getString(1), "lt") == 0)) && isValueArr(getString(2)) ) {

                gating(getString(0), getString(1), getsValue(2));
            }
            else
                ok = false;
        }
        else
            ok = false;
    }
    else if(isCommand("hysteresis", 1)){  //step 12
        if(isValueArr(getString(0))){

            hysteresis( getValue(0) );
        }
        else ok=false;
    }
    else if(isCommand("trigger", 0)){  //step 13
        trigger();
    }

    else if(isCommand("stop",0)){  //step 14
        //stopTimer0();
        GREEN_LED = 0;
        BLUE_LED = 0;
        RED_LED = 0;
        stopHibPeriodic(); //to stop sample
    }
    /*completed for bonus steps:
     * a. used I2C tool, turn on I2C bypass signal and able to read magnetometer
     * b. read acceleration, scale data showing magnitude of ~1g
     * c. read gyro for gx, gy, gz
     * d. done to display compass with NE, NW, SE, SW direction.
    */
    else if(isCommand("log", 1)){  //step 15
        if(isCommand("log", 2)) {
            if(strcmp(getString(0), "accel")==0 ) { //on offset #7
                if(strcmp(getString(1),"on")==0){
                    logMaskDataSet(6, 1);
                }
                else if(strcmp(getString(1),"off")==0){
                    logMaskDataSet(6, 0);
                }
                else ok=false;
            }
            else if(strcmp(getString(0), "gyro")==0) {
                if(strcmp(getString(1),"on")==0){
                    logMaskDataSet(7, 1);
                }
                else if(strcmp(getString(1),"off")==0){
                    logMaskDataSet(7, 0);
                }
                else ok=false;
            }
            else if(strcmp(getString(0), "mag")==0) {
                if(strcmp(getString(1),"on")==0){
                    logMaskDataSet(8, 1);
                }
                else if(strcmp(getString(1),"off")==0){
                    logMaskDataSet(8, 0);
                }
                else ok=false;
            }
            else if(strcmp(getString(0), "temp")==0) {
                if(strcmp(getString(1),"on")==0){
                    logMaskDataSet(9, 1);
                }
                else if(strcmp(getString(1),"off")==0){
                    logMaskDataSet(9, 0);
                }
                else ok=false;
            }
            else if(strcmp(getString(0), "all")==0) {
                uint8_t i;
                if(strcmp(getString(1),"on")==0){
                    for(i=0; i<4; i++) logMaskDataSet(i+6, 1);
                }
                else if(strcmp(getString(1),"off")==0){
                    for(i=0; i<4; i++) logMaskDataSet(i+6, 0);
                }
                else ok=false;
            }
            else ok=false;
        }
        else if(isCommand("log", 1)) {
            if(strcmp(getString(0), "show")==0) {
                int i;
                putsUart0("\r\nLog bit set shown here:\r\n");
                for (i = 0; i < 10; i++) {
                    putsUart0( myItoa(i) );
                    putsUart0("\t");
                }
                putsUart0("\r\nYYYY\tMM\tDD\tHH\tMN\tSS\tAccl\tGyro\tMagn\tTemp\r\n");
                for (i = 0; i < 10; i++) {
                    putsUart0(myItoa(HWREG(HIB_DATA + (i * 4 ))));
                    putsUart0("\t");
                }
                putsUart0("\r\n");
                for (i = 10; i < 16; i++) {
                    putsUart0( myItoa(i) );
                    putsUart0("\t");
                }
                putsUart0("\r\nsPage\tsTime\t#sampl\tlevel\tsMode\tsStatus\r\n");
                for (i = 10; i < 16; i++) {
                    putsUart0(myItoa(HWREG(HIB_DATA + (i * 4 ))));
                    putsUart0("\t");
                }
                putsUart0("\r\n");
            }
            else if(strcmp(getString(0), "clear")==0){
                clearLogMask();
            }
        }
        else ok=false;
    }
    else if(isCommand("store", 0)){  //step 16: store log mask
        store();
    }

    else if(isCommand("wdata", 0)){  //step 17: write data once to flash
        if ((HWREG(HIB_DATA + (6 * 4 )) == 1) || (HWREG(HIB_DATA + (7 * 4 )) == 1) || (HWREG(HIB_DATA + (8 * 4 )) == 1) || (HWREG(HIB_DATA + (9 * 4 )) == 1)) {
                saveSampleData();
                waitMicrosecond(10000);
        }
        else {
            putsUart0("Nothing to record, please check your log mask");
        }
    }

    else if(isCommand("leveling",1)){  //step 18: leveling
        if(strcmp(getString(0),"on")==0){
            setLeveling(true);
        }
        else if(strcmp(getString(0),"off")==0){
            setLeveling(false);
        }
        else ok=false;
    }

    else if(isCommand("sleep", 1)){  //step 19: sleep command
        if(strcmp(getString(0),"on")==0){
            setSleepMode(true);
        }
        else if(strcmp(getString(0),"off")==0){
            setSleepMode(false);
        }
        else ok=false;
    }

    else if(isCommand("rdata", 1)){  //step 20: read data from flash memory
        if (isValueArr(getString(0))) {
            readSampleData(getValue(0));
        }
        else if( strcmp(getString(0),"all")==0 ){
            uint16_t i;
            for(i=1; i<128; i++){
                readSampleData(i);
                putsUart0("\r\n");
            }
        }
        else
            ok = false;
    }

    else ok = false;

    return ok;
}

int main(void){
    initHw();
    initUart0();
    initI2c0();
    initLogMask();
    initWakeup();
    putsUart0("\r\n------------------------------------------------------");
    putsUart0("\r\n\tWelcome to Data Logger with Wear Leveling");
    putsUart0("\r\nThe purpose is to retrieve periodically accelero-gyro \r\nand magneto data and ambient temperature, then write \r\nto flash memory.");
    putsUart0("\r\n------------------------------------------------------");

    while (1){
        putsUart0("\r\nCommand $: ");
        getsUart0(str, MAX_CHARS);
        putsUart0(str);
        putsUart0("\r\n");
        parseString();
        if (!doCommand()) putsUart0("Command not found! ");
        putsUart0("\r\n");
    }
}
