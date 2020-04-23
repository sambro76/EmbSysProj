/**
 *Created in Fall 2019
 *Author:
 *  Samnang Chay
 *
 */
//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------
#include <stdbool.h>
#include <string.h>
#include "tm4c123gh6pm.h"
#include <time.h>
#include <math.h>
#include <miscFunction.h>
#include "uart0.h"
#include <stdio.h>
#include "wait.h"
#include "mpu9250.h"
#include <stdlib.h>

#define MAX_CHARS 80
#define FLASH_BASE_ADDR ((volatile uint32_t *)0x00020000)
#define randnum(min, max) ((rand() % (int)(((max) + 1) - (min))) + (min))

char str[MAX_CHARS];
uint8_t pos[2];
uint8_t count = 0;
char data[64] = { 0 };
uint32_t ss, NN=0, ii=0;

static uint16_t flashKey_ = 0;
char dataset[88];

uint16_t sampleCount=0;
uint32_t startPage = 1, key = 127, currentPg=0;

bool isValueArr(char* val) {
    uint8_t i, lnt=strlen(val);
    for (i=0; i<lnt; i++) {
        if ((val[i] < 48) || (val[i] > 57)) return false;
    }
    return true;
}

void parseString() {
    count = 0;
    uint8_t i, lnt=strlen(str);
    char p ='d', c;
    for (i = 0; i < lnt; i++){
        c = str[i];
        if (c>='a' && c<='z')
            c='a';
        else if (c>='0' && c<='9')
            c='n';
        else {
            c='d';
            str[i]=0;
        }

        if (p != c){
            if (p=='d' && c=='a'){
                pos[count]=i;
                count++;
            }
            else if (p=='d' && c=='n'){
                pos[count]=i;
                count++;
            }
        }
        p = c;
    }
}

char *getString(uint8_t arg){
    return &str[pos[arg+1]];
}

int myAtoi(char* str)
{
    int i, res = 0;
    for (i = 0; str[i] != '\0'; ++i)
        res = res * 10 + str[i] - '0';
    return res;
}

int myAtosi(char* str)
{
    int i, res = 0;
    for (i = 0; str[i] != '\0'; ++i)
        res = res * 10 + str[i] - '0';
    if(str[0]=='-') return (-1)*res;
    else return res;
}

char* myItoa(int val)
{
    static char buf[32] = { 0 };
    int i = 30;
    for (; val && i; --i, val /= 10)
        buf[i] = "0123456789abcdef"[val % 10];
    return &buf[i + 1];
}

uint32_t getValue(int arg){
    return myAtoi(getString(arg));
}

int getsValue(int arg){
    return myAtosi(getString(arg));
}

bool isCommand(char *cmd, uint8_t min){
    if (count > min){
        int i;
        for(i = 0; i < strlen(str); i++){
            if (cmd[i] != str[pos[0]+i])
                return false;
        }
        return true;
    }
    else
        return false;
}

void reset(){
    NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
}

struct tm currentTimeDate(){
    struct tm ts;
    uint32_t rtcCounter = HIB_RTCC_R;
    ts = *localtime(&rtcCounter);
    return ts;
}

void setDate(uint8_t M, uint8_t D, uint16_t Y){
    struct tm info = currentTimeDate();
    info.tm_mon = M - 1;      //built-in time variable
    info.tm_mday = D;
    info.tm_year = Y - 1900; //year counting from 1900
    uint32_t rtcCounter = mktime(&info);
    HIB_RTCLD_R = rtcCounter;
    while (!(HIB_CTL_R & HIB_CTL_WRC)) ;
}

void setTime(uint8_t hr, uint8_t mn, uint8_t s){
    struct tm ts = currentTimeDate();
    ts.tm_hour = hr;
    ts.tm_min = mn;
    ts.tm_sec = s;
    uint32_t rtcCounter = mktime(&ts);
    HIB_RTCLD_R = rtcCounter;
    while (!(HIB_CTL_R & HIB_CTL_WRC)) ;
}

void stopHibPeriodic(){
    HIB_IM_R &= ~HIB_IM_RTCALT0;
    while (!(HIB_CTL_R & HIB_CTL_WRC)) ;
    GREEN_LED = 0;
    HIB_CTL_R &= ~HIB_CTL_HIBREQ;
}

void initHibRTC1(uint32_t N, uint32_t s){  //RTC Match Functionality No Hibernation

    HIB_CTL_R &=~(HIB_CTL_RTCEN);
    while(!(HIB_CTL_R & HIB_CTL_WRC)) ;
    while(!(HIB_RIS_R & HIB_RIS_WC)) ;

    HIB_RTCM0_R = s + HIB_RTCC_R;
    while (!(HIB_CTL_R & HIB_CTL_WRC)) ;

    HIB_RTCLD_R = HIB_RTCC_R;
    while (!(HIB_CTL_R & HIB_CTL_WRC)) ;

    HIB_IM_R |= HIB_IM_RTCALT0;
    while (!(HIB_CTL_R & HIB_CTL_WRC)) ;

    HIB_RTCT_R = 0x7FFF;
    while (!(HIB_CTL_R & HIB_CTL_WRC)) ;

    HIB_CTL_R |= 0x00000041;
    while (!(HIB_CTL_R & HIB_CTL_WRC)) ;

    ss=s;
    NN=N;
    NVIC_EN1_R = (1 << ((INT_HIBERNATE - 16) & 31) );

}

void storeDateTimeData(){
    struct tm ts = currentTimeDate();

    HWREG(HIB_DATA + (0 * 4)) = ts.tm_year + 1900;
    while (!(HIB_CTL_R & HIB_CTL_WRC)) ;

    HWREG(HIB_DATA + (1 * 4)) = ts.tm_mon + 1;
    while (!(HIB_CTL_R & HIB_CTL_WRC)) ;

    HWREG(HIB_DATA + (2 * 4)) = ts.tm_mday;
    while (!(HIB_CTL_R & HIB_CTL_WRC)) ;

    HWREG(HIB_DATA + (3 * 4)) = ts.tm_hour;
    while (!(HIB_CTL_R & HIB_CTL_WRC)) ;

    HWREG(HIB_DATA + (4 * 4)) = ts.tm_min;
    while (!(HIB_CTL_R & HIB_CTL_WRC)) ;

    HWREG(HIB_DATA + (5 * 4)) = ts.tm_sec;
    while (!(HIB_CTL_R & HIB_CTL_WRC)) ;
}

uint32_t logMaskDataGet(uint16_t index){
    return HWREG(HIB_DATA + (index * 4 ));
}

void logMaskDataSet(uint8_t index, uint32_t valueSet){
    HWREG( HIB_DATA + index * 4 ) = valueSet;
    while (!(HIB_CTL_R & HIB_CTL_WRC)) ;
}

void initLogMask(){ //initialize log mask with 0 value
    if( (HWREG(HIB_DATA + (15 * 4))== NULL) ){
        uint8_t i;
        for(i=0; i<15; i++){
            logMaskDataSet(i, 0);
        }
    }
}

void clearLogMask(){
    uint8_t i;
    for(i=0; i<16; i++){
        logMaskDataSet(i, 0);
    }
}

uint32_t nextRandomPage(uint32_t a){
    int newbit = (( ( a >> (7-1) ) ^ ( a >> (7-2) ) ) & 1);
    a = ((a << 1) | newbit) & ( (1 << 7) - 1);
    return a;
}

uint32_t getNextPage(){
    if (HWREG(HIB_DATA + (13 * 4 ))==1) {  // leveling index 13
        uint32_t i;
        currentPg = HWREG(HIB_DATA + (10 * 4 ));
        for(i=0; i<HWREG(HIB_DATA + (12 * 4 )) ; i++) {
            currentPg = nextRandomPage(currentPg);
        }
        putsUart0(myItoa(currentPg));
        putsUart0(" ");
        currentPg = nextRandomPage(currentPg);
        putsUart0(myItoa(currentPg));
    }
    else {
        currentPg = HWREG(HIB_DATA + (10 * 4 )) + HWREG(HIB_DATA + (12 * 4 )) - 1;
        putsUart0(myItoa(currentPg));
        putsUart0(" ");
        currentPg +=1;
        if(currentPg>127) currentPg %= 127;
        putsUart0(myItoa(currentPg));
    }
    return currentPg;
}

void store(){
    if (HWREG(HIB_DATA + (13 * 4 ))==1 ) {
        srand(time(NULL));
        startPage = randnum(0, 127) + 1;
        logMaskDataSet(10, startPage);
        logMaskDataSet(12, sampleCount);

    }
    else {
        startPage = 1;
        logMaskDataSet(10, startPage);
        logMaskDataSet(12, sampleCount);
    }
}

void setLeveling(bool lvl){
    if(lvl){
        logMaskDataSet(13, 1);  //mask for leveling
    }
    else {
        logMaskDataSet(13, 0);
    }
}

void setSleepMode(bool mode){
    if(mode){
        logMaskDataSet(14, 1);  //sleep mode mask
    }
    else {
        logMaskDataSet(14, 0);
    }
}

void initHibSleep(uint32_t s){   //PINWEN or RTCWEN

    storeDateTimeData();
    logMaskDataSet(11, s);  //mask sleep time in 's' seconds

    HIB_RTCLD_R = 0;  // to reset RTCCounter = 0;
    while (!(HIB_CTL_R & HIB_CTL_WRC)) ;

    HIB_CTL_R |= HIB_CTL_RTCEN;
    while (!(HIB_CTL_R & HIB_CTL_WRC)) ;

    HIB_RTCM0_R = s;
    while (!(HIB_CTL_R & HIB_CTL_WRC)) ;

    HIB_CTL_R = ( (HIB_CTL_PINWEN | HIB_CTL_RTCWEN) | ( HIB_CTL_R & ~(HIB_CTL_PINWEN | HIB_CTL_RTCWEN | HIB_CTL_BATWKEN)));
    while (!(HIB_CTL_R & HIB_CTL_WRC)) ;

    if ( HWREG(HIB_DATA + (15 * 4)) == 's'){
        GREEN_LED = 0;
    }
    else if ( HWREG(HIB_DATA + (15 * 4)) == 't'){
        BLUE_LED = 0;
    }

    HIB_CTL_R |= HIB_CTL_HIBREQ;
}

void initWakeup(){
    if( (HWREG(HIB_DATA + (15 * 4)) == 's' ) || (HWREG(HIB_DATA + (15 * 4))=='t') ) {
        SYSCTL_RCGCHIB_R |= SYSCTL_RCGCHIB_R0;
        HIB_CTL_R |= HIB_CTL_CLK32EN;
        while(!(HIB_CTL_R & HIB_CTL_WRC)) ;
        HIB_CTL_R |= HIB_CTL_RTCEN;
        while (!(HIB_CTL_R & HIB_CTL_WRC)) ;

        setDate(HWREG(HIB_DATA + (1 * 4)), HWREG(HIB_DATA + (2 * 4)), HWREG(HIB_DATA + (0 * 4)));
        setTime(HWREG(HIB_DATA + (3 * 4)), HWREG(HIB_DATA + (4 * 4)), (HWREG(HIB_DATA + (5 * 4)) ) ); //+ HIB_RTCC_R

        if ( HWREG(HIB_DATA + (15 * 4)) == 's'){
            HIB_CTL_R &= ~HIB_CTL_HIBREQ;
            GREEN_LED = 1;
            saveSampleData();
            initHibSleep(HWREG(HIB_DATA + (11 * 4))); //use this register value to get 's' seconds sleep time
        }
        else if ( HWREG(HIB_DATA + (15 * 4)) == 't'){
            HIB_CTL_R &= ~HIB_CTL_HIBREQ;
            BLUE_LED = 1;
            saveSampleData();
            WoM();
            initHibSleep(0);
        }
    }
}

void flashEnable(){
    if (FLASH_BOOTCFG_R & 0x10)
        flashKey_ = 0xA442;
    else
        flashKey_ = 0x71D5;
}

int flashErase(int blockCount, uint32_t page)
{
    if (flashKey_ == 0) return -1;
    int i;
    for (i = 0; i < blockCount; i++) {
        FLASH_FMA_R &= 0xFFFC0000;
        FLASH_FMA_R |= (uint32_t)FLASH_BASE_ADDR + ((page-1) * 1024) + (i*1024);
        FLASH_FMC_R = (flashKey_ << 16) | 0x2;
        while (FLASH_FMC_R & 0x2) ;  // Poll the ERASE bit until it is cleared.
    }
    return 0;
}

int flashWrite(void* data, uint32_t wordCount, uint32_t page){
    if (flashKey_ == 0) return -1;
    int blockCount = ((wordCount * sizeof(uint32_t)) / 1024) + 1;
    flashErase(blockCount, page);

    int i;
    for (i = 0; i < wordCount; i++) {
        FLASH_FMD_R = ((volatile uint32_t*)data)[i];

        FLASH_FMA_R &= 0xFFFC0000;
        FLASH_FMA_R = (uint32_t)FLASH_BASE_ADDR + ((page-1) * 1024) + (i * sizeof(uint32_t)) ;

        FLASH_FMC_R = (flashKey_ << 16) | FLASH_FMC_WRITE; // Trigger a write operation
        while (FLASH_FMC_R & FLASH_FMC_WRITE) ;

    }
    putcUart0('p');
    return 0;
}

void saveSampleData() {
    char accel[20];
    char gyro[20];
    char mag[22];
    char temp[6];

    struct tm ts = currentTimeDate();
        strcpy(dataset,"\0");

        strcat(dataset, myItoa(ts.tm_year + 1900));
        strcat(dataset,"/");
        if (ts.tm_mon < 9)
        {
            strcat(dataset, "0");
            strcat(dataset, myItoa(ts.tm_mon + 1));
        }
        else
            strcat(dataset, myItoa(ts.tm_mon + 1));
        strcat(dataset,"/");
        if (ts.tm_mday < 10)
        {
            strcat(dataset, "0");
            strcat(dataset, myItoa(ts.tm_mday));
        }
        else
            strcat(dataset, myItoa(ts.tm_mday));
        strcat(dataset,",");

        if (ts.tm_hour == 0)
        {
            strcat(dataset, "00");
        }
        else if (ts.tm_hour < 10)
        {
            strcat(dataset, "0");
            strcat(dataset, myItoa(ts.tm_hour));
        }
        else
            strcat(dataset, myItoa(ts.tm_hour));
        strcat(dataset,":");
        if (ts.tm_min == 0)
        {
            strcat(dataset, "00");
        }
        else if (ts.tm_min < 10)
        {
            strcat(dataset, "0");
            strcat(dataset, myItoa(ts.tm_min));
        }
        else
            strcat(dataset, myItoa(ts.tm_min));
        strcat(dataset,":");
        if (ts.tm_sec == 0)
        {
            strcat(dataset, "00");
        }
        else if (ts.tm_sec < 10)
        {
            strcat(dataset, "0");
            strcat(dataset, myItoa(ts.tm_sec));
        }
        else
            strcat(dataset, myItoa(ts.tm_sec));
        strcat(dataset,",");

        if(HWREG(HIB_DATA + (6 * 4 ))==1){
            strcpy(accel,"\0");
            storeAccel(accel);
            strcat(dataset, accel);
        }

        if(HWREG(HIB_DATA + (7 * 4 ))==1){
            strcpy(gyro,"\0");
            storeGyro(gyro);
            strcat(dataset, gyro);
        }

        if(HWREG(HIB_DATA + (8 * 4 ))==1){
            strcpy(mag,"\0");
            storeMag(mag);
            strcat(dataset, mag);
        }

        if(HWREG(HIB_DATA + (9 * 4 ))==1){
            strcpy(temp,"\0");
            storeTemp(temp);
            strcat(dataset, temp);
        }
        putsUart0("\r\n");
        putsUart0(dataset);

        strncpy(dataset, dataset, sizeof(dataset));

        if( (logMaskDataGet(12)==0) || (!logMaskDataGet(12)) ){
            store(); //to set initial page
            flashWrite(dataset, sizeof(dataset) / sizeof(uint32_t), logMaskDataGet(10) );
            HWREG(HIB_DATA + (12 * 4 )) = 1;
            while (!(HIB_CTL_R & HIB_CTL_WRC)) ;
        }
        else {
            flashWrite(dataset, sizeof(dataset) / sizeof(uint32_t), getNextPage() );
            HWREG(HIB_DATA + (12 * 4 )) += 1;  //increase sample count
            while (!(HIB_CTL_R & HIB_CTL_WRC)) ;
            if(HWREG(HIB_DATA + (12 * 4 )) > 127) {
                HWREG(HIB_DATA + (12 * 4 )) %= 127;
                while (!(HIB_CTL_R & HIB_CTL_WRC)) ;
            }
        }
}

void flashRead(void* data, uint32_t wordCount, uint32_t page){
    int i;
    for (i = 0; i < wordCount; i++) {
        ((uint32_t *)data)[i] = (*(volatile uint32_t *)(0x00020000 + (0x400)*(page-1) + (i*4) ) );
    }
}

void readSampleData(uint32_t page){
    if(page>127){
        // putsUart0("Allocated memory space is in upper half of flash memory area. "
        // "\r\n(max.127KiB), so any page over 127, will be %127. \r\n");
        page %= 127;
    }
    flashRead(dataset, sizeof(dataset) / sizeof(uint32_t), page);
    putsUart0(dataset);
}

void hibHandler(){
    if(NN==0) { //periodically sample every 'ss' seconds
        HIB_IC_R |= (1<<0);
        while (!(HIB_CTL_R & HIB_CTL_WRC)) ;

        GREEN_LED ^= 1;
        saveSampleData(); //write to flash

        HIB_RTCM0_R = ss + HIB_RTCC_R;
        while (!(HIB_CTL_R & HIB_CTL_WRC)) ;
        HIB_RTCLD_R = HIB_RTCC_R;
        while (!(HIB_CTL_R & HIB_CTL_WRC)) ;
    }
    else {  //periodically sample every 'ss' seconds with total NN samples
        ii++;
        HIB_IC_R |= (1<<0);
        while (!(HIB_CTL_R & HIB_CTL_WRC)) ;

        GREEN_LED ^= 1;
        saveSampleData(); //write to flash

        HIB_RTCM0_R = ss + HIB_RTCC_R;
        while (!(HIB_CTL_R & HIB_CTL_WRC)) ;
        HIB_RTCLD_R = HIB_RTCC_R;
        while (!(HIB_CTL_R & HIB_CTL_WRC)) ;

        if(ii==NN) {
            stopHibPeriodic();
            ii=0;
            NN=0;
            ss=0;
            putsUart0("\r\n\r\nCommand $: ");
        }
    }
}
