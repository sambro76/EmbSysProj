/**
 *Created in Fall 2019
 *Author:
 *  Samnang Chay
 *
 */
#ifndef MiscFUNCTION_H_
#define MiscFUNCTION_H_

#include <stdint.h>
#include <stdbool.h>

#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define SW2_BTN      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 0*4)))
#define SW1_BTN      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))
#define HWREG(x)     (*((volatile uint32_t *)(x)))
#define HIB_DATA     0x400FC030

void initLogMask();
void logMaskDataSet(uint8_t index, uint32_t valueSet);
uint32_t logMaskDataGet(uint16_t index);
void store();
void setLeveling(bool lvl);
uint32_t getNextPage();
void setSleepMode(bool mode);

void parseString();
bool isCommand(char *cmd, uint8_t min);
bool isValueArr(char* val);

struct tm currentTimeDate();
void setDate(uint8_t M, uint8_t D, uint16_t Y);
void setTime(uint8_t hr, uint8_t mn, uint8_t s);
int getsValue(int arg);
uint32_t getValue(int arg);
int myAtoi(char* str);
char* myItoa(int val);

void reset();
void stopTimer0();
char *getString(uint8_t argN);
void initTimer0(uint32_t s);
void initHibRTC1(uint32_t N, uint32_t s);

void initHibSleep(uint32_t s);
void initWakeup();
//void TIMER0A_Handler();
void hibHandler();
void stopHibPeriodic();
void clearLogMask();

uint32_t nextRandomPage(uint32_t a);
void flashEnable(void);
void flashRead(void* data, uint32_t wordCount, uint32_t page);
int flashWrite(void* data, uint32_t wordCount, uint32_t page);
void saveSampleData();
void readSampleData(uint32_t page);

#endif /* HELPERFUNCTION_H_ */
