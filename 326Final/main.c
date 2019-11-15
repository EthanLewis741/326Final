/* DriverLib Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "driverlib.h"

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>
#include "ST7735.h"
#include "Functions.h"

#include <msp.h>

unsigned char timeDateToSet[15] = {0x55, 0x58, 0x23, 0x05, 0x21, 0x11, 0x19, 0}; // Place holder defualt Date
#define SLAVE_ADDR 0x68 // RTC slave address
volatile int CWcount, CCWcount;



int main(void)
{
    /* Stop Watchdog  */
    MAP_WDT_A_holdTimer();


     // index
    // halting the watch dog is done in the system_msp432p401r.c startup
    ClockInit();// Setting MCLK to 48MHz for faster programming
    ST7735_InitR(INITR_BLACKTAB); // Initiate the LCD
    TimerA_Capture_Init();
    PinInit();

    I2C1_init(); // Initiate the I2C communication for the RTC
    __enable_irq ( ); // enable global interrupts






    while(1)
    {
        DateInput();
    }

}

void PinInit (void)
{
    //Rotary Encoder Button
    P3->SEL0    &=~ BIT3;
    P3->SEL1    &=~ BIT3;
    P3->DIR     &=~ BIT3;
    P3->REN     |=  BIT3;
    P3->OUT     &=~  BIT3; //Input, Pull Down Resistor
}

void DateInput(void){
    int State = 0, DoneFlag = 0, Num, i;
    char Sec[2], Min[2], Hour[2], DoW[15], Month[2], Day[2], Year[2];

    printf("Enter a Date\n");
    while(!DoneFlag)
    {
        switch(State){
        case 0: // Seconds
            if(CWcount) {timeDateToSet[0]++; CWcount = 0;}
            if(CCWcount) {timeDateToSet[0]--; CCWcount = 0;}

            if(timeDateToSet[0]>59)
                timeDateToSet[0] = 0;

            if(RotaryButton())
                State++;

            break;
        case 1: // Minutes
            if(CWcount) {timeDateToSet[0]++; CWcount = 0;}
            if(CCWcount) {timeDateToSet[0]--; CCWcount = 0;}

            if(timeDateToSet[1]>59)
                timeDateToSet[1] = 0;

            if(RotaryButton())
                State++;

            break;
        case 2: // Hours
            if(CWcount) {timeDateToSet[2]++; CWcount = 0;}
            if(CCWcount) {timeDateToSet[2]--; CCWcount = 0;}

            if(timeDateToSet[2]>23)
                timeDateToSet[2] = 0;

            if(RotaryButton())
                State++;

            break;
        case 3: //Day of Week
            if(CWcount) {timeDateToSet[3]++; CWcount = 0;}
            if(CCWcount) {timeDateToSet[3]--; CCWcount = 0;}

            if(timeDateToSet[3]>6)
                timeDateToSet[3] = 0;

            if(RotaryButton())
                State++;

            break;
        case 4: // Month
            if(CWcount) {timeDateToSet[5]++; CWcount = 0;}
            if(CCWcount) {timeDateToSet[5]--; CCWcount = 0;}

            if(timeDateToSet[5]>12)
                timeDateToSet[5] = 0;

            if(RotaryButton())
                State++;

            break;
        case 5: // Day
            if(CWcount) {timeDateToSet[4]++; CWcount = 0;}
            if(CCWcount) {timeDateToSet[4]--; CCWcount = 0;}

            if(timeDateToSet[4]>31)
                timeDateToSet[4] = 0;

            if(RotaryButton())
                State++;

            break;
        case 6: // Year
            if(CWcount) {timeDateToSet[6]++; CWcount = 0;}
            if(CCWcount) {timeDateToSet[6]--; CCWcount = 0;}

            if(timeDateToSet[6]>99)
                timeDateToSet[6] = 0;

            if(RotaryButton())
                DoneFlag =0;

            break;
        }

        sprintf(Sec,"%02d",timeDateToSet[0]);
        sprintf(Min,"%02d",timeDateToSet[1]);
        sprintf(Hour,"%02d",timeDateToSet[2]);


        switch(timeDateToSet[3])
        {
            case 0: strcpy(DoW, "Sunday   ");    break;
            case 1: strcpy(DoW, "Monday   ");     break;
            case 2: strcpy(DoW, "Tuesday  ");    break;
            case 3: strcpy(DoW, "Wednesday");  break;
            case 4: strcpy(DoW, "Thursday ");   break;
            case 5: strcpy(DoW, "Friday   ");     break;
            case 6: strcpy(DoW, "Saturday ");   break;
        }

        sprintf(Month,"%02d",timeDateToSet[5]);
        sprintf(Day,"%02d",timeDateToSet[4]);
        sprintf(Year,"%02d",timeDateToSet[6]);


        ST7735_DrawStringV2(3,6, Sec  ,(State == 0)? 0x001F:0xFFE0,2,2);//Print it to the LCD!
        ST7735_DrawStringV2(7,6, ":"  ,0xFFE0,2,2);//Print it to the LCD!
        ST7735_DrawStringV2(9,6, Min  ,(State == 1)? 0x001F:0xFFE0,2,2);//Print it to the LCD!
        ST7735_DrawStringV2(13,6, ":"  ,0xFFE0);//Print it to the LCD!
        ST7735_DrawStringV2(15,6, Hour ,(State == 2)? 0x001F:0xFFE0,2,2);//Print it to the LCD!

        ST7735_DrawStringV2(3,8, DoW  ,(State == 3)? 0x001F:0xFFE0,2,2);//Print it to the LCD!

        ST7735_DrawStringV2(3,10, Month,(State == 4)? 0x001F:0xFFE0,2,2);//Print it to the LCD!
        ST7735_DrawStringV2(7,10, "/"  ,0xFFE0,2,2);//Print it to the LCD!
        ST7735_DrawStringV2(9,10, Day ,(State == 5)? 0x001F:0xFFE0,2,2);//Print it to the LCD!
        ST7735_DrawStringV2(13,10, "/"  ,0xFFE0,2,2);//Print it to the LCD!
        ST7735_DrawStringV2(15,10, Year,(State == 6)? 0x001F:0xFFE0,2,2);//Print it to the LCD!


    }
}

uint32_t ST7735_DrawStringV2(uint16_t x, uint16_t y, char *pt, int16_t textColor, int16_t size, int16_t space){
  uint32_t count = 0;
  if(y>15) return 0;
  while(*pt){
      ST7735_DrawCharS(x*6, y*10, *pt, textColor, ST7735_BLACK, size);
      //ST7735_DrawCharS(x*6, y*10, *pt, textColor, ST7735_BLACK, 2);
    pt++;
    x = x+space;
    if(x>20) return count;  // number of characters printed
    count++;
  }
  return count;  // number of characters printed
}
////////////////////////////////////////////////////////////
///                        Interrupts                    ///
///////////////////////////////////////////////////////////
void TA2_N_IRQHandler(void) // Timer A2 interrupt Rotary Encoder
{
    __delay_cycles(2000);
    #define DT (P3->IN & BIT3)>>3
    #define Clock (P5->IN & BIT6)>>6

    if(DT == Clock)
        CCWcount++;
    else
        CWcount++;

    //__delay_cycles(20);
    SysTick_delay(10);

    TIMER_A2->CCTL[1] &= ~(TIMER_A_CCTLN_CCIFG); // Clear the interrupt flag


}









