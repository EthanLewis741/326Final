//i h8 git
//fdnkoasfnsyujfhj
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

unsigned char timeDateToSet[15] = {55, 58, 23, 05, 21, 11, 19, 0}; // Place holder defualt Date

volatile char Sec[2], Min[2], Hour[2], DoW[15], Month[2], Day[2], Year[2];
volatile char SecOld[2], MinOld[2], HourOld[2], DoWOld[15], MonthOld[2], DayOld[2], YearOld[2];

#define SLAVE_ADDR 0x68 // RTC slave addressb
volatile int CWcount, CCWcount;

volatile int Speed= 70;
volatile char SpeedS[3], SpeedSOld[3];

volatile int Temp= 70;
volatile char TempS[3], TempSOld[3];

int main(void)
{
    /* Stop Watchdog  */
    MAP_WDT_A_holdTimer();

    P9->SEL0 &= ~ (BIT4|BIT6);
    P9->SEL1 &= ~ (BIT4|BIT6);                      // configure P9.2 (D/C), P9.3 (Reset), and P9.4 (TFT_CS) as GPIO
    P9->DIR |=    (BIT4|BIT6);                        // make P9.2 (D/C), P9.3 (Reset), and P9.4 (TFT_CS) out
//    P9->OUT |=  (BIT4|BIT6);                      // configure P9.2 (D/C), P9.3 (Reset), and P9.4 (TFT_CS) as GPIO

     // index
    // halting the watch dog is done in the system_msp432p401r.c startup
    ClockInit();// Setting MCLK to 48MHz for faster programming
    ST7735_InitR(INITR_BLACKTAB); // Initiate the LCD
    TimerA_Capture_Init();
    PinInit();


    I2C1_init(); // Initiate the I2C communication for the RTC

    TIMER32_1->CONTROL = 0b11100111;
    //TIMER32_1->LOAD = 3000000;
    NVIC_EnableIRQ( T32_INT1_IRQn );

    __enable_irq ( ); // enable global interrupts




    //LCDSel(2);
    //ST7735_DrawStringV2(7,10, "Test"  ,0xFFE0,2,2);
    //SysTick_delay(1);

    while(1)
    {

        //LCDSel(1);
        DateInput();
    }

}




void MeasurmentDisplay1(void)
{
    LCDSel(2);

    static int Init = 1;
    if(Init)
    {
        ST7735_FillRect(64, 110, 2, 70, 0xFFE0);
        ST7735_FillRect(0, 110, 128, 2, 0xFFE0);
        ST7735_DrawStringV2(7,4, ":"  ,0xFFE0,2,2);//Print it to the LCD!
        ST7735_DrawStringV2(13,4, ":"  ,0xFFE0,2,2);//Print it to the LCD!
        ST7735_DrawStringV2(7,8, "/"  ,0xFFE0,2,2);//Print it to the LCD!
        ST7735_DrawStringV2(13,8, "/"  ,0xFFE0,2,2);//Print it to the LCD!
        ST7735_DrawStringV2(2,14, "MPH" ,0xFFE0,2,2);
        ST7735_DrawStringV2(15,14, 167 ,0xFFE0,2,2);
        ST7735_DrawStringV2(16,14, "F" ,0xFFE0,2,2);
        Init = 0;

    }


    //Time
    if(strcmp(HourOld, Hour))
        ST7735_DrawStringV2(3,4, Hour ,0xFFE0,2,2);//Print it to the LCD!
    if(strcmp(MinOld, Min))
        ST7735_DrawStringV2(9,4, Min  ,0xFFE0,2,2);//Print it to the LCD!
    if(strcmp(SecOld, Sec))
        ST7735_DrawStringV2(15,4, Sec  ,0xFFE0,2,2);//Print it to the LCD!
    if(strcmp(DoWOld, DoW))
        ST7735_DrawStringV2(3,6, DoW  ,0xFFE0,2,2);//Print it to the LCD!
    if(strcmp(MonthOld, Month))
        ST7735_DrawStringV2(3,8, Month,0xFFE0,2,2);//Print it to the LCD!
    if(strcmp(DayOld, Day))
        ST7735_DrawStringV2(9,8, Day ,0xFFE0,2,2);//Print it to the LCD!
    if(strcmp(YearOld, Year))
        ST7735_DrawStringV2(15,8, Year,0xFFE0,2,2);//Print it to the LCD!

    //Speed
    Speed++;
    sprintf(SpeedS,"%03d", Speed);
    if(strcmp(SpeedSOld, SpeedS))
        ST7735_DrawStringV2(2,12, SpeedS ,0xFFE0,2,2);


    //Temp
    sprintf(TempS,"%03d", Temp);
    if(strcmp(TempSOld, Temp))
        ST7735_DrawStringV2(15,12, TempS ,0xFFE0,2,2);

    strcpy(SecOld, Sec); strcpy(HourOld, Hour); strcpy(MinOld, Min); strcpy(DoWOld, DoW); strcpy(MonthOld, Month); strcpy(DayOld, Day); strcpy(YearOld, Year);
    strcpy(SpeedSOld, SpeedS);
    strcpy(TempSOld, TempS);

    LCDSel(1);
}

void DateInput(void){
    int State = 0, DoneFlag = 0, Num, i;

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

        LCDSel(1);
        ST7735_DrawStringV2(15,6, Sec  ,(State == 0)? 0x001F:0xFFE0,2,2);//Print it to the LCD!
        ST7735_DrawStringV2(7,6, ":"  ,0xFFE0,2,2);//Print it to the LCD!
        ST7735_DrawStringV2(9,6, Min  ,(State == 1)? 0x001F:0xFFE0,2,2);//Print it to the LCD!
        ST7735_DrawStringV2(13,6, ":"  ,0xFFE0,2,2);//Print it to the LCD!
        ST7735_DrawStringV2(3,6, Hour ,(State == 2)? 0x001F:0xFFE0,2,2);//Print it to the LCD!

        ST7735_DrawStringV2(3,8, DoW  ,(State == 3)? 0x001F:0xFFE0,2,2);//Print it to the LCD!

        ST7735_DrawStringV2(3,10, Month,(State == 4)? 0x001F:0xFFE0,2,2);//Print it to the LCD!
        ST7735_DrawStringV2(7,10, "/"  ,0xFFE0,2,2);//Print it to the LCD!
        ST7735_DrawStringV2(9,10, Day ,(State == 5)? 0x001F:0xFFE0,2,2);//Print it to the LCD!
        ST7735_DrawStringV2(13,10, "/"  ,0xFFE0,2,2);//Print it to the LCD!
        ST7735_DrawStringV2(15,10, Year,(State == 6)? 0x001F:0xFFE0,2,2);//Print it to the LCD!
        //SysTick_delay(1);
        //LCDSel(0);

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
    #define DT (P6->IN & BIT6)>>6
    #define Clock (P5->IN & BIT6)>>6

    if(DT == Clock)
        CCWcount++;
    else
        CWcount++;

//    __delay_cycles(20);
    SysTick_delay(10);

    TIMER_A2->CCTL[1] &= ~(TIMER_A_CCTLN_CCIFG); // Clear the interrupt flag
}

void T32_INT1_IRQHandler ( )                             //Interrupt Handler for Timer32 1.
{
    TIMER32_1->INTCLR = 1;  //Clear interrupt flag so it does not interrupt again immediately.
    MeasurmentDisplay1();
    TIMER32_1->LOAD = 3000000;
}








