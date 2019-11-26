//i h8 git
//fdnkoasfnsyujfhj
/* DriverLib Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <Math.h>
#include "driverlib.h"

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>
#include "ST7735.h"
#include "Functions.h"

#include <msp.h>

#define Radius 6
unsigned char timeDateToSet[15] = {55, 58, 23, 05, 21, 11, 19, 0}; // Place holder default Date
unsigned char timeDateReadback[7];
unsigned char TempReadback[2];

volatile char Sec[2], Min[2], Hour[2], DoW[15], Month[2], Day[2], Year[2];
volatile char SecOld[2], MinOld[2], HourOld[2], DoWOld[15], MonthOld[2], DayOld[2], YearOld[2];

volatile int8_t Alarm1[4] = {4,4,4,4}, Alarm2[4] = {4,4,4,4};

#define SLAVE_ADDR 0x68 // RTC slave address
#define CALIBRATION_START 0x000200054 // Starting memory address in flash
volatile int8_t CWcount, CCWcount;

volatile float Speed=0;
volatile float SpeedAvg, SpeedCount;
volatile char SpeedS[5], SpeedSOld[5];

volatile float Temp= 70;
volatile char TempS[5], TempSOld[5];

int8_t MeasureScreenCount = 1;
int8_t Reset = 1;

int main(void)
{
    /* Stop Watchdog  */
    MAP_WDT_A_holdTimer();

    P9->SEL0 &= ~ (BIT4|BIT6);
    P9->SEL1 &= ~ (BIT4|BIT6);                      // configure P9.2 (D/C), P9.3 (Reset), and P9.4 (TFT_CS) as GPIO
    P9->DIR |=    (BIT4|BIT6);                        // make P9.2 (D/C), P9.3 (Reset), and P9.4 (TFT_CS) out
//    P9->OUT |=  (BIT4|BIT6);                      // configure P9.2 (D/C), P9.3 (Reset), and P9.4 (TFT_CS) as GPIO

//    P10->SEL0 &= ~ (BIT4);
//    P10->SEL1 &= ~ (BIT4);                      // configure P9.2 (D/C), P9.3 (Reset), and P9.4 (TFT_CS) as GPIO
//    P10->DIR |=    (BIT4);
//    P10->OUT &= ~     (BIT4);
    // index
    // halting the watch dog is done in the system_msp432p401r.c startup

    ClockInit();// Setting MCLK to 48MHz for faster programming
    ST7735_InitR(INITR_BLACKTAB); // Initiate the LCD
    TimerA_Capture_Init();
    SpeedInit();
    PinInit();
    adcsetup();
    WatchdogInit();
    I2C1_init(); // Initiate the I2C communication for the RTC


    TIMER32_1->CONTROL = 0b11100111;
    TIMER32_1->LOAD = 3000000;

    NVIC_EnableIRQ(PORT3_IRQn);
    NVIC_SetPriority(PORT3_IRQn, 1);

    NVIC_EnableIRQ(PORT1_IRQn);
    NVIC_SetPriority(PORT1_IRQn, 10);

    NVIC_EnableIRQ (TA1_N_IRQn); // enable capture interrupt
    NVIC_SetPriority(TA1_N_IRQn, 30);

    NVIC_EnableIRQ( T32_INT1_IRQn );
    NVIC_SetPriority(T32_INT1_IRQn, 20);

    __enable_irq ( ); // enable global interrupts




    I2C1_burstRead(SLAVE_ADDR, 0, 7, timeDateReadback);
    P3->IE      |=  BIT7;

    while(1)
    {
        //Alarm1Config();
        //MeasurmentDisplay2();
       MainMenu();
        //AlarmHist();
    }

}


void MainMenu(void)
{
    static int8_t State = 0, Flag = 1;

    if(CWcount)     {State++; (State>2)?State=0:0; Flag = 1; CWcount =  0;}
    if(CCWcount)    {State--; (State<0)?State=2:0; Flag = 1; CCWcount = 0;}

        switch(State)
        {
        case 0:// Time Set
            if(RotaryButton())
            {
                Output_Clear();
                DateInput();
                Flag =1;
            }
            break;
        case 1: // Alarm History
            if(RotaryButton())
            {
                Output_Clear();
                AlarmHist();
                Flag =1;
            }
            break;

        case 2: //Alarm Config
            if(RotaryButton())
            {
                Output_Clear();
                AlarmConfigMenu();
                Flag =1;
            }
            break;
        }

        if(Flag)
        {
            LCDSelect = 1;
            ST7735_DrawStringV2(3,4, "Time Set" ,(State == 0)? 0x001F:0xFFE0,2,2);//Print it to the LCD!
            circleBres(10, 46, 5,(State == 0)? 0x001F:0x0000);
            //ST7735_DrawStringV2(3,6, "Set" ,0xFFE0,2,2);//Print it to the LCD!

            ST7735_DrawStringV2(3,7, "Alarm " ,(State == 1)? 0x001F:0xFFE0,2,2);//Print it to the LCD!
            ST7735_DrawStringV2(3,9, "History" ,(State == 1)? 0x001F:0xFFE0,2,2);//Print it to the LCD!
            circleBres(10, 90, 5,(State == 1)? 0x001F:0x0000);

            ST7735_DrawStringV2(3,12, "Alarm" ,(State == 2)? 0x001F:0xFFE0,2,2);//Print it to the LCD!
            ST7735_DrawStringV2(3,14, "Config" ,(State == 2)? 0x001F:0xFFE0,2,2);//Print it to the LCD!
            circleBres(10, 135, 5,(State == 2)? 0x001F:0x0000);
            Flag = 0;
        }


}

void MeasurmentDisplay1(void)
{
    LCDSelect = 2;

    if(Reset)
    {
        Output_Clear();
        ST7735_FillRect(64, 110, 2, 70, 0xFFE0);
        ST7735_FillRect(0, 110, 128, 2, 0xFFE0);
        ST7735_DrawStringV2(7,4, ":"  ,0xFFE0,2,2);//Print it to the LCD!
        ST7735_DrawStringV2(13,4, ":"  ,0xFFE0,2,2);//Print it to the LCD!
        ST7735_DrawStringV2(7,8, "/"  ,0xFFE0,2,2);//Print it to the LCD!
        ST7735_DrawStringV2(13,8, "/"  ,0xFFE0,2,2);//Print it to the LCD!
        ST7735_DrawStringV2(2,14, "MPH" ,0xFFE0,2,2);
        ST7735_DrawStringV2(15,14, "o" ,0xFFE0,1,1);
        ST7735_DrawStringV2(16,14, "F" ,0xFFE0,2,2);

    }


    //Time
    if(strcmp(HourOld, Hour) || Reset)
        ST7735_DrawStringV2(3,4, Hour ,0xFFE0,2,2);//Print it to the LCD!
    if(strcmp(MinOld, Min) || Reset)
        ST7735_DrawStringV2(9,4, Min  ,0xFFE0,2,2);//Print it to the LCD!
    if(strcmp(SecOld, Sec) || Reset)
        ST7735_DrawStringV2(15,4, Sec  ,0xFFE0,2,2);//Print it to the LCD!
    if(strcmp(DoWOld, DoW) || Reset)
        ST7735_DrawStringV2(3,6, DoW  ,0xFFE0,2,2);//Print it to the LCD!
    if(strcmp(MonthOld, Month) || Reset)
        ST7735_DrawStringV2(3,8, Month,0xFFE0,2,2);//Print it to the LCD!
    if(strcmp(DayOld, Day) || Reset)
        ST7735_DrawStringV2(9,8, Day ,0xFFE0,2,2);//Print it to the LCD!
    if(strcmp(YearOld, Year) || Reset)
        ST7735_DrawStringV2(15,8, Year,0xFFE0,2,2);//Print it to the LCD!

    //Speed
    //Speed++;
    sprintf(SpeedS,"%03.0f\0", Speed);
    if(strcmp(SpeedSOld, SpeedS) || Reset)
        ST7735_DrawStringV2(2,12, SpeedS ,0xFFE0,2,2);


    //Temp
    sprintf(TempS,"%03.0f\0", Temp);
    if(strcmp(TempSOld, TempS)|| Reset)
        ST7735_DrawStringV2(13,12, TempS ,0xFFE0,2,2);


    LCDSelect = 1;
    Reset = 0;

}

void MeasurmentDisplay2(void)
{
    LCDSelect = 2;

    if(Reset)
    {
        Output_Clear();
        ST7735_FillRect(63, 110, 2, 70, 0xFFE0);
        ST7735_FillRect(0, 110, 128, 2, 0xFFE0);
        ST7735_DrawStringV2(15,12, ":"  ,0xFFE0,2,2);//Print it to the LCD!
        ST7735_DrawStringV2(13,14, ":"  ,0xFFE0,2,2);//Print it to the LCD!
        ST7735_DrawStringV2(6,8, "MPH" ,0xFFE0,3,3);
        ST7735_DrawStringV2(4,14, "o" ,0xFFE0,1,1);
        ST7735_DrawStringV2(5,14, "F" ,0xFFE0,2,2);


    }


    //Time
    if(strcmp(HourOld, Hour) || Reset)
        ST7735_DrawStringV2(11,12, Hour ,0xFFE0,2,2);//Print it to the LCD!
    if(strcmp(MinOld, Min) || Reset)
        ST7735_DrawStringV2(17,12, Min  ,0xFFE0,2,2);//Print it to the LCD!
    if(strcmp(SecOld, Sec) || Reset)
        ST7735_DrawStringV2(15,14, Sec  ,0xFFE0,2,2);//Print it to the LCD!


    //Speed
    //Speed++;
    sprintf(SpeedS,"%03.1f\0", Speed);
    if(strcmp(SpeedSOld, SpeedS) || Reset)
        ST7735_DrawStringV2(6,5, SpeedS ,0xFFE0,3,3);


    //Temp
    sprintf(TempS,"%03.0f\0", Temp);
    if(strcmp(TempSOld, TempS)|| Reset)
        ST7735_DrawStringV2(2,12, TempS ,0xFFE0,2,2);



    LCDSelect = 1;
    Reset = 0;
}

void MeasurmentDisplay3(void)
{
    LCDSelect = 2;

    if(Reset)
    {
        Output_Clear();
        ST7735_FillRect(64, 110, 2, 70, 0xFFE0);
        ST7735_FillRect(0, 110, 128, 2, 0xFFE0);
        ST7735_DrawStringV2(7,4, ":"  ,0xFFE0,2,2);//Print it to the LCD!
        ST7735_DrawStringV2(13,4, ":"  ,0xFFE0,2,2);//Print it to the LCD!
        ST7735_DrawStringV2(7,8, "/"  ,0xFFE0,2,2);//Print it to the LCD!
        ST7735_DrawStringV2(13,8, "/"  ,0xFFE0,2,2);//Print it to the LCD!
        ST7735_DrawStringV2(2,14, "MPH" ,0xFFE0,2,2);
        ST7735_DrawStringV2(15,14, 167 ,0xFFE0,2,2);
        ST7735_DrawStringV2(16,14, "3" ,0xFFE0,2,2);


    }


    //Time
    if(strcmp(HourOld, Hour) || Reset)
        ST7735_DrawStringV2(3,4, Hour ,0xFFE0,2,2);//Print it to the LCD!
    if(strcmp(MinOld, Min) || Reset)
        ST7735_DrawStringV2(9,4, Min  ,0xFFE0,2,2);//Print it to the LCD!
    if(strcmp(SecOld, Sec) || Reset)
        ST7735_DrawStringV2(15,4, Sec  ,0xFFE0,2,2);//Print it to the LCD!
    if(strcmp(DoWOld, DoW) || Reset)
        ST7735_DrawStringV2(3,6, DoW  ,0xFFE0,2,2);//Print it to the LCD!
    if(strcmp(MonthOld, Month) || Reset)
        ST7735_DrawStringV2(3,8, Month,0xFFE0,2,2);//Print it to the LCD!
    if(strcmp(DayOld, Day) || Reset)
        ST7735_DrawStringV2(9,8, Day ,0xFFE0,2,2);//Print it to the LCD!
    if(strcmp(YearOld, Year) || Reset)
        ST7735_DrawStringV2(15,8, Year,0xFFE0,2,2);//Print it to the LCD!

    //Speed
    //Speed++;
    sprintf(SpeedS,"%03.0f\0", Speed);
    if(strcmp(SpeedSOld, SpeedS) || Reset)
        ST7735_DrawStringV2(2,12, SpeedS ,0xFFE0,2,2);


    //Temp
    sprintf(TempS,"%03.0f\0", Temp);
    if(strcmp(TempSOld, TempS)|| Reset)
        ST7735_DrawStringV2(13,12, TempS ,0xFFE0,2,2);

    strcpy(SecOld, Sec); strcpy(HourOld, Hour); strcpy(MinOld, Min); strcpy(DoWOld, DoW); strcpy(MonthOld, Month); strcpy(DayOld, Day); strcpy(YearOld, Year);
    strcpy(SpeedSOld, SpeedS);
    strcpy(TempSOld, TempS);

    LCDSelect = 1;
    Reset = 0;
}

void DateInput(void){
    int8_t State = 0, DoneFlag = 0, Flag = 1;
    Output_Clear();
    //ST7735_FillScreen(0xFFFF);

    while(!DoneFlag)
    {
        switch(State){
        case 0: // Seconds
            if(CWcount) {timeDateToSet[0]++; Flag = 1; CWcount = 0;}
            if(CCWcount) {timeDateToSet[0]--;  Flag = 1; CCWcount = 0;}

            if(timeDateToSet[0]>59)
                timeDateToSet[0] = 0;

            if(RotaryButton()){ State++; Flag = 1;}

            break;
        case 1: // Minutes
            if(CWcount) {timeDateToSet[1]++;  Flag = 1; CWcount = 0;}
            if(CCWcount) {timeDateToSet[1]--; Flag = 1; CCWcount = 0;}

            if(timeDateToSet[1]>59)
                timeDateToSet[1] = 0;

            if(RotaryButton()){ State++; Flag = 1;}


            break;
        case 2: // Hours
            if(CWcount) {timeDateToSet[2]++; Flag = 1; CWcount = 0;}
            if(CCWcount) {timeDateToSet[2]--;  Flag = 1;CCWcount = 0;}

            if(timeDateToSet[2]>23)
                timeDateToSet[2] = 0;

            if(RotaryButton()){ State++; Flag = 1;}

            break;
        case 3: //Day of Week
            if(CWcount) {timeDateToSet[3]++; Flag = 1; CWcount = 0;}
            if(CCWcount) {timeDateToSet[3]--; Flag = 1;  CCWcount = 0;}

            if(timeDateToSet[3]>6)
                timeDateToSet[3] = 0;

            if(RotaryButton()){ State++; Flag = 1;}

            break;
        case 4: // Month
            if(CWcount) {timeDateToSet[5]++; Flag = 1; CWcount = 0;}
            if(CCWcount) {timeDateToSet[5]--; Flag = 1; CCWcount = 0;}

            if(timeDateToSet[5]>12)
                timeDateToSet[5] = 0;

            if(RotaryButton()){ State++; Flag = 1;}

            break;
        case 5: // Day
            if(CWcount) {timeDateToSet[4]++; Flag = 1; CWcount = 0;}
            if(CCWcount) {timeDateToSet[4]--; Flag = 1; CCWcount = 0;}

            if(timeDateToSet[4]>31)
                timeDateToSet[4] = 0;

            if(RotaryButton()){ State++; Flag = 1;}

            break;
        case 6: // Year
            if(CWcount) {timeDateToSet[6]++; Flag = 1; CWcount = 0;}
            if(CCWcount) {timeDateToSet[6]--; Flag = 1;  CCWcount = 0;}

            if(timeDateToSet[6]>99)
                timeDateToSet[6] = 0;

            if(RotaryButton())
            {
                DoneFlag =1;
                Output_Clear();
            }


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

        if(Flag)
        {
            LCDSelect = 1;
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
            Flag =0;
        }


        //SysTick_delay(1);

    }

    int8_t i;
    unsigned char timeDateToSetHEX[15]; // Place holder default Date
    for (i=0; i<15; i++)
        timeDateToSetHEX[i]=((timeDateToSet[i]/10)*16)+(timeDateToSet[i]%10);
    I2C1_burstWrite(SLAVE_ADDR, 0, 7, timeDateToSetHEX);
}

void AlarmConfigMenu(void)
{
    int8_t DoneFlag = 0, State = 0, Flag = 1;
    Output_Clear();
    while(!DoneFlag)
    {
        if(CWcount)     {State++; (State>2)?State=0:0; Flag = 1; CWcount =  0;}
        if(CCWcount)    {State--; (State<0)?State=2:0; Flag = 1; CCWcount = 0;}

            switch(State)
            {
            case 0:// Alarm 1 Config
                if(RotaryButton())
                {
                    Output_Clear();
                    Alarm1Config();
                    Flag =1;
                }
                break;
            case 1: // Alarm 2 Config
                if(RotaryButton())
                {
                    Output_Clear();
                    Alarm2Config();
                    Flag =1;
                }
                break;

            case 2: //Done
                if(RotaryButton())
                {
                    Output_Clear();
                    DoneFlag = 1;
                }
                break;
            }

            if(Flag)
            {
                LCDSelect = 1;
                ST7735_DrawStringV2(3,4, "Alarm 1" ,(State == 0)? 0x001F:0xFFE0,2,2);//Print it to the LCD!
                circleBres(10, 46, 5,(State == 0)? 0x001F:0x0000);
                //ST7735_DrawStringV2(3,6, "Set" ,0xFFE0,2,2);//Print it to the LCD!

                ST7735_DrawStringV2(3,7, "Alarm 2  " ,(State == 1)? 0x001F:0xFFE0,2,2);//Print it to the LCD!
                circleBres(10, 75, 5,(State == 1)? 0x001F:0x0000);

                ST7735_DrawStringV2(3,12, "Done" ,(State == 2)? 0x001F:0xFFE0,2,2);//Print it to the LCD!
                circleBres(10, 125, 5,(State == 2)? 0x001F:0x0000);
                Flag = 0;
            }

    }
}

void Alarm1Config(void)
{
    int8_t DoneFlag = 0, State = 0, Flag = 1;
    Output_Clear();
    while(!DoneFlag)
    {
            switch(State)
            {
                case 0: // First Note
                    if(CWcount) {Alarm1[State]++; Flag = 1; CWcount = 0;}
                    if(CCWcount) {Alarm1[State]--;  Flag = 1; CCWcount = 0;}

                    if(Alarm1[State]>8) Alarm1[State] = 8;
                    if(Alarm1[State]<0) Alarm1[State] = 0;

                    if(RotaryButton()){ State++; Flag = 1;}

                    break;
                case 1: // Second
                    if(CWcount) {Alarm1[State]++; Flag = 1; CWcount = 0;}
                    if(CCWcount) {Alarm1[State]--;  Flag = 1; CCWcount = 0;}

                    if(Alarm1[State]>8) Alarm1[State] = 8;
                    if(Alarm1[State]<0) Alarm1[State] = 0;

                    if(RotaryButton()){ State++; Flag = 1;}


                    break;
                case 2: // Third
                    if(CWcount) {Alarm1[State]++; Flag = 1; CWcount = 0;}
                    if(CCWcount) {Alarm1[State]--;  Flag = 1; CCWcount = 0;}

                    if(Alarm1[State]>8) Alarm1[State] = 8;
                    if(Alarm1[State]<0) Alarm1[State] = 0;

                    if(RotaryButton()){ State++; Flag = 1;}

                    break;
                case 3: //Fourth
                    if(CWcount) {Alarm1[State]++; Flag = 1; CWcount = 0;}
                    if(CCWcount) {Alarm1[State]--;  Flag = 1; CCWcount = 0;}

                    if(Alarm1[State]>8) Alarm1[State] = 8;
                    if(Alarm1[State]<0) Alarm1[State] = 0;

                    if(RotaryButton()){ DoneFlag=1; Flag = 1;}

                    break;
            }



            if(Flag)
            {
                LCDSelect = 1;
                ST7735_DrawStringV2(3,14, "1" ,(State == 0)? 0x001F:0xFFE0,2,2);//Print it to the LCD!
                ST7735_FillRect(21, 55, 4, 80, 0);
                ST7735_FillRect(21, (135-Alarm1[0]*10), 4, Alarm1[0]*10, (State == 0)? 0x001F:0xFFE0);

                ST7735_DrawStringV2(8,14, "2" ,(State == 1)? 0x001F:0xFFE0,2,2);//Print it to the LCD!
                ST7735_FillRect(50, 55, 4, 80, 0);
                ST7735_FillRect(50, (135-Alarm1[1]*10), 4, Alarm1[1]*10, (State == 1)? 0x001F:0xFFE0);

                ST7735_DrawStringV2(12,14, "3" ,(State == 2)? 0x001F:0xFFE0,2,2);//Print it to the LCD!
                ST7735_FillRect(75, 55, 4, 80, 0);
                ST7735_FillRect(75, (135-Alarm1[2]*10), 4, Alarm1[2]*10, (State == 2)? 0x001F:0xFFE0);

                ST7735_DrawStringV2(16,14, "4" ,(State == 3)? 0x001F:0xFFE0,2,2);//Print it to the LCD!
                ST7735_FillRect(100, 55, 4, 80, 0);
                ST7735_FillRect(100, (135-Alarm1[3]*10), 4, Alarm1[3]*10, (State == 3)? 0x001F:0xFFE0);

                Flag = 0;
            }

    }
    Output_Clear();
}

void Alarm2Config(void)
{
    int8_t DoneFlag = 0, State = 0, Flag = 1;
    Output_Clear();
    while(!DoneFlag)
    {
            switch(State)
            {
                case 0: // First Note
                    if(CWcount) {Alarm2[State]++; Flag = 1; CWcount = 0;}
                    if(CCWcount) {Alarm2[State]--;  Flag = 1; CCWcount = 0;}

                    if(Alarm2[State]>8) Alarm2[State] = 8;
                    if(Alarm2[State]<0) Alarm2[State] = 0;

                    if(RotaryButton()){ State++; Flag = 1;}

                    break;
                case 1: // Second
                    if(CWcount) {Alarm2[State]++; Flag = 1; CWcount = 0;}
                    if(CCWcount) {Alarm2[State]--;  Flag = 1; CCWcount = 0;}

                    if(Alarm2[State]>8) Alarm2[State] = 8;
                    if(Alarm2[State]<0) Alarm2[State] = 0;

                    if(RotaryButton()){ State++; Flag = 1;}


                    break;
                case 2: // Third
                    if(CWcount) {Alarm2[State]++; Flag = 1; CWcount = 0;}
                    if(CCWcount) {Alarm2[State]--;  Flag = 1; CCWcount = 0;}

                    if(Alarm2[State]>8) Alarm2[State] = 8;
                    if(Alarm2[State]<0) Alarm2[State] = 0;

                    if(RotaryButton()){ State++; Flag = 1;}

                    break;
                case 3: //Fourth
                    if(CWcount) {Alarm2[State]++; Flag = 1; CWcount = 0;}
                    if(CCWcount) {Alarm2[State]--;  Flag = 1; CCWcount = 0;}

                    if(Alarm2[State]>8) Alarm2[State] = 8;
                    if(Alarm2[State]<0) Alarm2[State] = 0;

                    if(RotaryButton()){ DoneFlag=1; Flag = 1; }

                    break;
            }



            if(Flag)
            {
                LCDSelect = 1;
                ST7735_DrawStringV2(3,14, "1" ,(State == 0)? 0x001F:0xFFE0,2,2);//Print it to the LCD!
                ST7735_FillRect(21, 55, 4, 80, 0);
                ST7735_FillRect(21, (135-Alarm2[0]*10), 4, Alarm2[0]*10, (State == 0)? 0x001F:0xFFE0);

                ST7735_DrawStringV2(8,14, "2" ,(State == 1)? 0x001F:0xFFE0,2,2);//Print it to the LCD!
                ST7735_FillRect(50, 55, 4, 80, 0);
                ST7735_FillRect(50, (135-Alarm2[1]*10), 4, Alarm2[1]*10, (State == 1)? 0x001F:0xFFE0);

                ST7735_DrawStringV2(12,14, "3" ,(State == 2)? 0x001F:0xFFE0,2,2);//Print it to the LCD!
                ST7735_FillRect(75, 55, 4, 80, 0);
                ST7735_FillRect(75, (135-Alarm2[2]*10), 4, Alarm2[2]*10, (State == 2)? 0x001F:0xFFE0);

                ST7735_DrawStringV2(16,14, "4" ,(State == 3)? 0x001F:0xFFE0,2,2);//Print it to the LCD!
                ST7735_FillRect(100, 55, 4, 80, 0);
                ST7735_FillRect(100, (135-Alarm2[3]*10), 4, Alarm2[3]*10, (State == 3)? 0x001F:0xFFE0);

                Flag = 0;
            }

    }
    Output_Clear();
}

void AlarmHist(void)
{

    uint8_t i,j;
    uint8_t* addr_pointer;
    unsigned char Alarms[8][5] =0;
    char TimeString[30] = 0;


    for(j=0;j<5;j++)
    {
        TimeString[0] = 0;
        for(i=0;i<8;i++)
        {
            addr_pointer = (CALIBRATION_START +(0x14 * (j))+i);
            Alarms[j][i]= *addr_pointer;
        }
        sprintf(TimeString, "%x:%x:%x %x/%x/%x\0",
                Alarms[j][2],
                Alarms[j][1],
                Alarms[j][0],
                Alarms[j][5],
                Alarms[j][4],
                Alarms[j][6]);
        ST7735_DrawStringV2(1,(4+j*2),TimeString,0xFFE0,1,1);

        sprintf(TimeString,"%s\0",(Alarms[j][7]=='S')?"Speed":"Temp");
        ST7735_DrawStringV2(3,(4+j*2+1),TimeString,0xFFE0,1,1);
    }

    ST7735_DrawStringV2(3,14, "Done" ,0x001F,2,2);//Print it to the LCD!
    circleBres(10, 147, 5,0x001F);
    int8_t DoneFlag = 0;
    while(!DoneFlag)
    {
        //SysTick_delay(1000);
        if(RotaryButton())
        {
            DoneFlag=1;
            Output_Clear();
        }


    }
}

////////////////////////////////////////////////////////////
///                        Interrupts                    ///
///////////////////////////////////////////////////////////

void TA1_N_IRQHandler(void) // Timer A2 interrupt service routine
{
    SpeedAvg += TIMER_A1 -> CCR[2];//OnFlag holds the data for the hall sensor
    TIMER_A1->CTL |= TIMER_A_CTL_CLR;

    SpeedCount++;//OnFlag continuously reads in, holding the values of j until
    TIMER_A1->CCTL[2] &= ~(TIMER_A_CCTLN_CCIFG); // Clear the interrupt flag
}

void TA2_N_IRQHandler(void) // Timer A2 interrupt Rotary Encoder
{
    SysTick_delay(1);
    //__delay_cycles(200);
    int8_t DT = (P5->IN & BIT7)>>7 ;
    int8_t Clock = (P3->IN & BIT0)>>0;

    if(DT == Clock)
        CCWcount++;
    else
        CWcount++;

//    __delay_cycles(20);
    SysTick_delay(10);

    TIMER_A2->CCTL[2] &= ~(TIMER_A_CCTLN_CCIFG); // Clear the interrupt flag
}

void T32_INT1_IRQHandler (void)                             //Interrupt Handler for Timer32 1.
{
    TIMER32_1->INTCLR = 1;  //Clear interrupt flag so it does not interrupt again immediately.
    WatchdogInit();

    ADC14->CTL0 |= 1; //start a conversion
    while(!ADC14->IFGR0); // wait until conversion complete
    int result = ADC14->MEM[5]; // immediately store value in a variable
    //result = 6001;
    //(result>=6000)?BacklightPWM(10, 1):
    (result>=5000)?BacklightPWM(10, .8):
    (result>=4000 && result<5000)?BacklightPWM(10, .7):
    (result>=3000 && result<4000)?BacklightPWM(10, .6):
    (result>=2000 && result<3000)?BacklightPWM(10, .5):
    (result>=1000 && result<2000)?BacklightPWM(10, .4):
    (result<1000)?BacklightPWM(10, 1):0;

    SpeedAvg/=SpeedCount;
    SpeedAvg/=32768;
    SpeedAvg*=2;
    SpeedAvg= 1/SpeedAvg;
    SpeedAvg*=60;
    //SpeedAvg = (1/(2/(SpeedCount*32768)))*60;//RPM
    Speed = ((2*3.14152*Radius)*SpeedAvg*60)/5280;//MPH
    (SpeedCount<3)?Speed=0:0;

    SpeedAvg = 0;
    SpeedCount = 0;

    I2C1_burstRead(SLAVE_ADDR, 0, 7, timeDateReadback);

    sprintf(Sec,"%02x",timeDateReadback[0]);
    sprintf(Min,"%02x",timeDateReadback[1]);
    sprintf(Hour,"%02x",timeDateReadback[2]);
    switch(timeDateReadback[3])
    {
        case 0: strcpy(DoW, "Sunday   ");    break;
        case 1: strcpy(DoW, "Monday   ");     break;
        case 2: strcpy(DoW, "Tuesday  ");    break;
        case 3: strcpy(DoW, "Wednesday");  break;
        case 4: strcpy(DoW, "Thursday ");   break;
        case 5: strcpy(DoW, "Friday   ");     break;
        case 6: strcpy(DoW, "Saturday ");   break;
    }
    sprintf(Month,"%02x",timeDateReadback[5]);
    sprintf(Day,"%02x",timeDateReadback[4]);
    sprintf(Year,"%02x",timeDateReadback[6]);

    I2C1_burstRead(SLAVE_ADDR, 0x11, 2, TempReadback);
    Temp=(TempReadback[0]+((TempReadback[1]>>6)+1)/4)*1.8+32;

    (MeasureScreenCount==0)? MeasurmentDisplay1():
    (MeasureScreenCount==1)? MeasurmentDisplay2():
    (MeasureScreenCount==2)? MeasurmentDisplay3():0;

    strcpy(SecOld, Sec); strcpy(HourOld, Hour); strcpy(MinOld, Min); strcpy(DoWOld, DoW); strcpy(MonthOld, Month); strcpy(DayOld, Day); strcpy(YearOld, Year);
    strcpy(SpeedSOld, SpeedS);
    strcpy(TempSOld, TempS);

    TIMER32_1->LOAD = 3000000;
}

void PORT1_IRQHandler(void)
{
    //DebounceSwitch1();
    MeasureScreenCount++;
    (MeasureScreenCount>2)?MeasureScreenCount = 0:0;
    Reset = 1;
    P1->IFG = 0; //Clear all flags
    TIMER32_1->LOAD = 10;

}

void PORT3_IRQHandler(void)
{
    MemShift(timeDateReadback, 'S');
    P3->IFG = 0; //Clear all flags
    //while(1);

}







