<<<<<<< HEAD
Skip to content
Search or jump to…

Pull requests
Issues
Marketplace
Explore

@EthanLewis741
Learn Git and GitHub without any code!
Using the Hello World guide, you’ll start a branch, write comments, and open a pull request.


1
00EthanLewis741/326Final
 Code Issues 0 Pull requests 0 Projects 0 Wiki Security Insights Settings
326Final/326Final/main.c
 Ethan big commit
a7d0ae5 7 minutes ago
412 lines (327 sloc)  13.9 KB

=======
>>>>>>> branch 'master' of https://github.com/EthanLewis741/326Final.git
/*
 * Name:            Ethan Lewis
 *                  Cat Costantino
 * Title:           Lab 8
 * Section:         EGR 326-902
 * Description:     Takes user input from the keypad, every time the star is pressed, the current date
 *                  and time is saved from the RTC into Flash, and displayed onto the LCD
 *                  LCD code sourced from Valvono
 *                  I2C and Flash code Based on codes from Dr's Krug and Kandalft
 *
 */


/* DriverLib Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "driverlib.h"

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>
#include "ST7735.h"

#include <msp.h>

unsigned char timeDateToSet[15] = {0x55, 0x58, 0x23, 0x05, 0x21, 0x11, 0x19, 0}; // Place holder defualt Date
#define SLAVE_ADDR 0x68 // RTC slave address

#define CALIBRATION_START 0x000200054 // Starting memory address in flash
uint8_t read_back_data[16]; // array to hold values read back from flash
uint8_t* addr_pointer; // pointer to address in flash for reading back values

int main(void)
{
    /* Stop Watchdog  */
    MAP_WDT_A_holdTimer();


     // index
    // halting the watch dog is done in the system_msp432p401r.c startup
    ClockInit();// Setting MCLK to 48MHz for faster programming
    ST7735_InitR(INITR_BLACKTAB); // Initiate the LCD

    I2C1_init(); // Initiate the I2C communication for the RTC
    __enable_irq ( ); // enable global interrupts


    I2C1_burstWrite(SLAVE_ADDR, 0, 7, timeDateToSet); //Write the default date



    unsigned char timeDateReadback[7];
    unsigned char TimeDate1[7] = 0, TimeDate2[7] = 0, TimeDate3[7] = 0, TimeDate4[7] = 0, TimeDate5[7] = 0; //holds Alarm Values
    char TimeString1[20] = 0, TimeString2[20] = 0, TimeString3[20] = 0, TimeString4[20] = 0, TimeString5[20] = 0; //Holds Alarm Values as formated strings
    int Flag=0, i=0, j=0;


    while(1)
    {
        int Key = keypad_getkey(); // Read Keypad
        if(Key == 10 && Flag) // On star Press
        {
            I2C1_burstRead(SLAVE_ADDR, 0, 7, timeDateReadback); // Read from the I2C
            MemWriteInit(); // Iniates flash write

            //Saves the values read from the RTC to a variable
            if(i==0) for(j=0;j<8;j++) TimeDate1[j] = timeDateReadback[j];
            if(i==1) for(j=0;j<8;j++) TimeDate2[j] = timeDateReadback[j];
            if(i==2) for(j=0;j<8;j++) TimeDate3[j] = timeDateReadback[j];
            if(i==3) for(j=0;j<8;j++) TimeDate4[j] = timeDateReadback[j];
            if(i==4) for(j=0;j<8;j++) TimeDate5[j] = timeDateReadback[j];

            //Format the Alarms into the correct string format
            sprintf(TimeString1, "%x:%x:%x %x/%x/%x    ",
                    TimeDate1[2],
                    TimeDate1[1],
                    TimeDate1[0],
                    TimeDate1[5],
                    TimeDate1[4],
                    TimeDate1[6]);

            sprintf(TimeString2, "%x:%x:%x %x/%x/%x    ",
                    TimeDate2[2],
                    TimeDate2[1],
                    TimeDate2[0],
                    TimeDate2[5],
                    TimeDate2[4],
                    TimeDate2[6]);

            sprintf(TimeString3, "%x:%x:%x %x/%x/%x    ",
                    TimeDate3[2],
                    TimeDate3[1],
                    TimeDate3[0],
                    TimeDate3[5],
                    TimeDate3[4],
                    TimeDate3[6]);

            sprintf(TimeString4, "%x:%x:%x %x/%x/%x    ",
                    TimeDate4[2],
                    TimeDate4[1],
                    TimeDate4[0],
                    TimeDate4[5],
                    TimeDate4[4],
                    TimeDate4[6]);

            sprintf(TimeString5, "%x:%x:%x %x/%x/%x    ",
                    TimeDate5[2],
                    TimeDate5[1],
                    TimeDate5[0],
                    TimeDate5[5],
                    TimeDate5[4],
                    TimeDate5[6]);

            // Print the Values to the LCD
            Output_Clear(); // Clear display
            ST7735_DrawString(2,5,TimeString1,0xFFE0);
            ST7735_DrawString(2,7,TimeString2,0xFFE0);
            ST7735_DrawString(2,9,TimeString3,0xFFE0);
            ST7735_DrawString(2,11,TimeString4,0xFFE0);
            ST7735_DrawString(2,13,TimeString5,0xFFE0);

            //Write Values to the Correct memory address
            MemWrite(TimeDate1, CALIBRATION_START +(0xD * 0), 7);
            MemWrite(TimeDate2, CALIBRATION_START +(0xD * 1), 7);
            MemWrite(TimeDate3, CALIBRATION_START +(0xD * 2), 7);
            MemWrite(TimeDate4, CALIBRATION_START +(0xD * 3), 7);
            MemWrite(TimeDate5, CALIBRATION_START +(0xD * 4), 7);
            flashwritefinish(); // Relock Flash

            // Increment the counter
            i++;
            if(i>4) i = 0;

            Flag = 0;
        }
        if(Key == 255)
            Flag = 1;
<<<<<<< HEAD
    }

}

void MemWriteInit(void){
        addr_pointer = CALIBRATION_START; // point to address in flash for saving data
        uint8_t i;
        for(i=0; i<16; i++)
        {// read values in flash before programming
            read_back_data[i] = *addr_pointer++;
        }
        /* Unprotecting Info Bank 0, Sector 0 */
        MAP_FlashCtl_unprotectSector(FLASH_INFO_MEMORY_SPACE_BANK0,FLASH_SECTOR0);

        /* Erase the flash sector starting CALIBRATION_START. */
        while(!MAP_FlashCtl_eraseSector(CALIBRATION_START));

}

void MemWrite(char data[], unsigned int address, int length){
    /* Program the flash with the new data. */
        uint8_t i;
        while (!MAP_FlashCtl_programMemory(data,(void*) address, length)); // leave first 4 bytes unprogrammed
}

void MemRead(void){
    uint8_t i;
    addr_pointer = CALIBRATION_START; // point to address in flash for saved data
    for(i=0; i<16; i++)
    {// read values in flash after programming
        read_back_data[i] = *addr_pointer++;
=======
>>>>>>> branch 'master' of https://github.com/EthanLewis741/326Final.git
    }

}

void MemWriteInit(void){
        addr_pointer = CALIBRATION_START; // point to address in flash for saving data
        uint8_t i;
        for(i=0; i<16; i++)
        {// read values in flash before programming
            read_back_data[i] = *addr_pointer++;
        }
        /* Unprotecting Info Bank 0, Sector 0 */
        MAP_FlashCtl_unprotectSector(FLASH_INFO_MEMORY_SPACE_BANK0,FLASH_SECTOR0);

        /* Erase the flash sector starting CALIBRATION_START. */
        while(!MAP_FlashCtl_eraseSector(CALIBRATION_START));

}

void MemWrite(char data[], unsigned int address, int length){
    /* Program the flash with the new data. */
        uint8_t i;
        while (!MAP_FlashCtl_programMemory(data,(void*) address, length)); // leave first 4 bytes unprogrammed
}

void MemRead(void){
    uint8_t i;
    addr_pointer = CALIBRATION_START; // point to address in flash for saved data
    for(i=0; i<16; i++)
    {// read values in flash after programming
        read_back_data[i] = *addr_pointer++;
    }
}

void flashwritefinish(void)
{
    /* Setting the sector back to protected */
    MAP_FlashCtl_protectSector(FLASH_INFO_MEMORY_SPACE_BANK0,FLASH_SECTOR0);
}

void ClockInit(void){

    volatile uint32_t i;
    uint32_t currentPowerState;

    currentPowerState = PCM->CTL0 & PCM_CTL0_CPM_MASK;

    while ((PCM->CTL1 & PCM_CTL1_PMR_BUSY));
    PCM->CTL0 = PCM_CTL0_KEY_VAL | PCM_CTL0_AMR_1;
    while ((PCM->CTL1 & PCM_CTL1_PMR_BUSY));

    /* Step 2: Configure Flash wait-state to 1 for both banks 0 & 1 */
    FLCTL->BANK0_RDCTL = (FLCTL->BANK0_RDCTL & ~(FLCTL_BANK0_RDCTL_WAIT_MASK)) |
            FLCTL_BANK0_RDCTL_WAIT_1;
    FLCTL->BANK1_RDCTL = (FLCTL->BANK0_RDCTL & ~(FLCTL_BANK1_RDCTL_WAIT_MASK)) |
            FLCTL_BANK1_RDCTL_WAIT_1 ;

    /* Step 3: Configure HFXT to use 48MHz crystal, source to MCLK & HSMCLK*/


    PJ->SEL0 |= BIT2 | BIT3;                // Configure PJ.2/3 for HFXT function
    PJ->SEL1 &= ~(BIT2 | BIT3);

    CS->KEY = CS_KEY_VAL ;                  // Unlock CS module for register access
    CS->CTL2 |= CS_CTL2_HFXT_EN | CS_CTL2_HFXTFREQ_6 | CS_CTL2_HFXTDRIVE;
    while(CS->IFG & CS_IFG_HFXTIFG)
        CS->CLRIFG |= CS_CLRIFG_CLR_HFXTIFG;

    /* Select MCLK & HSMCLK = HFXT, no divider */
    CS->CTL1 = CS->CTL1 & ~(CS_CTL1_SELM_MASK | CS_CTL1_DIVM_MASK | CS_CTL1_SELS_MASK | CS_CTL1_DIVHS_MASK) |
            CS_CTL1_SELM__HFXTCLK | CS_CTL1_SELS__HFXTCLK;

    CS->CTL1 |= CS_CTL1_DIVS_2;

    CS->KEY = 0;                            // Lock CS module from unintended accesses

    /* Step 4: Output MCLK to port pin to demonstrate 48MHz operation */
    P7->DIR |= BIT0;
    P7->SEL0 |=BIT0;                 // Output MCLK
    P7->SEL1 &= ~BIT0;

}

void DatePrint (void){

    unsigned char timeDateReadback[7];

    I2C1_burstRead(SLAVE_ADDR, 0, 7, timeDateReadback);

    printf("%x:%x:%x\t",timeDateReadback[2],timeDateReadback[1],timeDateReadback[0]);
    switch(timeDateReadback[3])
    {
    case 0:
        printf("Sunday\t");
        break;
    case 1:
        printf("Monday\t");
        break;
    case 2:
        printf("Tuesday\t");
        break;
    case 3:
        printf("Wednesday\t");
        break;
    case 4:
        printf("Thursday\t");
        break;
    case 5:
        printf("Friday\t");
        break;
    case 6:
        printf("Saturday\t");
        break;
    }
    printf("%x/%x/20%x",timeDateReadback[5],timeDateReadback[4],timeDateReadback[6]);
    printf("\n");

    SysTick_delay(237);
}

void I2C1_init(void) {
    EUSCI_B1->CTLW0 |= 1;             /* disable UCB1 during config */
    EUSCI_B1->CTLW0 = 0x0F81;         /* 7-bit slave addr, master, I2C, synch mode, use SMCLK */
    EUSCI_B1->BRW = 30;               /* set clock prescaler 3MHz / 30 = 100kHz */
    P6->SEL0 |= 0x30;             /* P6.5, P6.4 for UCB1 */
    P6->SEL1 &= ~0x30;
    EUSCI_B1->CTLW0 &= ~1;            /* enable UCB1 after config */
}

int I2C1_burstWrite(int slaveAddr, unsigned char memAddr, int byteCount, unsigned char* data)
{
    if (byteCount <= 0)
        return -1;              /* no write was performed */

    EUSCI_B1->I2CSA = slaveAddr;      /* setup slave address */
    EUSCI_B1->CTLW0 |= 0x0010;        /* enable transmitter */
    EUSCI_B1->CTLW0 |= 0x0002;        /* generate START and send slave address */
    while((EUSCI_B1->CTLW0 & 2));   /* wait until slave address is sent */
    EUSCI_B1->TXBUF = memAddr;        /* send memory address to slave */

    /* send data one byte at a time */
    do {
        while(!(EUSCI_B1->IFG & 2));  /* wait till it's ready to transmit */
        EUSCI_B1->TXBUF = *data++;    /* send data to slave */
        byteCount--;
        __delay_cycles(1000);
     } while (byteCount > 0);

    while(!(EUSCI_B1->IFG & 2));      /* wait till last transmit is done */
    EUSCI_B1->CTLW0 |= 0x0004;        /* send STOP */
    while(EUSCI_B1->CTLW0 & 4) ;      /* wait until STOP is sent */

    return 0;                   /* no error */
}

int I2C1_burstRead(int slaveAddr, unsigned char memAddr, int byteCount, unsigned char* data) {
    if (byteCount <= 0)
        return -1;              /* no read was performed */

    EUSCI_B1->I2CSA = slaveAddr;      /* setup slave address */
    EUSCI_B1->CTLW0 |= 0x0010;        /* enable transmitter */
    EUSCI_B1->CTLW0 |= 0x0002;        /* generate START and send slave address */
    while((EUSCI_B1->CTLW0 & 2));   /* wait until slave address is sent */
    EUSCI_B1->TXBUF = memAddr;        /* send memory address to slave */
    while(!(EUSCI_B1->IFG & 2));      /* wait till last transmit is done */
    EUSCI_B1->CTLW0 &= ~0x0010;       /* enable receiver */
    EUSCI_B1->CTLW0 |= 0x0002;        /* generate RESTART and send slave address */
    while(EUSCI_B1->CTLW0 & 2);       /* wait till RESTART is finished */

    /* receive data one byte at a time */
    do {
        if (byteCount == 1)     /* when only one byte of data is left */
            EUSCI_B1->CTLW0 |= 0x0004; /* setup to send STOP after the last byte is received */

        while(!(EUSCI_B1->IFG & 1));  /* wait till data is received */
        *data++ = EUSCI_B1->RXBUF;    /* read the received data */
        byteCount--;
    } while (byteCount);

    while(EUSCI_B1->CTLW0 & 4) ;      /* wait until STOP is sent */

    return 0;                   /* no error */
}


void SysTick_delay (uint16_t delay)
{ // Systick delay function
    static int Init = 1;
    if(Init)
    {
        SysTick -> CTRL = 0; // disable SysTick During step
        SysTick -> LOAD = 0x00FFFFFF; // max reload value
        SysTick -> VAL = 0; // any write to current clears it
        SysTick -> CTRL = 0x00000005; // enable systic, 3MHz, No Interrupts
        Init = 0;
    }
    SysTick -> LOAD = ((delay * 3000) - 1); //delay for 1 msecond per delay value
    SysTick -> VAL = 0; // any write to CVR clears it
    while ( (SysTick->CTRL & 0x00010000) == 0); // wait for flag to be SET
}

char keypad_getkey() { // assumes port 4 bits 0-3 are connected to rows

    int row, col, num=0; // bits 4,5,6 are connected to columns
    const char column_select[] = {0b00010000, 0b01000000, 0b10000000}; // one column is active

    static int Init = 1;
    if(Init)
    {
        //Columns
        P2->SEL0    &=~ (BIT7|BIT6|BIT4);
        P2->SEL1    &=~ (BIT7|BIT6|BIT4);
        P2->DIR     &=~ (BIT7|BIT6|BIT4);
        P2->REN     |=  (BIT7|BIT6|BIT4);
        P2->OUT     |=  (BIT7|BIT6|BIT4);

        //Rows
        P5->SEL0    &=~ BIT6;
        P5->SEL1    &=~ BIT6;
        P5->DIR     &=~ BIT6;
        P5->REN     |=  BIT6;
        P5->OUT     |=  BIT6;

        P6->SEL0    &=~ (BIT6|BIT7);
        P6->SEL1    &=~ (BIT6|BIT7);
        P6->DIR     &=~ (BIT6|BIT7);
        P6->REN     |=  (BIT6|BIT7);
        P6->OUT     |=  (BIT6|BIT7);

        P2->SEL0    &=~ BIT3; // Setup the P1.1 on the Launchpad as Input, Pull Up Resistor
        P2->SEL1    &=~ BIT3;
        P2->DIR     &=~ BIT3;
        P2->REN     |=  BIT3;
        P2->OUT     |=  BIT3; //Input, Pull up Resistor

        Init = 0;
    }

    // Activates one column at a time, read the input to see which column
    for (col = 0; col < 3; col++) {
        P2->DIR &= ~0b11010000; // 1 disable all columns
        P2->DIR |= column_select[col]; // 2 enable one column at a time
        P2->OUT &= ~column_select[col]; // 3 drive the active column low

        __delay_cycles(10); // 4 wait for signal to settle

        row = ( ((P5->IN & BIT6)>>3) | ((P6 ->IN & (BIT6))>>4) | ((P6 ->IN & (BIT7))>>6) | ((P2->IN & BIT3)>>3)); // 5 read all rows

        P2->OUT |= column_select[col]; // 6 drive the active column high

        if (row != 0b00001111)
            break; // 7 if one of the input is low,
    // some key is pressed.
    }

    P2->OUT |= 0b11010000; // 8 drive all columns high before disable
    P2->DIR &= ~0b11010000; // 9 disable all columns

    if (col == 3)
        num =  255; // 10 if we get here, no key is pressed
    // 10 gets here when one of the columns has key pressed,
    // check which row it is
    if (row == 0b00001110) num = col + 1; // key in row 0
    if (row == 0b00001101) num = 3 + col + 1; // key in row 1
    if (row == 0b00001011) num = 6 + col + 1; // key in row 2
    if (row == 0b00000111) num = 9 + col + 1; // key in row 3
    if(num == 11)
        num = 0;


    return num;

}

void flashwritefinish(void)
{
    /* Setting the sector back to protected */
    MAP_FlashCtl_protectSector(FLASH_INFO_MEMORY_SPACE_BANK0,FLASH_SECTOR0);
}

void ClockInit(void){

    volatile uint32_t i;
    uint32_t currentPowerState;

    currentPowerState = PCM->CTL0 & PCM_CTL0_CPM_MASK;

    while ((PCM->CTL1 & PCM_CTL1_PMR_BUSY));
    PCM->CTL0 = PCM_CTL0_KEY_VAL | PCM_CTL0_AMR_1;
    while ((PCM->CTL1 & PCM_CTL1_PMR_BUSY));

    /* Step 2: Configure Flash wait-state to 1 for both banks 0 & 1 */
    FLCTL->BANK0_RDCTL = (FLCTL->BANK0_RDCTL & ~(FLCTL_BANK0_RDCTL_WAIT_MASK)) |
            FLCTL_BANK0_RDCTL_WAIT_1;
    FLCTL->BANK1_RDCTL = (FLCTL->BANK0_RDCTL & ~(FLCTL_BANK1_RDCTL_WAIT_MASK)) |
            FLCTL_BANK1_RDCTL_WAIT_1 ;

    /* Step 3: Configure HFXT to use 48MHz crystal, source to MCLK & HSMCLK*/


    PJ->SEL0 |= BIT2 | BIT3;                // Configure PJ.2/3 for HFXT function
    PJ->SEL1 &= ~(BIT2 | BIT3);

    CS->KEY = CS_KEY_VAL ;                  // Unlock CS module for register access
    CS->CTL2 |= CS_CTL2_HFXT_EN | CS_CTL2_HFXTFREQ_6 | CS_CTL2_HFXTDRIVE;
    while(CS->IFG & CS_IFG_HFXTIFG)
        CS->CLRIFG |= CS_CLRIFG_CLR_HFXTIFG;

    /* Select MCLK & HSMCLK = HFXT, no divider */
    CS->CTL1 = CS->CTL1 & ~(CS_CTL1_SELM_MASK | CS_CTL1_DIVM_MASK | CS_CTL1_SELS_MASK | CS_CTL1_DIVHS_MASK) |
            CS_CTL1_SELM__HFXTCLK | CS_CTL1_SELS__HFXTCLK;

    CS->CTL1 |= CS_CTL1_DIVS_2;

    CS->KEY = 0;                            // Lock CS module from unintended accesses

    /* Step 4: Output MCLK to port pin to demonstrate 48MHz operation */
    P7->DIR |= BIT0;
    P7->SEL0 |=BIT0;                 // Output MCLK
    P7->SEL1 &= ~BIT0;

}

void DatePrint (void){

    unsigned char timeDateReadback[7];

    I2C1_burstRead(SLAVE_ADDR, 0, 7, timeDateReadback);

    printf("%x:%x:%x\t",timeDateReadback[2],timeDateReadback[1],timeDateReadback[0]);
    switch(timeDateReadback[3])
    {
    case 0:
        printf("Sunday\t");
        break;
    case 1:
        printf("Monday\t");
        break;
    case 2:
        printf("Tuesday\t");
        break;
    case 3:
        printf("Wednesday\t");
        break;
    case 4:
        printf("Thursday\t");
        break;
    case 5:
        printf("Friday\t");
        break;
    case 6:
        printf("Saturday\t");
        break;
    }
    printf("%x/%x/20%x",timeDateReadback[5],timeDateReadback[4],timeDateReadback[6]);
    printf("\n");

    SysTick_delay(237);
}

void I2C1_init(void) {
    EUSCI_B1->CTLW0 |= 1;             /* disable UCB1 during config */
    EUSCI_B1->CTLW0 = 0x0F81;         /* 7-bit slave addr, master, I2C, synch mode, use SMCLK */
    EUSCI_B1->BRW = 30;               /* set clock prescaler 3MHz / 30 = 100kHz */
    P6->SEL0 |= 0x30;             /* P6.5, P6.4 for UCB1 */
    P6->SEL1 &= ~0x30;
    EUSCI_B1->CTLW0 &= ~1;            /* enable UCB1 after config */
}

int I2C1_burstWrite(int slaveAddr, unsigned char memAddr, int byteCount, unsigned char* data)
{
    if (byteCount <= 0)
        return -1;              /* no write was performed */

    EUSCI_B1->I2CSA = slaveAddr;      /* setup slave address */
    EUSCI_B1->CTLW0 |= 0x0010;        /* enable transmitter */
    EUSCI_B1->CTLW0 |= 0x0002;        /* generate START and send slave address */
    while((EUSCI_B1->CTLW0 & 2));   /* wait until slave address is sent */
    EUSCI_B1->TXBUF = memAddr;        /* send memory address to slave */

    /* send data one byte at a time */
    do {
        while(!(EUSCI_B1->IFG & 2));  /* wait till it's ready to transmit */
        EUSCI_B1->TXBUF = *data++;    /* send data to slave */
        byteCount--;
        __delay_cycles(1000);
     } while (byteCount > 0);

    while(!(EUSCI_B1->IFG & 2));      /* wait till last transmit is done */
    EUSCI_B1->CTLW0 |= 0x0004;        /* send STOP */
    while(EUSCI_B1->CTLW0 & 4) ;      /* wait until STOP is sent */

    return 0;                   /* no error */
}

int I2C1_burstRead(int slaveAddr, unsigned char memAddr, int byteCount, unsigned char* data) {
    if (byteCount <= 0)
        return -1;              /* no read was performed */

    EUSCI_B1->I2CSA = slaveAddr;      /* setup slave address */
    EUSCI_B1->CTLW0 |= 0x0010;        /* enable transmitter */
    EUSCI_B1->CTLW0 |= 0x0002;        /* generate START and send slave address */
    while((EUSCI_B1->CTLW0 & 2));   /* wait until slave address is sent */
    EUSCI_B1->TXBUF = memAddr;        /* send memory address to slave */
    while(!(EUSCI_B1->IFG & 2));      /* wait till last transmit is done */
    EUSCI_B1->CTLW0 &= ~0x0010;       /* enable receiver */
    EUSCI_B1->CTLW0 |= 0x0002;        /* generate RESTART and send slave address */
    while(EUSCI_B1->CTLW0 & 2);       /* wait till RESTART is finished */

    /* receive data one byte at a time */
    do {
        if (byteCount == 1)     /* when only one byte of data is left */
            EUSCI_B1->CTLW0 |= 0x0004; /* setup to send STOP after the last byte is received */

        while(!(EUSCI_B1->IFG & 1));  /* wait till data is received */
        *data++ = EUSCI_B1->RXBUF;    /* read the received data */
        byteCount--;
    } while (byteCount);

    while(EUSCI_B1->CTLW0 & 4) ;      /* wait until STOP is sent */

    return 0;                   /* no error */
}


void SysTick_delay (uint16_t delay)
{ // Systick delay function
    static int Init = 1;
    if(Init)
    {
        SysTick -> CTRL = 0; // disable SysTick During step
        SysTick -> LOAD = 0x00FFFFFF; // max reload value
        SysTick -> VAL = 0; // any write to current clears it
        SysTick -> CTRL = 0x00000005; // enable systic, 3MHz, No Interrupts
        Init = 0;
    }
    SysTick -> LOAD = ((delay * 3000) - 1); //delay for 1 msecond per delay value
    SysTick -> VAL = 0; // any write to CVR clears it
    while ( (SysTick->CTRL & 0x00010000) == 0); // wait for flag to be SET
}

char keypad_getkey() { // assumes port 4 bits 0-3 are connected to rows

    int row, col, num=0; // bits 4,5,6 are connected to columns
    const char column_select[] = {0b00010000, 0b01000000, 0b10000000}; // one column is active

    static int Init = 1;
    if(Init)
    {
        //Columns
        P2->SEL0    &=~ (BIT7|BIT6|BIT4);
        P2->SEL1    &=~ (BIT7|BIT6|BIT4);
        P2->DIR     &=~ (BIT7|BIT6|BIT4);
        P2->REN     |=  (BIT7|BIT6|BIT4);
        P2->OUT     |=  (BIT7|BIT6|BIT4);

        //Rows
        P5->SEL0    &=~ BIT6;
        P5->SEL1    &=~ BIT6;
        P5->DIR     &=~ BIT6;
        P5->REN     |=  BIT6;
        P5->OUT     |=  BIT6;

        P6->SEL0    &=~ (BIT6|BIT7);
        P6->SEL1    &=~ (BIT6|BIT7);
        P6->DIR     &=~ (BIT6|BIT7);
        P6->REN     |=  (BIT6|BIT7);
        P6->OUT     |=  (BIT6|BIT7);

        P2->SEL0    &=~ BIT3; // Setup the P1.1 on the Launchpad as Input, Pull Up Resistor
        P2->SEL1    &=~ BIT3;
        P2->DIR     &=~ BIT3;
        P2->REN     |=  BIT3;
        P2->OUT     |=  BIT3; //Input, Pull up Resistor

        Init = 0;
    }

    // Activates one column at a time, read the input to see which column
    for (col = 0; col < 3; col++) {
        P2->DIR &= ~0b11010000; // 1 disable all columns
        P2->DIR |= column_select[col]; // 2 enable one column at a time
        P2->OUT &= ~column_select[col]; // 3 drive the active column low

        __delay_cycles(10); // 4 wait for signal to settle

        row = ( ((P5->IN & BIT6)>>3) | ((P6 ->IN & (BIT6))>>4) | ((P6 ->IN & (BIT7))>>6) | ((P2->IN & BIT3)>>3)); // 5 read all rows

        P2->OUT |= column_select[col]; // 6 drive the active column high

        if (row != 0b00001111)
            break; // 7 if one of the input is low,
    // some key is pressed.
    }

    P2->OUT |= 0b11010000; // 8 drive all columns high before disable
    P2->DIR &= ~0b11010000; // 9 disable all columns

    if (col == 3)
        num =  255; // 10 if we get here, no key is pressed
    // 10 gets here when one of the columns has key pressed,
    // check which row it is
    if (row == 0b00001110) num = col + 1; // key in row 0
    if (row == 0b00001101) num = 3 + col + 1; // key in row 1
    if (row == 0b00001011) num = 6 + col + 1; // key in row 2
    if (row == 0b00000111) num = 9 + col + 1; // key in row 3
    if(num == 11)
        num = 0;


    return num;

}
© 2019 GitHub, Inc.
Terms
Privacy
Security
Status
Help
Contact GitHub
Pricing
API
Training
Blog
About
