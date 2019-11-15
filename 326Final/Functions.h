/*
 * Functions.h
 *
 *  Created on: Oct 14, 2019
 *      Author: lewiset
 */
#define CALIBRATION_START 0x000200054 // Starting memory address in flash
uint8_t read_back_data[16]; // array to hold values read back from flash
uint8_t* addr_pointer; // pointer to address in flash for reading back values

#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_


////////////////////////////////////////////////////////////
///                    General                          ///
///////////////////////////////////////////////////////////
void SysTick_delay (uint16_t delay)
{ // Systick delay function
    static int Init = 1;
    int i;
    if(Init)
    {
        SysTick -> CTRL = 0; // disable SysTick During step
        SysTick -> LOAD = 0x00FFFFFF; // max reload value
        SysTick -> VAL = 0; // any write to current clears it
        SysTick -> CTRL = 0x00000005; // enable systic, 3MHz, No Interrupts
        Init = 0;
    }
    for (i = 0; i<=delay; i++)
    {
        SysTick -> LOAD = ((48000) - 1); //delay for 1 msecond per delay value
        SysTick -> VAL = 0; // any write to CVR clears it
        while ( (SysTick->CTRL & 0x00010000) == 0); // wait for flag to be SET
    }

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
}

////////////////////////////////////////////////////////////
///                    Rotary Encoder                    ///
///////////////////////////////////////////////////////////
void TimerA_Capture_Init (void) // set up timer A2.1 capture
{
    P5->SEL0 |= BIT6; // TA0.CCI2A input capture pin, second function
    P5->SEL1 &= ~ BIT6; // TA0.CCI2A input capture pin, second function
    P5->DIR &= ~ BIT6;

    TIMER_A2->CTL |= TIMER_A_CTL_TASSEL_2 | // Use SMCLK as clock source,
    TIMER_A_CTL_MC_2| // Start timer in continuous mode
    TIMER_A_CTL_CLR; // clear TA0R
    //(0x0214)
    TIMER_A2->CCTL[1] |= TIMER_A_CCTLN_CM_3 | // Capture rising and falling edge,
    TIMER_A_CCTLN_CCIS_0 | // Use CCI2A
    TIMER_A_CCTLN_CCIE | // Enable capture interrupt
    TIMER_A_CCTLN_CAP | // Enable capture mode,
    TIMER_A_CCTLN_SCS; // Synchronous capture
    //(0x4910)

    NVIC_EnableIRQ (TA2_N_IRQn); // enable capture interrupt
}

uint8_t RotaryButton()
{
    #define pin         P4
    #define bit         BIT1
    static int Init = 1;
    if(Init)
    {
        pin->SEL0    &=~ bit; // Setup the P1.1 on the Launchpad as Input, Pull Up Resistor
        pin->SEL1    &=~ bit;
        pin->DIR     &=~ bit;
        pin->REN     |=  bit;
        pin->OUT     |=  bit; //Input, Pull Up Resistor
        Init = 0;
    }

    int i;
    int shift = log(bit)/log(2);
    static uint16_t State = 0; // Current debounce status

    for(i=0;i<100;i++)
    {
        State=((State<<1) | (pin ->IN & bit)>> shift | 0xf800);
        if(State==0xfc00)
            return 1;
    }
    __delay_cycles(2);
    return 0;
}

////////////////////////////////////////////////////////////
///                    Flash Control                    ///
///////////////////////////////////////////////////////////
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


////////////////////////////////////////////////////////////
///                        I2C                           ///
///////////////////////////////////////////////////////////
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



////////////////////////////////////////////////////////////
///                        GPIO                          ///
///////////////////////////////////////////////////////////
uint8_t DebounceSwitch1()
{
    #define pin         P6
    #define bit         BIT4
    static int Init = 1;
    if(Init)
    {
        pin->SEL0    &=~ bit; // Setup the P1.1 on the Launchpad as Input, Pull Up Resistor
        pin->SEL1    &=~ bit;
        pin->DIR     &=~ bit;
        pin->REN     |=  bit;
        pin->OUT     |=  bit; //Input, Pull Up Resistor
        Init = 0;
    }

    int shift = log(bit)/log(2);
    static uint16_t State = 0; // Current debounce status

    // read switch, upper 5 bits of State are don't cares
    State=((State<<1) | (pin ->IN & bit)>> shift | 0xf800);

    if(State==0xfc00)
        return 1;
    // indicates 0 level is stable for 10 consecutive calls
    __delay_cycles(2);
    return 0;
}


#endif /* FUNCTIONS_H_ */
