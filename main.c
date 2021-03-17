/*
 * Include Files
 */
#include <msp430.h> 


/*
 * Compiler Switch Definition
 */
/* Enter LPM0 when VLO calibration */
/*#define VLO_CAL_LPM0*/

//#define MSP430_EVM


/*
 * Constant Definition
 */
/* WDI Pin Definition */
#define WDI_PORT_DIR    P1DIR
#define WDI_PORT_IN     P1IN
#define WDI_PORT_PIN    (BIT1)

/* WDT EN SW Pin Definition */
#define WDT_PORT_DIR    P1DIR
#define WDT_PORT_IN     P1IN
#define WDT_PORT_PIN    (BIT2)

/* RESET in Pin Definition */
#define RESETIN_PORT_DIR  P1DIR
#define RESETIN_PORT_IN  P1IN
#define RESETIN_PORT_PIN  (BIT0)

/* POWER EN Pin Definition */
#define POWEREN_PORT_DIR    P1DIR
#define POWEREN_PORT_OUT    P1OUT
#define POWEREN_PORT_PIN    (BIT3)

#define POWEREN_ACTIVE      (POWEREN_PORT_OUT |= POWEREN_PORT_PIN)
#define POWEREN_INACTIVE    (POWEREN_PORT_OUT &= ~POWEREN_PORT_PIN)

/* WDT HW EN Pin Definition */
#define WDTEN_HW_PORT_DIR   P1DIR
#define WDTEN_HW_PORT_IN   P1IN
#define WDTEN_HW_PORT_PIN   (BIT4)

/* LED Pin Definition */
#define LED_PORT_DIR    P1DIR
#define LED_PORT_OUT    P1OUT
#define LED_PORT_PIN    (BIT5)

#define LED_ACTIVE      (LED_PORT_OUT |= LED_PORT_PIN)
#define LED_INACTIVE    (LED_PORT_OUT &= ~LED_PORT_PIN)
#define LED_TOGGLE      (LED_PORT_OUT ^= LED_PORT_PIN)


/* MODE Pin Definition */
#define MODE_PORT_DIR   P2DIR
#define MODE_PORT_OUT   P2OUT
#define MODE_PORT_IN    P2IN
#define MODE_PORT_SEL   P2SEL
#define MODE_PORT_REN   P2REN
#define MODE_PORT_PIN   (BIT6)

/* I2C Slave Device Definition */
#define SLAVE_ADDRESS       (0x29 << 1)
#define REG_TIME_OUT        0x15
//#define REG_RST_COUNT     0x16
#define REG_PRE_TIME        0x16
#define REG_TIME_REMAIN     0x25
#define REG_PRE_REMAIN      0x26
#define REG_FW_VERSION      0x27
#define REG_PWR_MODE        0x28
#define REG_CMD        0x30

/* Power Button Mode: bit 0: 0-> AT mode, 1-> ATX mode */
#define PWR_BUTTON_MODE 0x00

/* Definition of Power Button */
#define PWR_BUTTON_ATX  0x01

/* WDI Detection delay time: N * 50ms */
#define WDI_DELAY_TIME  (20)

/* Reset Time Out: N * 2 * 50ms */
#define RST_TIME_OUT    (60 * 10)

/* Reset Time Out 300 seconds: N * 2 * 50ms */
#define RST_TIME_OUT_P  (300 * 10)

/* Reset Pulse time: N * 50ms */
#define RST_PUS_TIME    (4)

/* Reset Pulse Count: N */
#define RST_COUNT       2

/* Power Pulse time: N * 50ms */
#define PWR_ATT0_TIME   (20)
//#define PWR_ATT1_TIME (0)
//#define PWR_ATT2_TIME (0)
#define PWR_ATT3_TIME   (1)
//#define PWR_ATXT0_TIME    (20)
#define PWR_ATXT1_TIME  (30)
#define PWR_ATXT2_TIME  (30)
//#define PWR_ATXT3_TIME    (1)

/* The definition for variable, clock */
typedef enum
{
    DCO_1MHZ = 0,
//  DCO_8MHZ,
//  DCO_12MHZ,
    DCO_16MHZ
} DCO_FREQ;

/* The definition for variable, I2CState */
typedef enum
{
    I2C_IDLE = 0,
    I2C_RECE_ADDR,
    I2C_RECE_DATA,
    I2C_SEND_RX_ACK,
    I2C_SEND_TX_ACK,
    I2C_SEND_DATA,
    I2C_RECE_ACK
} I2C_STATE;

/* The definition for variable, WDIState */
//typedef enum
//{
//  WDI_LOW = 0,
//  WDI_LOW_HIGH,
//  WDI_HIGH,
//  WDI_HIGH_LOW
//} WDI_STATE;

/* The definition for variable, WDTState */
typedef enum
{
    WDT_LOW = 0,
    WDT_LOW_HIGH,
    WDT_HIGH,
    WDT_HIGH_LOW
} WDT_STATE;

/* The definition for variable, PowerPulse */
typedef enum
{
    PWR_BUTTON_IDLE = 0,
    PWR_BUTTON_T0,
    PWR_BUTTON_T1,
    PWR_BUTTON_T2,
    PWR_BUTTON_END
} PWR_BUTTON;


/*
 * Global Variables Definition
 */
/* Variables for I2C communication */
I2C_STATE I2CState;
unsigned char I2CRegister;
unsigned char *pI2Cdata;

/* Variable for VLO count for 50ms */
unsigned int VLO50msCount;

/* Variable for Watchdog active */
unsigned char WachdogActive;

/* Variable for WDI detection delay */
unsigned char WDICountDown;

/* Variables for WDI detection */
unsigned char WDIPreLevel;
unsigned char WDICurLevel;
//WDI_STATE WDIState;
unsigned char WDITrigger;

/* Variables for WDT detection */
unsigned char WDTPreLevel;
unsigned char WDTCurLevel;
WDT_STATE WDTState;

/* Variables for Reset time out */
unsigned int ResetCountDown;
unsigned char Count50ms;
unsigned char ResetPulseCountDown;
unsigned char ResetCount;
unsigned int ActiveResetTimeOut;
//unsigned char ActiveResetCount;
unsigned int ActivePreTime;

/* Timer counter*/
unsigned char TimerMinFlag;
unsigned int CountOneMin;
unsigned char TimerHalfSecFlag;
unsigned int CountHalfSec;
unsigned int TimerCount;
unsigned char LedCycle;
//unsigned char LedTimes;
unsigned char LedFlag;
unsigned char LedCount;

unsigned char WaitInfoATimeOut;
unsigned char ResetFlag;
unsigned int ResetInCount;

/* Variables for POWER EN & POWER BTN */
unsigned char PowerPulseCountDown;
PWR_BUTTON PowerButton;

/* Variables for I2C registers */
unsigned char I2CResetTimeOut[3], i2c_data_count;
//unsigned char I2CResetCount[2];
unsigned char I2CPowerMode[2];
unsigned char I2CPreTime[3];
unsigned char I2CPreTimeRemain[3];
unsigned char I2CResetTimeOutRemain[3];
unsigned char I2CCmd[3];

/* NULL data for I2C */
const unsigned char NullData = 0xFF;

/* FW version: 1.7 */
const unsigned char FWVersion[] =
{
    0x31,
    0x37,
    0xFF
};

#define CPU_F                               ((double)16000000)
#define delay_ms(x)                      __delay_cycles((long)(CPU_F*(double)x/1000.0))

/*
 * CalibrateVLO.c: Set DCO to 1M and VLO as ACLK source before calling this function
 * return: how many 1M clock per 1 VLO
 */
unsigned int CalibrateVLO(void)
{
    unsigned int VLOCount;

    /* To calibrate the VLO by Timer/DCO, the MSP430 can goes into LPM3 with clocking ACLK by VLO */
#ifdef VLO_CAL_LPM0
    /* Rising edge, CCIxB: ACLK, Capture Mode, Enable Interrupt */
    TA0CCTL0 = CM_1 + CCIS_1 + CAP + CCIE;
#else
    /* Rising edge, CCIxB: ACLK, Capture Mode */
    TA0CCTL0 = CM_1 + CCIS_1 + CAP;
#endif
    /* SMCLK, Continuous Up, Clear Counter */
    TA0CTL = TASSEL_2 + MC_2 + TACLR;
    /* Clear interrpt flag */
    TA0CCTL0 &= ~(CCIFG);

#ifdef VLO_CAL_LPM0
    /* Enter LPM0 */
    __low_power_mode_0();
    /* For debugger */
    __no_operation();
#else
    /* First rising edge of ACLK */
    while (!(TA0CCTL0 & CCIFG)) ;
    /* Clear interrpt flag */
    TA0CCTL0 &= ~(CCIFG);
#endif
    /* Get Timer Counter Value */
    VLOCount = TA0CCR0;

#ifdef VLO_CAL_LPM0
    /* Enter LPM0 */
    __low_power_mode_0();
    /* For debugger */
    __no_operation();
#else
    /* Second rising edge of ACLK */
    while (!(TA0CCTL0 & CCIFG)) ;
    /* Clear interrpt flag */
    TA0CCTL0 &= ~(CCIFG);
#endif
    /* Calculate the counter value of VLO */
    VLOCount = TA0CCR0 - VLOCount;
    /* ACLK, Stop, Clear Counter */
    TA0CTL = TASSEL_1 + MC_3 + TACLR;

    return VLOCount;
}


/*
 * SetDCO.c
 */
void SetDCO(DCO_FREQ clock)
{
    /* If calibration constants is erased */
    if (CALBC1_1MHZ == 0xFF)
    {
        while (1)
        {
            __delay_cycles(100000);
        }
    }

    /* Select lowest DCOx and MODx settings */
    DCOCTL = 0;

    switch (clock)
    {
    case DCO_1MHZ:
        /* Set range */
        BCSCTL1 = CALBC1_1MHZ;
        /* Set DCO step + modulation */
        DCOCTL = CALDCO_1MHZ;
        break;

//  case DCO_8MHZ:
        /* Set range */
//      BCSCTL1 = CALBC1_8MHZ;
        /* Set DCO step + modulation */
//      DCOCTL = CALDCO_8MHZ;
//      break;

//  case DCO_12MHZ:
        /* Set range */
//      BCSCTL1 = CALBC1_12MHZ;
        /* Set DCO step + modulation */
//      DCOCTL = CALDCO_12MHZ;
//      break;

    case DCO_16MHZ:
        /* Set range */
        BCSCTL1 = CALBC1_16MHZ;
        /* Set DCO step + modulation */
        DCOCTL = CALDCO_16MHZ;
        break;
    }

    /* LFXT1 = VLO */
    BCSCTL3 |= LFXT1S_2;
}


/*
 * IOInit.c
 */
void IOInit(void)
{
    /* Set unused pin to output low */
    P2OUT &= ~(BIT7);
    P2DIR |= BIT7;

    /* Set WDI as input pin */
    WDI_PORT_DIR &= ~WDI_PORT_PIN;
    /* Set WDT as input pin */
    WDT_PORT_DIR &= ~WDT_PORT_PIN;
    /* Set RST IN as input pin */
    RESETIN_PORT_DIR &= ~RESETIN_PORT_PIN;
    /* Set POWEREN as output pin */
    POWEREN_PORT_DIR |= POWEREN_PORT_PIN;
    /* Set WDT HW EN as input pin */
    WDTEN_HW_PORT_DIR &= ~WDTEN_HW_PORT_PIN;
    /* Set LED as output pin */
    LED_PORT_DIR |= LED_PORT_PIN;

    P1REN &= ~(WDI_PORT_PIN | WDT_PORT_PIN |RESETIN_PORT_PIN |WDTEN_HW_PORT_PIN );


    /* Set POWER EN as output high by input floating with external pull-up */
    POWEREN_ACTIVE;

    /* Set MODE as input pin with internal pull-up (ATX mode) */
    MODE_PORT_SEL &= ~MODE_PORT_PIN;
    MODE_PORT_OUT |= MODE_PORT_PIN;
    MODE_PORT_REN |= MODE_PORT_PIN;
    MODE_PORT_DIR &= ~MODE_PORT_PIN;
}


/*
 * TimerInit.c
 */
void TimerInit(void)
{
    /* Compare mode, CCR0 interrupt enabled */
    TA0CCTL0 = CCIE;
    /* CCR0 period time: 50ms */
    TA0CCR0 = VLO50msCount;
    /* ACLK, Continuous Up, Clear Counter */
    TA0CTL = TASSEL_1 + MC_2 + TACLR;
    /* SMCLK, Continuous Up, / 8, Clear Counter */
}


/*
 * I2CInit.c
 */
void I2CInit(void)
{
    /* Port & USI mode setup */
    USICTL0 = USIPE6 | USIPE7 | USISWRST;
    /* Enable I2C mode & USI interrupts */
    USICTL1 = USII2C | USIIE | USISTTIE;
    /* Setup clock polarity */
    USICKCTL = USICKPL;
    /* Disable automatic clear control */
    USICNT |= USIIFGCC;
    /* Enable USI */
    USICTL0 &= ~USISWRST;
    /* Clear pending flag */
    USICTL1 &= ~(USIIFG | USISTTIFG);
}


/*
 * WDIDetect.c
 */
unsigned char WDIDetect(void)
{
    /* Get current WDI level */
    WDICurLevel = WDI_PORT_IN & WDI_PORT_PIN;

    /* If WDI level doesn't change */
    if (WDIPreLevel == WDICurLevel)
    {
//      if (WDICurLevel)
//      {
//          WDIState = WDI_HIGH;
//      }
//      else
//      {
//          WDIState = WDI_LOW;
//      }

        return 0;
    }
    else
    {
        WDIPreLevel = WDICurLevel;

//      if (WDICurLevel)
//      {
//          WDIState = WDI_LOW_HIGH;
//      }
//      else
//      {
//          WDIState = WDI_HIGH_LOW;
//      }

        return 1;
    }
}


/*
 * WDTDetect.c
 */
void WDTDetect(void)
{
    /* Get current WDT level */
    WDTCurLevel = WDT_PORT_IN & WDT_PORT_PIN;

    /* If WDT level doesn't change */
    if (WDTPreLevel == WDTCurLevel)
    {
        if (WDTCurLevel)
        {
            WDTState = WDT_HIGH;
        }
        else
        {
            WDTState = WDT_LOW;
        }
    }
    else
    {
        WDTPreLevel = WDTCurLevel;

        if (WDTCurLevel)
        {
            WDTState = WDT_LOW_HIGH;

            /* Set WDI detection delay time when WDT re-enable */
            WDICountDown = WDI_DELAY_TIME - 1;

            /* Get WDI level */
            WDIPreLevel = WDI_PORT_IN & WDI_PORT_PIN;
//          if (WDIPreLevel)
//          {
//              WDIState = WDI_HIGH;
//          }
//          else
//          {
//              WDIState = WDI_LOW;
//          }
        }
        else
        {
            WDTState = WDT_HIGH_LOW;
        }
    }
}


/*
 * I2CProceed.c
 */
void I2CProceed(void)
{
    while(!(USICTL1 & USISTP));
    /* Stop condition */
    if (USICTL1 & USISTP)
    {
        /* SDA = input */
        USICTL0 &= ~USIOE;
        /* Clear stop flag */
        USICTL1 &= ~USISTP;

        if ((I2CState == I2C_RECE_DATA) || (I2CState == I2C_SEND_RX_ACK))
        {
            /* Update Time Out */
            if (I2CRegister == REG_TIME_OUT)
            {
                if ((I2CResetTimeOut[0] != 0) || (I2CResetTimeOut[1] != 0))
                {
                    ActiveResetTimeOut = I2CResetTimeOut[1];
                    ActiveResetTimeOut = I2CResetTimeOut[0] + (ActiveResetTimeOut << 8);
                    ResetCountDown = ActiveResetTimeOut - 1;
                }
                else
                {
                    I2CResetTimeOut[0] = ActiveResetTimeOut & 0xFF;
                    I2CResetTimeOut[1] = (ActiveResetTimeOut >> 8) & 0xFF;
                }
            }
            /* Update Reset Count */
//          else if (I2CRegister == REG_RST_COUNT)
//          {
//              if (I2CResetCount[0] != 0)
//              {
//                  ActiveResetCount = I2CResetCount[0];
//              }
//              else
//              {
//                  I2CResetCount[0] = ActiveResetCount;
//              }
//          }
            /* Update INT Pre-time */
            else if (I2CRegister == REG_PRE_TIME)
            {
                /* Set INT Pre-time */
                ActivePreTime = I2CPreTime[1];
                ActivePreTime = I2CPreTime[0] + (ActivePreTime << 8);
                if (ActivePreTime > (ActiveResetTimeOut - ResetCountDown))
                {
                    /* INT Pre-time is not expired */
                    ActivePreTime -= (ActiveResetTimeOut - ResetCountDown);
                    /* Output high level on the INT */
                    //INT_INACTIVE;
                }
                else
                {
                    /* INT Pre-time is expired */
                    ActivePreTime = 0;
                    /* Output low level on the INT */
                    //INT_ACTIVE;
                }
            }
        }

        /* Reset state machine */
        I2CState = I2C_IDLE;
        I2CRegister = 0;
        pI2Cdata = (unsigned char *)&NullData;
    }
}


/*
 * main.c
 */
void main(void)
{
    /* Stop watchdog timer */
    WDTCTL = WDTPW | WDTHOLD;

    /* I/O pin initialization */
    IOInit();

    /* Set DCO to 1MHz */
    SetDCO(DCO_1MHZ);

    /* delay a while for DCO stable */
    __delay_cycles(5000);

    /* Calibrate VLO by DCO */
    VLO50msCount = CalibrateVLO();
    /* Calculate 50ms for VLO */
    VLO50msCount = 50000 / VLO50msCount;

    /* Set DCO to 16MHz */
    SetDCO(DCO_16MHZ);

    /* delay a while for DCO stable */
//  __delay_cycles(5000);

    /* Timer initialization */
    TimerInit();

    /* I2C initialization */
    I2CInit();

    /* Initial global variables */
    I2CState = I2C_IDLE;
    I2CRegister = 0;
    pI2Cdata = (unsigned char *)&NullData;

    /* Start Watchdog */
    WDTCTL = WDT_ARST_250;

    /* Global Interrupt Enable */
    __enable_interrupt();

    while (1)
    {
        ResetCount = 0;
        ResetFlag = 0;
        ResetInCount = 0;
        PowerPulseCountDown = 0;
        PowerButton = PWR_BUTTON_IDLE;
        Count50ms = 0;
        WDITrigger = 0;
        I2CCmd[0] = 0;
        I2CCmd[1] = 0;
        I2CCmd[2] = 0;
//      ActiveResetCount = RST_COUNT;
//      I2CResetCount[0] = RST_COUNT;
//      I2CResetCount[1] = 0xFF;
        I2CPreTime[0] = 0;
        I2CPreTime[1] = 0;
        I2CPreTime[2] = 0xFF;
        I2CPreTimeRemain[0] = 0;
        I2CPreTimeRemain[1] = 0;
        I2CPreTimeRemain[2] = 0xFF;
        I2CResetTimeOut[0] = RST_TIME_OUT & 0xFF;
        I2CResetTimeOut[1] = RST_TIME_OUT >> 8;
        I2CResetTimeOut[2] = 0xFF;
        I2CResetTimeOutRemain[0] = RST_TIME_OUT & 0xFF;
        I2CResetTimeOutRemain[1] = RST_TIME_OUT >> 8;
        I2CResetTimeOutRemain[2] = 0xFF;
        ActivePreTime = I2CPreTime[1];
        ActivePreTime = I2CPreTime[0] + (ActivePreTime << 8);
        ActivePreTime--;
        ActiveResetTimeOut = I2CResetTimeOut[1];
        ActiveResetTimeOut = I2CResetTimeOut[0] + (ActiveResetTimeOut << 8);
        ResetCountDown = ActiveResetTimeOut - 1;
        WDICountDown = WDI_DELAY_TIME - 1;

    TimerMinFlag = 0;
        CountOneMin = 0;
        TimerHalfSecFlag = 0;
        CountHalfSec = 0;
        TimerCount = 0;
        LedCycle = 0;
        //LedTimes = 0;
        LedCount = 0;

        WaitInfoATimeOut = 0;

        /* Get MODE pin level */
        if (MODE_PORT_IN & MODE_PORT_PIN)
        {
            I2CPowerMode[0] = 1;
        }
        else
        {
            I2CPowerMode[0] = 0;
        }
        I2CPowerMode[1] = 0xFF;

    TimerMinFlag = 1;

loop:
        LedFlag = 0;
        LED_ACTIVE;
    while((WDTEN_HW_PORT_IN & WDTEN_HW_PORT_PIN) == 0)
    {
            if((RESETIN_PORT_IN & RESETIN_PORT_PIN) == 0)
            {
                 while((RESETIN_PORT_IN & RESETIN_PORT_PIN) == 0)
                 {
                      delay_ms(1);
                      ResetInCount++;
                      if(ResetInCount == 1000)
                          POWEREN_INACTIVE;
                 }
                 POWEREN_ACTIVE;
             }

        if(I2CCmd[0] == 1) //接收到A信号
        {
                 TimerMinFlag = 0;
                 LedCycle = 2;
                 LedFlag = 1;
                 //LedTimes = 1;
                 LED_TOGGLE;
                 break;
        }

        if(CountOneMin == 120)
        {
            TimerMinFlag = 0;
        CountOneMin = 0;
                TimerCount = 0;
                WaitInfoATimeOut = 1;
        ResetCount++;
                POWEREN_INACTIVE;
                TimerHalfSecFlag = 1;
        }

        if(ResetCount == 4)
            while(1);

            if(ResetFlag == 1)
            {
                  if(CountHalfSec == 3)
                 {
                       ResetFlag = 0;
                       TimerHalfSecFlag = 0;
                       CountHalfSec = 0;
                       TimerCount = 0;
                       POWEREN_ACTIVE;
                       TimerMinFlag = 1;
                       CountOneMin = 0;
         }
            }

            if(WaitInfoATimeOut == 1)
            {
                 if(CountHalfSec == 3)
                 {
                       WaitInfoATimeOut = 0;
                       TimerHalfSecFlag = 0;
                       CountHalfSec = 0;
                       TimerCount = 0;
                       POWEREN_ACTIVE;
                       TimerMinFlag = 1;
         }
            }
    }

        /* wait for watchdog enable */
        while ((WDT_PORT_IN & WDT_PORT_PIN) == 0)
        {
            if((RESETIN_PORT_IN & RESETIN_PORT_PIN) == 0)
            {
                 while((RESETIN_PORT_IN & RESETIN_PORT_PIN) == 0)
                 {
                      delay_ms(1);
                      ResetInCount++;
                      if(ResetInCount == 1000)
                          POWEREN_INACTIVE;
                 }
                 POWEREN_ACTIVE;
             }
            /* Enter LPM3 */
            __low_power_mode_3();
            /* For debugger */
            __no_operation();

            /* Refresh Watchdog */
            WDTCTL = WDT_ARST_250;

  //          I2CProceed();
        }

        /* Get WDT pin level */
        WDTPreLevel = WDT_PORT_IN & WDT_PORT_PIN;
//      if (WDTPreLevel)
//      {
            WDTState = WDT_HIGH;
//      }
//      else
//      {
//          WDTState = WDT_LOW;
//      }

        /* Set WachdogActive */
        WachdogActive = 1;

        /* Get WDI level */
        WDIPreLevel = WDI_PORT_IN & WDI_PORT_PIN;

        /* while loop */
        while (WachdogActive)
        {
            /* Enter LPM3 */
            __low_power_mode_3();
            if((RESETIN_PORT_IN & RESETIN_PORT_PIN) == 0)
            {
                 while((RESETIN_PORT_IN & RESETIN_PORT_PIN) == 0)
                 {
                      delay_ms(1);
                      ResetInCount++;
                      if(ResetInCount == 1000)
                          POWEREN_INACTIVE;
                 }
                 POWEREN_ACTIVE;
             }
            /* For debugger */
            __no_operation();

            /* Refresh Watchdog */
            WDTCTL = WDT_ARST_250;

    //        I2CProceed();

            /* Count Down for Reset Low Pulse Time or Power Low Pulse Time */
            if (ResetPulseCountDown || PowerPulseCountDown)
            {
                if (ResetPulseCountDown)
                {
                    ResetPulseCountDown--;
                }
                else
                {
                    PowerPulseCountDown--;
                }
            }
            else
            {
#if 0
                if (PowerButton == PWR_BUTTON_T0)
                {
                    if (I2CPowerMode[0] & PWR_BUTTON_ATX)
                    {
                        /* Set PowerPulse */
                        PowerButton = PWR_BUTTON_T1;
                        /* Set Power Button T1 */
                        PowerPulseCountDown = PWR_ATXT1_TIME - 1;
                    }
                    else
                    {
                        /* Set PowerPulse */
                        PowerButton = PWR_BUTTON_IDLE;
                        /* Set Power Button T3 */
                        PowerPulseCountDown = PWR_ATT3_TIME - 1;
                    }
                    /* Output high on POWER EN */
                    POWEREN_INACTIVE;
                }
                else if (PowerButton == PWR_BUTTON_T1)
                {
                    /* Set PowerButton */
                    PowerButton = PWR_BUTTON_T2;
                    /* Set Power Button T2 */
                    PowerPulseCountDown = PWR_ATXT2_TIME - 1;
                    /* Output low on POWER BTN */
                    POWERBTN_ACTIVE;
                }
                else if (PowerButton == PWR_BUTTON_T2)
                {
                    /* Set PowerButton */
                    PowerButton = PWR_BUTTON_END;
                    /* Set Power Third Pulse Time */
                    PowerPulseCountDown = PWR_ATT3_TIME - 1;
                    /* Output low on POWER BTN */
                    POWERBTN_INACTIVE;
                }
                else
                {
                    if (RESET_PORT_DIR & RESET_PORT_PIN)
                    {
                        /* Output high on the Reset pin */
                        RESET_INACTIVE;
                        if (PowerButton == PWR_BUTTON_END)
                        {
                            /* Set PowerButton */
                            PowerButton = PWR_BUTTON_IDLE;
                            /* Clear WachdogActive */
                            WachdogActive = 0;
                        }
                    }
                }
#endif

                /* Count Down for WDI detection delay time */
                if (WDICountDown)
                {
                    WDICountDown--;

                    /* If WDICountDown is 0 */
                    if (WDICountDown == 0)
                    {
                        /* Check WDI level */
                        WDIPreLevel = WDI_PORT_IN & WDI_PORT_PIN;
                    }
                }
                else
                {
                    if (WDITrigger)
                    {
                        /* Check WDT level */
                        WDTDetect();
                    }

                    /* If WDT becomes enable */
                    if (WDTState == WDT_LOW_HIGH)
                    {
                        /* Get MODE pin level */
                        if (MODE_PORT_IN & MODE_PORT_PIN)
                        {
                            I2CPowerMode[0] = 1;
                        }
                        else
                        {
                            I2CPowerMode[0] = 0;
                        }
                    }

                    /* If WDT level is high */
                    if (WDTState == WDT_HIGH)
                    {
                        /* Check WDI level */
                        if (WDIDetect())
                        {
                            /* Re-set count down time when WDI trigger evenet is occured */
                            ResetCountDown = ActiveResetTimeOut - 1;
                            ResetCount = 0;

                            /* Output high level on the INT */
                            //INT_INACTIVE;
                            /* restore INT Pre-time */
                            ActivePreTime = I2CPreTime[1];
                            ActivePreTime = I2CPreTime[0] + (ActivePreTime << 8);
                            ActivePreTime--;

                            /* Set WDITrigger */
                            WDITrigger = 1;
                            LedCycle = 1;
                            //LedTimes = 1;
                            LED_TOGGLE;
                        }
                        else
                        {
                            /* 50ms counter for Reset Count Down */
                            Count50ms++;
                            /* Reset Count Down every 100ms */
                            if ((Count50ms & 0x01) == 0)
                            {
                                /* Reset Count Down */
                                if (ResetCountDown)
                                {
                                    ResetCountDown--;

                                    /* Update remaining Reset Time Out */
                                    I2CResetTimeOutRemain[0] = ResetCountDown & 0xFF;
                                    I2CResetTimeOutRemain[1] = ResetCountDown >> 8;

                                    /* INT Pre-time */
                                    if (ActivePreTime)
                                    {
                                        ActivePreTime--;
                                        /* Update remaining INT Pre-time */
                                        I2CPreTimeRemain[0] = ActivePreTime & 0xFF;
                                        I2CPreTimeRemain[1] = ActivePreTime >> 8;
                                    }
                                    else
                                    {
                                        /* Output low level on the INT */
                                        //INT_ACTIVE;
                                    }
                                }
                                else
                                {
                                    /* Re-set count down time when WDI trigger evenet is occured */
                                    ResetCountDown = ActiveResetTimeOut - 1;

                                    /* Output high level on the INT */
                                    //INT_INACTIVE;
                                    /* restore INT Pre-time */
                                    ActivePreTime = I2CPreTime[1];
                                    ActivePreTime = I2CPreTime[0] + (ActivePreTime << 8);
                                    ActivePreTime--;

                                    /* Clear WDITrigger */
                                    WDITrigger = 0;

                                    /* Increase Reset Pulse Count */
                                    ResetCount++;

//                                  if (ResetCount >= ActiveResetCount)
                                    if (ResetCount >= RST_COUNT)
                                    {
                                        ResetCount = 0;
                                        /* Set PowerButton */
                                        PowerButton = PWR_BUTTON_T0;
                                        /* Set Power Button T0 */
                                        PowerPulseCountDown = PWR_ATT0_TIME - 1;
                                        /* Output low on the Reset pin */
                                        //RESET_ACTIVE;
                                        /* Output low on POWER EN */
                                        POWEREN_INACTIVE;
                                    }
                                    else
                                    {
                                        /* Re-set count down time when WDI trigger evenet is occured */
                                        ResetCountDown = RST_TIME_OUT_P - 1;

                                        /* Set Reset Low Pulse Time */
                                        ResetPulseCountDown = RST_PUS_TIME - 1;
                                        /* Output low on the Reset pin */
                                        //RESET_ACTIVE;
                                        POWEREN_INACTIVE;
                                    }

                                    /* Set WDI detection delay time when Reset */
                                    WDICountDown = WDI_DELAY_TIME - 1;
                                    TimerHalfSecFlag = 1;
                                    CountHalfSec = 0;
                                    ResetFlag = 1;
                                    I2CCmd[0] = 0;
                                    goto loop;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}


/*
 * Timer0_A0 Interrupt Vector Handler
 */
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer0_A0(void)
{
    /* Clear interrpt flag */
    TA0CCTL0 &= ~(CCIFG);

    /* update CCR0 period time: 50ms */
    TA0CCR0 += VLO50msCount;

    if(LedFlag == 1)
    {
         //LedTimes--;
         LedCount++;
         if(LedCount == (LedCycle * 20))
         {
             LedCount = 0;
         LED_TOGGLE;
         }
    }

    if(TimerMinFlag == 1)
    {
        TimerCount++;
        if(TimerCount == 1200)
        {
            CountOneMin++;
            TimerCount = 0;
        }
    }

    if(TimerHalfSecFlag == 1)
    {
        TimerCount++;
        if(TimerCount == 10)
        {
            CountHalfSec++;
            TimerCount = 0;
        }
    }

    /* Exit low mode */
    __low_power_mode_off_on_exit();
}


/*
 * USI_I2C_ISR Interrupt Vector Handler
 */
#pragma vector=USI_VECTOR
__interrupt void USI_I2C_ISR(void)
{
    /* Start condition */
    if (USICTL1 & USISTTIFG)
    {
        /* Bit counter = 8, RX address */
        USICNT = (USICNT & 0xE0) + 0x08;
        /* Clear start flag */
        USICTL1 &= ~USISTTIFG;
        /* Go to next state: check address */
        I2CState = I2C_RECE_ADDR;
    }
    /* Counter interrupt flag */
    else if (USICTL1 & USIIFG)
    {
        switch (I2CState)
        {
        case I2C_RECE_ADDR:
            /* Address match */
            if (USISRL == SLAVE_ADDRESS)
            {
                /* SDA = output */
                USICTL0 |= USIOE;
                /* Go to next state: RX data */
                I2CState = I2C_SEND_RX_ACK;
                /* Send Ack */
                USISRL = 0x00;
                /* Bit counter = 1, send (N)Ack bit */
                USICNT |= 0x01;
            }
            else if (USISRL == (SLAVE_ADDRESS + 1))
            {
                /* SDA = output */
                USICTL0 |= USIOE;
                /* Go to next state: TX data */
                I2CState = I2C_RECE_ACK;
                /* Send Ack */
                USISRL = 0x00;
                /* Bit counter = 1, send (N)Ack bit */
                USICNT |= 0x01;
            }
            else
            {
                /* next state: prep for next Start */
                I2CState = I2C_IDLE;
                /* SCL release when the address is not own address */
                USICNT |= USISCLREL;
            }
            break;

        case I2C_RECE_DATA:
            /* Receive data of register */
            if (I2CRegister)
            {
//              if ((I2CRegister == REG_TIME_OUT) || (I2CRegister == REG_RST_COUNT) || (I2CRegister == REG_PRE_TIME))
                if ((I2CRegister == REG_TIME_OUT) || (I2CRegister == REG_PRE_TIME) || (I2CRegister == REG_CMD))
                {
                    i2c_data_count++;

                    if (i2c_data_count < 3)
                    {
                        *pI2Cdata++ = USISRL;
                    }
                    if (i2c_data_count == 4)
                    {
                        I2CProceed();
                        i2c_data_count = 0;
                    }
                }
            }
            else
            {
                I2CRegister = USISRL;
                if (I2CRegister == REG_TIME_OUT)
                {
                    pI2Cdata = I2CResetTimeOut;
                }
//              else if (I2CRegister == REG_RST_COUNT)
//              {
//                  pI2Cdata = I2CResetCount;
//              }
                else if (I2CRegister == REG_PRE_TIME)
                {
                    pI2Cdata = I2CPreTime;
                }
                else if (I2CRegister == REG_TIME_REMAIN)
                {
                    pI2Cdata = I2CResetTimeOutRemain;
                }
                else if (I2CRegister == REG_PRE_REMAIN)
                {
                    pI2Cdata = I2CPreTimeRemain;
                }
                else if (I2CRegister == REG_FW_VERSION)
                {
                    pI2Cdata = (unsigned char *)FWVersion;
                }
                else if (I2CRegister == REG_PWR_MODE)
                {
                    pI2Cdata = I2CPowerMode;
                }
                else if (I2CRegister == REG_CMD)
                {
                    pI2Cdata = I2CPowerMode;
                }
            }
            /* SDA = output */
            USICTL0 |= USIOE;
            /* Send Ack */
            USISRL = 0x00;
            /* Rcv another byte */
            I2CState = I2C_SEND_RX_ACK;
            /* Bit counter = 1, send (N)Ack bit */
            USICNT |= 0x01;
            break;

        case I2C_SEND_RX_ACK:
            /* SDA = input */
            USICTL0 &= ~USIOE;
            /* Bit counter = 8, RX data */
            USICNT |=  0x08;
            /* next state: Test data and (N)Ack */
            I2CState = I2C_RECE_DATA;
            break;

        case I2C_SEND_DATA:
            /* SDA = input */
            USICTL0 &= ~USIOE;
            /* Bit counter = 1, receive (N)Ack */
            USICNT |= 0x01;
            /* Go to next state: check (N)Ack */
            I2CState = I2C_RECE_ACK;
            break;
        case I2C_RECE_ACK:
            if(USISRL & 0x01)
                I2CProceed();
            else
            {
                /* SDA = output */
                USICTL0 |= USIOE;
                if (*pI2Cdata == 0xFF)
                {
                    USISRL = *pI2Cdata;
                }
                else
                {
                    USISRL = *pI2Cdata++;
                }
                /* Bit counter = 8, TX data */
                USICNT |= 0x08;
                /* Go to next state: receive (N)Ack */
                I2CState = I2C_SEND_DATA;
            }
            break;

        case I2C_IDLE:
        default:
            break;
        }
        /* Clear pending flags */
    }
    USICTL1 &= ~USIIFG;
}

