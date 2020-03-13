#include <msp430.h>
#include <inttypes.h>

#define pos1 4                                               // Digit A1 - L4
#define pos2 6                                               // Digit A2 - L6
#define pos3 8                                               // Digit A3 - L8
#define pos4 10                                              // Digit A4 - L10
#define pos5 2                                               // Digit A5 - L2
#define pos6 18                                              // Digit A6 - L18

const char digit[10] =
{
    0xFC,                                                    // "0"
    0x60,                                                    // "1"
    0xDB,                                                    // "2"
    0xF3,                                                    // "3"
    0x67,                                                    // "4"
    0xB7,                                                    // "5"
    0xBF,                                                    // "6"
    0xE4,                                                    // "7"
    0xFF,                                                    // "8"
    0xF7                                                     // "9"
};

uint8_t speedMatrix[4] = {0};
uint8_t defaultLcdDigit = 0;
volatile unsigned int speed = 0;
volatile unsigned int ADC_Result = 0;
// Global Variables Added 24.02.20
volatile uint8_t UARTbuffor[3] = {0};



void Init_GPIO()
{
    P1DIR = 0xFF; P2DIR = 0xFF; P3DIR = 0xFF; P4DIR = 0xFF;
    P5DIR = 0xFF; P6DIR = 0xFF; P7DIR = 0xFF; P8DIR = 0xFF;
    P1REN = 0xFF; P2REN = 0xFF; P3REN = 0xFF; P4REN = 0xFF;
    P5REN = 0xFF; P6REN = 0xFF; P7REN = 0xFF; P8REN = 0xFF;
    P1OUT = 0x00; P2OUT = 0x00; P3OUT = 0x00; P4OUT = 0x00;
    P5OUT = 0x00; P6OUT = 0x00; P7OUT = 0x00; P8OUT = 0x00;
}

void SetGPIO()
{
    // Configure UART pins
    P1SEL0 |= BIT0 | BIT1;                    // set 2-UART pin as second function

    // COnfigure PWM pins
    P8DIR |= BIT3; //output
    P8SEL0 |= BIT3; //PWM mode

    // Configure ADC A9 pin (P8.1)
    SYSCFG2 |= ADCPCTL9;

    // Configure  TA0CLK source 
    P1DIR &= ~BIT5;
    P1REN |=  BIT5;  //pull up resistor for open collector  (SENS pin) 
    P1OUT |= BIT5;
    P1SEL0 |= BIT5;  // P1.5 selected as TA0CLK

    // Configure XT1 pins
    P4SEL0 |= BIT1 | BIT2;  // set XT1 pin as second function

    // Configure LCD pins
    SYSCFG2 |= LCDPCTL;                                 // R13/R23/R33/LCDCAP0/LCDCAP1 pins selected

    LCDPCTL0 = 0xFFFF;
    LCDPCTL1 = 0x07FF;
    LCDPCTL2 = 0x00F0;                                  // L0~L26 & L36~L39 pins selected

    LCDCTL0 = LCDSSEL_0 | LCDDIV_7;                     // flcd ref freq is xtclk
}

void ConfigureLCD(void)
{
    // LCD Operation - Mode 3, internal 3.08v, charge pump 256Hz
    LCDVCTL = LCDCPEN | LCDREFEN | VLCD_6 | (LCDCPFSEL0 | LCDCPFSEL1 | LCDCPFSEL2 | LCDCPFSEL3);

    LCDMEMCTL |= LCDCLRM;                               // Clear LCD memory

    LCDCSSEL0 = 0x000F;                                 // Configure COMs and SEGs
    LCDCSSEL1 = 0x0000;                                 // L0, L1, L2, L3: COM pins
    LCDCSSEL2 = 0x0000;

    LCDM0 = 0x21;                                       // L0 = COM0, L1 = COM1
    LCDM1 = 0x84;                                       // L2 = COM2, L3 = COM3

    LCDCTL0 |= LCD4MUX | LCDON;                         // Turn on LCD, 4-mux selected (LCD4MUX also includes LCDSON)

    // Display start LCD value
    LCDMEM[pos1] = 0;
    LCDMEM[pos2] = digit[defaultLcdDigit];
    LCDMEM[pos3] = digit[defaultLcdDigit];
    LCDMEM[pos4] = digit[defaultLcdDigit];
    LCDMEM[pos5] = digit[defaultLcdDigit];
    LCDMEM[pos6] = 0;

    // Display  comma
    // LCDMEM[7] = 0x01;
    // LCDMEM[11] = 0x00;    
}

void ConfigureRTC(void)
{
    // Initialize RTC
    // RTC count re-load compare value at 32.
    // 1024/32768 * 31 = 1 sec.
    RTCMOD = 32-1;
    // Source = 32kHz crystal, divided by 1024
    RTCCTL = RTCSS__XT1CLK | RTCSR | RTCPS__1024 | RTCIE;
}

void SetXT1()
{
    do
    {
        CSCTL7 &= ~(XT1OFFG | DCOFFG);      // Clear XT1 and DCO fault flag
        SFRIFG1 &= ~OFIFG;
    }while (SFRIFG1 & OFIFG);               // Test oscillator fault flag
    CSCTL6 = (CSCTL6 & ~(XT1DRIVE_3)) | XT1DRIVE_2;     // Higher drive strength and current consumption for XT1 oscillator
}

void ConfigureTA0(void)
{
    // Configure Timer_A
    TA0CTL = TASSEL_0 | MC_2 | TACLR;         // TA0CLK, continiuous mode, clear TAR,

}

void ConfigureADC()
{
    // Configure ADC10
    ADCCTL0 |= ADCSHT_2 | ADCON;                             // ADCON, S&H=16 ADC clks
    ADCCTL1 |= ADCSHP;                                       // ADCCLK = MODOSC; sampling timer
    ADCCTL2 |= ADCRES;                                       // 10-bit conversion results
    ADCMCTL0 |= ADCINCH_9;                                   // A1 ADC input select; Vref=AVCC
    ADCIE |= ADCIE0;                                         // Enable ADC conv complete interrupt
}

void ConfigurePWM()
{
    // Timer1_A3 setup
    TA1CCR0 = 1000-1;                         // PWM Period 1ms
    TA1CCTL2 = OUTMOD_7;                      // CCR2 reset/set
    TA1CCR2 = 250;                            // CCR2 PWM duty cycle
    TA1CTL = TASSEL__SMCLK | MC__UP | TACLR;  // SMCLK, up mode, clear TAR
}

void SetDCO(void)
{
    __bis_SR_register(SCG0);                 // disable FLL
    CSCTL3 |= SELREF__REFOCLK;               // Set REFO as FLL reference source
    CSCTL0 = 0;                              // clear DCO and MOD registers
    CSCTL1 &= ~(DCORSEL_7);                  // Clear DCO frequency select bits first
    CSCTL1 |= DCORSEL_3;                     // Set DCO = 8MHz
    CSCTL2 = FLLD_0 + 243;                   // DCODIV = 8MHz
    __delay_cycles(3);
    __bic_SR_register(SCG0);                 // enable FLL

    while(CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1)); // Poll until FLL is locked

    CSCTL4 = SELMS__DCOCLKDIV | SELA__REFOCLK; // set default REFO(~32768Hz) as ACLK source, ACLK = 32768Hz
                                               // default DCODIV as MCLK and SMCLK source
}

void ConfigureUART(void)
{
    // Configure UART
    UCA0CTLW0 |= UCSWRST;
    UCA0CTLW0 |= UCSSEL__SMCLK;

    // Baud Rate calculation
    // 8000000/(16*9600) = 52.083
    // Fractional portion = 0.083
    // User's Guide Table 14-4: UCBRSx = 0x49
    // UCBRFx = int ( (52.083-52)*16) = 1
    UCA0BR0 = 52;                             // 8000000/16/9600
    UCA0BR1 = 0x00;
    UCA0MCTLW = 0x4900 | UCOS16 | UCBRF_1;

    UCA0CTLW0 &= ~UCSWRST;                    // Initialize eUSCI
    UCA0IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt
}

void InitBoard(void)
{
    WDTCTL = WDTPW | WDTHOLD; // Stop WDT
    Init_GPIO();
    SetGPIO();
    PM5CTL0 &= ~LOCKLPM5; //activate previously configured port settings
}

void InitClocks(void)
{
    SetDCO();
    SetXT1();
}

void InitPeriphery(void)
{
    ConfigureUART();
    ConfigurePWM();
    ConfigureADC();
    ConfigureTA0();
    ConfigureRTC();
    ConfigureLCD();
}

void calculateDigits(void)
{   
    // chenge ADC_result to speed for fan speed info 
    speedMatrix[0] = ADC_Result/1000 ;
    speedMatrix[1] = (ADC_Result - speedMatrix[0] * 1000)/100;
    speedMatrix[2] = (ADC_Result - speedMatrix[0] * 1000 - speedMatrix[1]*100)/10;
    speedMatrix[3] = (ADC_Result - speedMatrix[0] * 1000 - speedMatrix[1]*100 - speedMatrix[2]*10)/1;
}

void LCD_Display(void)
{
    LCDMEM[pos2] = digit[speedMatrix[0]];
    LCDMEM[pos3] = digit[speedMatrix[1]];
    LCDMEM[pos4] = digit[speedMatrix[2]];
    LCDMEM[pos5] = digit[speedMatrix[3]];
}

int main(void)
{
    //__bis_SR_register(GIE);       // Enter LPM3, enable interrupts
    InitBoard();
    InitClocks();
    InitPeriphery();

    while(1)
    {
        ADCCTL0 |= ADCENC | ADCSC;                           // Sampling and conversion start
        __bis_SR_register(LPM0_bits | GIE);                  // LPM0, ADC_ISR will force exit
        __no_operation();                                    // For debug only
        calculateDigits();
        LCD_Display();
        __delay_cycles(1000000);
    }
}

// RTC interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=RTC_VECTOR
__interrupt void RTC_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(RTC_VECTOR))) RTC_ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch(__even_in_range(RTCIV,RTCIV_RTCIF))
    {
        case  RTCIV_NONE:   break;          // No interrupt
        case  RTCIV_RTCIF:                  // RTC Overflow
            P4OUT ^= BIT0;
            speed = TA0R * 30;
            TA0CTL |= TACLR;
            break;
        default: break;
    }
}

// ADC interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=ADC_VECTOR
__interrupt void ADC_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(ADC_VECTOR))) ADC_ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch(__even_in_range(ADCIV,ADCIV_ADCIFG))
    {
        case ADCIV_NONE:
            break;
        case ADCIV_ADCOVIFG:
            break;
        case ADCIV_ADCTOVIFG:
            break;
        case ADCIV_ADCHIIFG:
            break;
        case ADCIV_ADCLOIFG:
            break;
        case ADCIV_ADCINIFG:
            break;
        case ADCIV_ADCIFG:
            ADC_Result = ADCMEM0;
            TA1CCR2 = (ADC_Result/330.0) * (800 - 1) + 200  ;  // set PWm duty 72/330 *999
            __bic_SR_register_on_exit(LPM0_bits);            // Clear CPUOFF bit from LPM0
            break;
        default:
            break;
    }
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A0_VECTOR))) USCI_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
  switch(__even_in_range(UCA0IV,USCI_UART_UCTXCPTIFG))
  {
    case USCI_NONE: break;
    case USCI_UART_UCRXIFG:
        uint8_t i = 0;   
        P4OUT ^= BIT0;                      // Toggle P1.0 using exclusive-OR
        UARTbuffor[i] = UCA0RXBUF;
        i++;

        if(i >= 3)
        {
            uint8_t k = 0;
            for(k = 0; k < 4; k++ )
            {
                while(!(UCA0IFG&UCTXIFG));      // wait kuntile buffor is empty
                UCA0TXBUF = speedMatrix[k] + 48;               //Send back ACK + LF / shift for ASCII
            }
            while(!(UCA0IFG&UCTXIFG));      // wait untile buffor is empty
            UCA0TXBUF = 0x0a;               //Send back LF

            i = 0;
        }

        __no_operation();
        break;
    case USCI_UART_UCTXIFG: break;
    case USCI_UART_UCSTTIFG: break;
    case USCI_UART_UCTXCPTIFG: break;
    default: break;
  }
}



