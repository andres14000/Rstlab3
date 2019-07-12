/*
 
 * CONTROL RST LAB1 2018 
 * RECHAZO DE DOS FRECUENCIAS 
 * ESSP=0
 * TRABAJA CON ARCHIVO DE MATLAB CONTROL MODERNO/ 2018/1CORTE/ LAB1/RST1  (DW)
 */
#include <p33FJ128MC802.h>
#define FOSC    (80000000ULL)
 #define FCY     (FOSC/2)
#include <libpic30.h>
#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include <math.h>
#include <string.h>
#include <float.h>
#include "C:\Program Files (x86)\Microchip\xc16\v1.26\include\uart.h"



// FBS
#pragma config BWRP = WRPROTECT_OFF     // Boot Segment Write Protect (Boot Segment may be written)
#pragma config BSS = NO_FLASH           // Boot Segment Program Flash Code Protection (No Boot program Flash segment)
#pragma config RBS = NO_RAM             // Boot Segment RAM Protection (No Boot RAM)

// FSS
#pragma config SWRP = WRPROTECT_OFF     // Secure Segment Program Write Protect (Secure segment may be written)
#pragma config SSS = NO_FLASH           // Secure Segment Program Flash Code Protection (No Secure Segment)
#pragma config RSS = NO_RAM             // Secure Segment Data RAM Protection (No Secure RAM)

// FGS
#pragma config GWRP = OFF               // General Code Segment Write Protect (User program memory is not write-protected)
#pragma config GSS = OFF                // General Segment Code Protection (User program memory is not code-protected)

// FOSCSEL
#pragma config FNOSC = PRIPLL           // Oscillator Mode (Primary Oscillator (XT, HS, EC) w/ PLL)
#pragma config IESO = ON                // Internal External Switch Over Mode (Start-up device with FRC, then automatically switch to user-selected oscillator source when ready)

// FOSC
#pragma config POSCMD = HS              // Primary Oscillator Source (HS Oscillator Mode)
#pragma config OSCIOFNC = OFF           // OSC2 Pin Function (OSC2 pin has clock out function)
#pragma config IOL1WAY = OFF            // Peripheral Pin Select Configuration (Allow Multiple Re-configurations)
#pragma config FCKSM = CSECMD           // Clock Switching and Monitor (Clock switching is enabled, Fail-Safe Clock Monitor is disabled)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler (1:32,768)
#pragma config WDTPRE = PR128           // WDT Prescaler (1:128)
#pragma config WINDIS = OFF             // Watchdog Timer Window (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (Watchdog timer enabled/disabled by user software)

// FPOR
#pragma config FPWRT = PWR1             // POR Timer Value (Disabled)
#pragma config ALTI2C = OFF             // Alternate I2C  pins (I2C mapped to SDA1/SCL1 pins)
#pragma config LPOL = OFF               // Motor Control PWM Low Side Polarity bit (PWM module low side output pins have active-low output polarity)
#pragma config HPOL = OFF               // Motor Control PWM High Side Polarity bit (PWM module high side output pins have active-low output polarity)
#pragma config PWMPIN = OFF             // Motor Control PWM Module Pin Mode bit (PWM module pins controlled by PWM module at device Reset)

// FICD
#pragma config ICS = PGD1               // Comm Channel Select (Communicate on PGC1/EMUC1 and PGD1/EMUD1)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG is Disabled)
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
//////////////////variables
 ////////////////////////////////////////////////////////////////////////////////////////
#define BAUDRATE 9600
#define BRGVAL ((FCY/BAUDRATE)/16)-1
#define DELAY_105uS asm volatile ("REPEAT, #4201"); Nop(); // 105uS delay
int pos1=0;
int vuelta=0;
int CU=0, i=0;
int CONTA=0;
unsigned int timePeriod= 0, periodo=0, T2=0;
float frec=0, rpm=0, time=0, vel, ref=0;

int tempRX=0, pwm;
 
float c1= 1.415990, c2= 1.846313, a=0.959437, b=0.000663, am=0.904837, bm=0.095163; 
float r0=1596.932188, r1=-7189.081775, r2=13291.542397, r3=-12675.190007, r4=6262.608054, r5=-1285.842645; 
float t0=1507.456942, t1=-6274.615437, t2=10849.258610, t3=-9974.282588, t4=5142.142666, t5=-1409.459158, t6=160.467177; 
float yf=0, y1=0, y2=0, y3=0, y4=0, y5=0;  
float rff, ref1, rff1, T, rff2, rff3, rff4, rff5, rff6;  
 float e, u, u1, u2, u3, u4, u5;




/////////////////////////////////////////////////////////////////////////////////////
 void  main(void) {
        
     PWM1CON1=0;
    PWM2CON1=0;
    I2C1CON=0;
    
    T1CON=0;
    T2CON=0;
    T3CON=0;
    T4CON=0;
    IC1CON=0;
    IC2CON=0;
    
    
     CLKDIVbits.PLLPRE=0XE;  //fin/16(n1) 
     CLKDIVbits.PLLPOST=0; // x/2(n2))
     PLLFBDbits.PLLDIV=0X9E; //*160 .. FOSC=80 PARA CRISTAL DE 16   
    
    
    
    
    //CONFIGURACION DE PUERTOS 
    TRISAbits.TRISA1=1;
    TRISAbits.TRISA2=1;
    TRISAbits.TRISA3=1;
    TRISB=0x3FF; //ENTRADA PORT B 0-9
        LATBbits.LATB15 = 0;
        LATBbits.LATB14 = 0;
        LATBbits.LATB13 = 0;
        LATBbits.LATB12 = 0;
        
        ///////////////////////////////////////////// Configure PWM 
    ///////////////////////////////////////
    P1TPER = 1998; // para 20khz
    P1DC1=0;   // ciclo util
    P1TCONbits.PTMOD = 0b00;   //for free running mode
    P1TCONbits.PTCKPS = 0b00;  //prescale=1:1
    P1TCONbits.PTOPS = 0b00;  // PWM time base input clock period is T_CY
    P1TCONbits.PTSIDL=1;
  //PWM1CON1  
    PWM1CON1bits.PMOD1 = 0; // complementario pwm1
    PWM1CON1bits.PMOD2 = 0; //complementario
    PWM1CON1bits.PMOD3 = 0;// complementario
    PWM1CON1bits.PEN1H = 1;// habilitar salida 1 
    PWM1CON1bits.PEN2H = 0;//deshabilitar
    PWM1CON1bits.PEN3H = 0;//deshabilitar
    PWM1CON1bits.PEN1L = 1;// habili
    PWM1CON1bits.PEN2L = 0;//
    PWM1CON1bits.PEN3L = 0;//
    
 // PWM1CON2 
    PWM1CON2bits.IUE = 0;
    PWM1CON2bits.OSYNC=0;
    PWM1CON2bits.UDIS=0;
    PWM1CON2bits.SEVOPS=0;
    
 //TIEMPO MUERTO
    P1DTCON1bits.DTA=0; // 
    P1DTCON1bits.DTAPS=10;
    
    P1FLTACON=0x0000;
    P1OVDCON=0x3F00;
    
    
       
P1TCONbits.PTEN = 1;        // Enable PWM time base 
   
   //////////////////////////////////////////////////////////////////////     
        //configuracion conversor
   //////////////////////////////////////////////////////////////////////////// 
        AD1CON1bits.ADSIDL=0; //CONTINUE MODE LDLE
        AD1CON1bits.ADDMABM=0; //DMA ESCRITO EN ORDEN DE CONVERSION
        AD1CON1bits.AD12B=1; // 12 BITS 
        AD1CON1bits.FORM=0; //SALIDA EN ENTEROS 
        AD1CON1bits.SSRC=0; // CONVERSION MANUAL USNDO SAMP
        AD1CON1bits.ASAM=0; // CONVERSION CON SAMP
        
        AD1CON2bits.VCFG=0; // REFERENCIA AVDD Y AVSS
        AD1CON2bits.CSCNA=0; //NO ESCANEAR ENTRADAS
        AD1CON2bits.BUFS=0; // LLENADO DEL BUFFER EN 0X0-0X7 Y ACCESO EN 0X8-0XF
        AD1CON2bits.SMPI=0 ; //LA DIRECCION DEL BUFFER AUMENTA DESUES DE COMPLETAR CADA MUESTREO  Y CONVERSION
        AD1CON2bits.BUFM=0; //EL BUFFER COMIENZA A LLENAR EN 0X0
        AD1CON2bits.ALTS=0; //SIEMPRE USA EL MISMO CANAL SELECCIONADO
        
        AD1CON3bits.ADRC=0; // RELOJ DE CONVERSION EN BASE AL RELOJ DEL SISTEMA
        AD1CON3bits.ADCS=0; // PERIODO DE CONVERSION TAD= TCY
            
        AD1CON4bits.DMABL=0; //  UN BUFFER DE UNA PALABRA  A CADA ENTRADA ANALOGA
        
        AD1CHS0bits.CH0NA=0; // channel 0  entrada negativa  es vref-
        AD1CHS0bits.CH0SA=0; // channel 0 entrada positiva es AN0
        
        AD1PCFGL=0x3E; //A0 COMO ANALOGICA
        
        //AD1CON1bits.ADON=1; // ENCENDER ADC
         
        
   /////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////    // Configure Remappables Pins
    __builtin_write_OSCCONL(OSCCON & ~(1<<6));
    RPINR14 = 0x605; //////////////////////////////////qea1 al rp5 y qeb1 al rp6
    RPINR19bits.U2RXR=0x07 ;  ///u2rx-- input al rp7 ----u2cts al rp0 
    RPOR4bits.RP8R = 0x05;   ///a rp8 se le asigna u2tx
    RPINR7bits.IC1R=9; // imput capture a rp9
    
    __builtin_write_OSCCONL(OSCCON | (1<<6));
     ///////////////////////////////////////////////////////////////////////////////////   
   ////////////////////////////////////////////////////////////////// 
    //Initialize QEI 1 Peripheral encoder 4x 
    QEI1CONbits.QEIM = 0; // Disable QEI Module
    QEI1CONbits.CNTERR = 0; // Clear any count errors
    QEI1CONbits.QEISIDL = 0; // Continue operation during sleep
    QEI1CONbits.SWPAB = 0; // QEA and QEB not swapped
    QEI1CONbits.PCDOUT = 0; // Normal I/O pin operation
    DFLT1CONbits.CEID = 1; // Count error interrupts disabled
    DFLT1CONbits.QEOUT = 0; // Digital filters output disabled for QEn pins
    POS1CNT = 0; // Reset position counter
    QEI1CONbits.QEIM = 7; // (x4 mode) with position counter reset by match (MAXxCNT)
    MAX1CNT=0xFFFF;
   ////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////7
    // Configure Timer 2: Para InputCapture1
    T2CON = 0;            // Timer reset
    IFS0bits.T2IF = 0;    // Reset Timer2 interrupt flag
    TMR2 = 0;             // Reset Timer 2 counter
    PR2 = 0xFFFF;              // periodo máximo en ticks: (max 65535)
    T2CONbits.TCKPS = 2;  // Prescaler (0=1:1, 1=1:8, 2=1:64, 3=1:256)
    ////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////7
    //////////////////////// Initialize the Input Capture Module
    //////////////////////////////////////////////////////////////////////////////////
    IC1CONbits.ICM = 0b00; // Disable Input Capture 1 module
    IC1CONbits.ICTMR = 1; // Select Timer2 as the IC1 Time base
    IC1CONbits.ICI = 1; // Interrupt on every second capture event
    IC1CONbits.ICM = 0b011; // Generate capture event on every Rising edge
    // Enable Capture Interrupt And Timer2
    IPC0bits.IC1IP = 5; // Setup IC1 interrupt priority level
    IFS0bits.IC1IF = 0; // Clear IC1 Interrupt Status Flag
    IEC0bits.IC1IE = 1; // Enable IC1 interrupt
    //
    T2CONbits.TON = 1;    // Turn on Timer 2
    
    ////////////////////////////////////// Configure Timer 1:
    T1CON = 0;            // Timer reset
    IFS0bits.T1IF = 0;    // Reset Timer1 interrupt flag
    IPC0bits.T1IP = 4;    // Timer1 Interrupt priority level=4
    IEC0bits.T1IE = 1;    // Enable Timer1 interrupt
    TMR1 = 0;             // Reset Timer 1 counter
    PR1 = 12500;          // Set the Timer 1 period (max 65535)
    T1CONbits.TCKPS = 2;  // Prescaler (0=1:1, 1=1:8, 2=1:64, 3=1:256) 1/fcy *pr1*prescaler= T
    T1CONbits.TON = 1;    // Turn on Timer 1
    //////////////////////////////////////////////////////////////////////////7
    //uart////////////////////////
    /////////////////////////////////////////////////////////////////////////////
     
     
    U2MODEbits.BRGH = 0; // Standard-Speed mode
    
    U2MODEbits.STSEL = 0; // 1-Stop bit
    U2MODEbits.PDSEL = 0; // No Parity, 8-Data bits
    U2MODEbits.ABAUD = 0; // Auto-Baud disabled
    U2MODEbits.RTSMD=0;  //FLOW CONTROL
    U2BRG = 42;
    U2STAbits.UTXISEL0 = 1; // Interrupt after one TX character is transmitted
    U2STAbits.UTXISEL1 = 0;
    U2STAbits.UTXISEL1 = 0;// Interrupt after one RX character iS RECIEVE
    
    U2STAbits.TRMT = 0;
    IEC1bits.U2TXIE = 1; // Enable UART TX interrupts
    IPC7bits.U2TXIP=4; // PRIORIDAD DE LA INTERRUPCION
    IFS1bits.U2TXIF = 0; // Clear TX Interrupt flag
    IEC1bits.U2RXIE = 1; // Enable UART RX interrupt
    IPC7bits.U2RXIP=5; // PRIORIDAD DE LA INTERRUPCION
    IFS1bits.U2RXIF = 0; // Clear TX Interrupt flag
    __C30_UART=2;
    U2MODEbits.UARTEN=1;
    U2STAbits.UTXEN=1;
    // Wait at least 105 microseconds (1/9600) before sending first char 
    __delay_ms(1);
    
    /////////////////////////////////////////////////////////////////////////
    while(1){
        

        
        }
        }
  

void __attribute__((__interrupt__,no_auto_psv)) _U2TXInterrupt(void)
{
IFS1bits.U2TXIF = 0; // Clear TX Interrupt flag

}
void __attribute__((__interrupt__,no_auto_psv)) _U2RXInterrupt(void)
{
    _U2RXIF = 0;
   tempRX = U2RXREG;
    switch (tempRX) {
        case 48 : ref=0;      //si recibe "0"
           break;
        case 49 : ref=1;    //si recibe "1"
           break;
        case 50 : ref=2;    //si recibe "2"
           break;
        case 51 : ref=3;    //si recibe "3"
           break;
        case 52 : ref=4;    //si recibe "4"
           break;
        case 53 : ref=5;   //si recibe "5"
           break;
        case 54 : ref=10;   //si recibe "6"
           break;
        case 55 : ref=15;   //si recibe "7"
           break;
        case 56 : ref=25;   //si recibe "8"
           break;
        default : u=0;      //si recibe otra cosa
           break;
     }
    
}
// Capture Interrupt Service Routine
void __attribute__((__interrupt__)) _IC1Interrupt(void)
{
    
    unsigned int t1,t2;
    t1=IC1BUF;
    t2=IC1BUF;
    IFS0bits.IC1IF=0;
    if(t2>t1)
    timePeriod = t2-t1;
    else
    timePeriod = (PR2 - t1) + t2;
    if (timePeriod<=0) vel=0;
    
    else vel=(3125.0/timePeriod);
    
}               
void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void)
{
    IFS0bits.T1IF = 0; 
    LATBbits.LATB13=~LATBbits.LATB13;
    yf=(r0*vel)+(r1*y1)+(r2*y2)+(r3*y3)+(r4*y4)+(r5*y5);
    rff=(bm*ref1)+(am*rff1);
    T=(t0*rff)+(t1*rff1)+(t2*rff2)+(t3*rff3)+(t4*rff4)+(t5*rff5)+(t6*rff6);
    e=T-yf;
    u=e+(c1+c2+1)*u1-(c1*c2+c1+c2+2)*u2+(c1*c2+c1+c2+2)*u3-(c1+c2+1)*u4+u5;

    if(u>4000){
            u=4000;
        }
        if(u<-4000){
            u=-4000;
        }
        if(u>=0){
            //LATBbits.LATB10 = 0;
            pwm=u;
            P1DC1=pwm;
            LATBbits.LATB10 = 1;

        }

        if(u<0){
            //LATBbits.LATB11 = 0;
            pwm=u;
            P1DC1=-pwm;
            LATBbits.LATB10 = 0;
        }
        

     ref1=ref;
    
    rff6=rff5; 
    rff5=rff4;
     rff4=rff3;
     rff3=rff2;
     rff2=rff1;
     rff1=rff;
     
     y5=y4;
     y4=y3;
     y3=y2;
     y2=y1;
     y1=vel;
     
     u5=u4;
     u4=u3;
     u3=u2;
     u2=u1;
     u1=u;

printf("%f, %4.3f, %4.3f, %d \n",ref,vel,e,pwm);
//printf("%f, %f\r\n",vel,ref);



    
}
