//**************************************************************************************************************
/**
 * main.c
 */
//**************************************************************************************************************
// Librerias
//**************************************************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "driverlib/adc.h"
#include "pinout.h"

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

#define XTAL 16000000

//**************************************************************************************************************
// Variables Globales
//**************************************************************************************************************
int estado;
uint8_t charIn = '0';
uint8_t char0 = '0';
int contador1 = '0';
int state, pstate;
int state2, pstate2;
uint32_t pui32ADC0Value[1];
uint32_t contmap;
unsigned char SEG[] = {0b0111111, 0b0000110, 0b1011011, 0b1001111, 0b01100110, 0b1101101, 0b1111101, 0b0000111, 0b1111111, 0b1101111, 0b1110111, 0b1111100, 0b0111001, 0b01011110, 0b1111001, 0b1110001};
unsigned char ANIME[] = {0b11101101, 0b10000110, 0b00111111, 0b11011011, 0b00111111, 0b00111111, 0b00000110};
int banderatmr = 0;
int banderaan = 0;
int ledanim = 0;
uint32_t ui32Period;
uint32_t i = 0;
char env;
int led_1 = 0;
int led_2 = 0;
int led_3 = 0;
int led_4 = 0;
int dis = 0;
int decenas = 0;

//**************************************************************************************************************
// Prototipos de Función
//**************************************************************************************************************
void InitConsole(void);
void Timer0IntHandler(void);
unsigned short map(uint32_t val, uint32_t in_min, uint32_t in_max,
            unsigned short out_min, unsigned short out_max);

//**************************************************************************************************************
// Código Principal
//**************************************************************************************************************
int main(void)
{
SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                  SYSCTL_XTAL_16MHZ);
PinoutSet();

//ADC
SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
            ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE |
                                         ADC_CTL_END);
            ADCSequenceEnable(ADC0_BASE, 3);
            ADCIntClear(ADC0_BASE, 3);

//UART
            InitConsole();

                // Display the setup on the console.
                UARTprintf("HOLAAAA\n\n");

// Timer

                // Se habilita el reloj para el temporizador
                SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

                // Configuración del Timer 0 como temporizador per�odico
                TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

                // Se calcula el período para el temporizador (1 seg)
                ui32Period = (SysCtlClockGet()) / 2;
                // Establecer el periodo del temporizador
                TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period - 1);

                // Se habilita la interrupción por el TIMER0A
                IntEnable(INT_TIMER0A);
                // Se establece que exista la interrupción por Timeout
                TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
                // Se habilitan las interrupciones Globales
                IntMasterEnable();
                // Se habilita el Timer
                TimerEnable(TIMER0_BASE, TIMER_A);
                GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3| GPIO_PIN_4| GPIO_PIN_5| GPIO_PIN_6| GPIO_PIN_7);



    //**********************************************************************************************************
    // Loop Principal
    //**********************************************************************************************************
    while (1)
    {

    estado = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_6);

    state = GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_0);
    state2 = GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_2);


    switch(estado){
        case 0:
            //GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0);
            env = UARTCharGetNonBlocking(UART0_BASE);    //UART

                               if (env == 'h'){           // on blue
                                   if (led_1 == 0){
                                       GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_PIN_3);
                                       led_1++;
                                   }
                                   else if (led_1 == 1){ // off blue
                                       GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0);
                                       led_1 = 0;
                                   }
                               }

                               if (env == 'o'){
                                   if (led_2 == 0){
                                       GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2);
                                       led_2++;
                                   }
                                   else if (led_2 == 1){
                                       GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0);
                                       led_2 = 0;
                                   }
                               }
                               if (env == 'l'){
                                  if (led_3 == 0){
                                       GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1);
                                       led_3++;
                                  }
                                  else if (led_3 == 1){
                                       GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 0);
                                       led_3 = 0;
                                  }
                               }

                               if (env == 'a'){
                                   if (led_4 == 0){
                                       GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0);
                                       led_4++;
                                   }
                                   else if (led_4 == 1){
                                        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 0);
                                        led_4 = 0;
                                   }
                               }

                               if (env == '0'){
                                   GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3| GPIO_PIN_4| GPIO_PIN_5| GPIO_PIN_6 | GPIO_PIN_7, 0b0111111);
                                   dis = 0;
                               }

                               if (env == '1'){
                                   GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3| GPIO_PIN_4| GPIO_PIN_5| GPIO_PIN_6 | GPIO_PIN_7, 0b0000110);
                                   dis = 1;
                               }

                               if (env == '2'){
                                   GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3| GPIO_PIN_4| GPIO_PIN_5| GPIO_PIN_6 | GPIO_PIN_7, 0b1011011);
                                   dis = 2;
                               }
                               if (env == '3'){
                                   GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3| GPIO_PIN_4| GPIO_PIN_5| GPIO_PIN_6 | GPIO_PIN_7, 0b1001111);
                                   dis = 3;
                               }
                               if (env == '4'){
                                   GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3| GPIO_PIN_4| GPIO_PIN_5| GPIO_PIN_6 | GPIO_PIN_7, 0b01100110);
                                   dis = 4;
                               }
                               if (env == '5'){
                                   GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3| GPIO_PIN_4| GPIO_PIN_5| GPIO_PIN_6 | GPIO_PIN_7, 0b1101101);
                                   dis = 5;
                               }
                               if (env == '6'){
                                   GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3| GPIO_PIN_4| GPIO_PIN_5| GPIO_PIN_6 | GPIO_PIN_7, 0b1111101);
                                   dis = 6;
                               }
                               if (env == '7'){
                                   GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3| GPIO_PIN_4| GPIO_PIN_5| GPIO_PIN_6 | GPIO_PIN_7, 0b0000111);
                                   dis = 7;
                               }
                               if (env == '8'){
                                   GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3| GPIO_PIN_4| GPIO_PIN_5| GPIO_PIN_6 | GPIO_PIN_7, 0b1111111);
                                   dis = 8;
                               }
                               if (env == '9'){
                                   GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3| GPIO_PIN_4| GPIO_PIN_5| GPIO_PIN_6 | GPIO_PIN_7, 0b1101111);
                                   dis = 9;
                               }

                               if(state && pstate== 0){             // print con UART
                                                      UARTprintf("LED 1: %d\n", led_1);
                                                      UARTprintf("LED 2: %d\n", led_2);
                                                      UARTprintf("LED 3: %d\n", led_3);
                                                      UARTprintf("LED 4: %d\n", led_4);
                                                      UARTprintf("7SEG: %d\n\n", dis);
                                                  }
                                                  pstate = state;

                                                  if(state2 && pstate2== 0){             // print con UART
                                                      UARTprintf("LED 1: %d\n", led_1);
                                                      UARTprintf("LED 2: %d\n", led_2);
                                                      UARTprintf("LED 3: %d\n", led_3);
                                                      UARTprintf("LED 4: %d\n", led_4);
                                                      UARTprintf("7SEG: %d\n\n", dis);
                                                 }
                                                 pstate2 = state2;

            break;

        case 0x20:
            //GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1);


                            if(state != pstate){
                                if(state == 0){
                                contador1++;
                                }
                                pstate = state;
                            }

                            if(state2 != pstate2){
                                if(state2 == 0){
                                    contador1--;
                                }
                                pstate2 = state2;
                            }

                            GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1| GPIO_PIN_2| GPIO_PIN_3, contador1);

                            if(contador1 > 16){ //2^4 LEDS
                                contador1 = 0;
                            }

                            else if(contador1 < 0){
                                contador1 = 16;
                            }

            break;

        case 0x40:
            //GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2);
            if(ledanim > 10){
                ledanim = 0;
            }
            SysCtlDelay((uint32_t)(SysCtlClockGet() / 12));
            ledanim++;

            if(ledanim == 5){
                GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1| GPIO_PIN_2| GPIO_PIN_3, 0x9);
            }
            else if(ledanim == 10){
                GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1| GPIO_PIN_2| GPIO_PIN_3, 0x6);
            }

            break;

        case 0x60:
            //GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_PIN_3);

            //ADC
            ADCProcessorTrigger(ADC0_BASE, 3);
            while(!ADCIntStatus(ADC0_BASE, 3, false))
                    {
                    }

            ADCIntClear(ADC0_BASE, 3);
            ADCSequenceDataGet(ADC0_BASE, 3, pui32ADC0Value);

            //GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, pui32ADC0Value[0]);

            contmap = map(pui32ADC0Value[0], 0, 4095, 0, 99);

            UARTprintf("AIN0 = %4d\r", contmap);


            decenas = (contmap%100 - contmap%10)/10;

            GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1| GPIO_PIN_2| GPIO_PIN_3, decenas);


            break;

        default:

            break;

    }
    }
}
//**************************************************************************************************************
// Inicialización de UART
//**************************************************************************************************************

void
InitConsole(void)
{

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTStdioConfig(0, 9600, 16000000);
}

unsigned short map(uint32_t x, uint32_t x0, uint32_t x1,
                   unsigned short y0, unsigned short y1){
    return (unsigned short)(y0+((float)(y1-y0)/(x1-x0))*(x-x0));
}

void Timer0IntHandler(void)
{


    // Clear the timer interrupt
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    if(estado == 0x20){

        UARTprintf("LEDS: %d\n\n", contador1);
        UARTprintf("7SEG: %d\n", banderatmr);

        if(banderatmr > 15) // DE 0 A F
            banderatmr = 0;

        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3| GPIO_PIN_4| GPIO_PIN_5| GPIO_PIN_6 | GPIO_PIN_7, SEG[banderatmr]);
        banderatmr++;


    }

    if (estado == 0x40){

        if(banderaan > 7) //LARGO DE LA ANIMACION
            banderaan = 0;

        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3| GPIO_PIN_4| GPIO_PIN_5| GPIO_PIN_6 | GPIO_PIN_7 , ANIME[banderaan]);
        SysCtlDelay((uint32_t)(SysCtlClockGet() / 12));
        banderaan++;


    }



}
