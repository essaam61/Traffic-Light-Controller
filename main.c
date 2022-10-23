#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "inc/hw_ints.h"


#define RedEw (1U << 5)   //Port B
#define YellowEw (1U << 4)
#define GreenEw (1U << 3)


#define RedNs (1U << 2)   //Port B
#define YellowNs (1U << 1)
#define GreenNs (1U << 0)



#define RedPdNs (1U << 2)   //Port A
#define RedPdEw (1U << 5)
#define GreenPdNs (1U << 3)
#define GreenPdEw (1U << 6)


#define sw1 (1U)    //Port F
#define sw2 (1U<<4)



#define tgNs (5*1000)
#define tgEw (5*1000)
#define tyNs (2*1000)
#define tyEw (2*1000)
#define tNs (1000)
#define tEw (1000)
#define tcross (2000)


int NsGreenFlag=0;
int PdTimeFlag=0;
int NsYellowFlag=0;
int EwGreenFlag=0;
int EwYellowFlag=0;
int GNs = 0;
int GEw = 0;
int StartNs = 0;
int StartEw = 0;
int SNs = 0;
int SEw = 0;
int YNs = 0;
int YEw = 0;
int Ped = 0;
int StartNext = 0;
int StartNext2 = 0;

/* Interrupt handler for car timer */
void Timer1_Handler(void)
{
    uint32_t status = 0;
    status = TimerIntStatus(TIMER5_BASE,true);
    TimerIntClear(TIMER5_BASE, status);

    if(Ped==1)
    {
        PdTimeFlag = 1;
        Ped = 0;
    }

    if(GNs==1)
    {
        NsGreenFlag = 1;
        GNs = 0;
    }

    if(YNs==1)
    {
        NsYellowFlag = 1;
        YNs = 0;
    }

    if(GEw==1)
    {
        EwGreenFlag = 1;
        GEw = 0;
    }

    if(YEw==1)
    {
        EwYellowFlag = 1;
        YEw = 0;
    }

    if(GEw==1)
    {
        EwGreenFlag = 1;
        GEw = 0;
    }


    if(SEw==1)
    {
        StartEw = 1;
        SEw = 0;
    }

        if(GEw==1)
    {
        EwGreenFlag = 1;
        GEw = 0;
    }

    if(SNs==1)
    {
        StartNs = 1;
        SNs = 0;
    }

    if(StartNext2==1) {
        EwFunc();
        StartNext2=0;
    }

    if(StartNext==1) {
        NsFunc();
        StartNext=0;
    }

    TimerDisable(TIMER5_BASE, TIMER_A);


}


/* Interrupt handler for Ped timer */
void Timer2_Handler(void)
{
    uint32_t status = 0;
    status = TimerIntStatus(TIMER0_BASE,true);
    TimerIntClear(TIMER0_BASE, status);

    if(Ped==1) {
        PdTimeFlag=1;
        Ped = 0;
    }

    TimerDisable(TIMER0_BASE, TIMER_A);

}


/* Car Timer Setup */
void CarTimerBegin(int time)
{
    //we set the load value so the timer interrupts each 1ms
    uint32_t Period;
    Period = 80000 * time;
    if(time==tgNs)
        GNs=1;
    if(time==tyNs)
        YNs=1;
    if(time==tgEw)
        GEw=1;
    if(time==tyEw)
        YEw=1;
    if(time==tNs)
        SNs=1;
    if(time==tEw)
        SEw=1;


    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    SysCtlDelay(3);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, Period-1);
    TimerIntRegister(TIMER0_BASE, TIMER_A, Timer1_Handler);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);


    TimerEnable(TIMER0_BASE, TIMER_A);

}


/* Pedestrian Timer Setup */
void PedTimerBegin(int time)
{
    //we set the load value so the timer interrupts each 1ms
    uint32_t Period;
    Period = 80000 * time; //1ms
    if(time==tcross)
        Ped=1;


    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    SysCtlDelay(3);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, Period-1);
    TimerIntRegister(TIMER0_BASE, TIMER_A, Timer2_Handler);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);


    TimerEnable(TIMER0_BASE, TIMER_A);

}


void EW_NS_Init(){

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIO_PORTB_DEN_R |= (RedEw | GreenEw | YellowEw | RedNs | YellowNs | GreenNs);
    GPIO_PORTB_DIR_R |= (RedEw | GreenEw | YellowEw | RedNs | YellowNs | GreenNs);
    GPIO_PORTB_DATA_R = 0;
}

void Ped_Init(){

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIO_PORTA_DEN_R |= (RedPdEw | GreenPdEw | RedPdNs | GreenPdNs);
    GPIO_PORTA_DIR_R |= (RedPdEw | GreenPdEw | RedPdNs | GreenPdNs);
    GPIO_PORTA_DATA_R = 0;
}

void PortF_Init(){
    SYSCTL_RCGCGPIO_R |= 0x20;          /* Enable PORTF. */
    GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY;  /* Unlock PORTF. */
    GPIO_PORTF_CR_R  |= 0x1F;           /* Enable writing to PUR. */
    GPIO_PORTF_DIR_R &= (~(sw1 | sw2));        /* Set PF0, PF4 as i/p. */
    GPIO_PORTF_PUR_R |= (sw1 | sw2);           /* Enable pull up resistors PF0, PF4. */
    GPIO_PORTF_DEN_R |= (sw1 | sw2);           /* Enable pins PF0, PF4. */
}

void Timer0_init() {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    SysCtlDelay(3);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, Period-1);
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntRegister(TIMER0_BASE, TIMER_A, Timer0_Handler);
}


void NsFunc(){
    GPIOPinWrite(GPIO_PORTB_BASE, RedNs, 0);
    GPIOPinWrite(GPIO_PORTB_BASE, YellowNs, 0);
    GPIOPinWrite(GPIO_PORTA_BASE, RedPdNs, RedPdNs);
    GPIOPinWrite(GPIO_PORTA_BASE, RedPdEw, RedPdEw);
    GPIOPinWrite(GPIO_PORTB_BASE, RedEw, RedEw);
    GPIOPinWrite(GPIO_PORTB_BASE, GreenNs, GreenNs);

    CarTimerBegin(tgNs);
    while(NsGreenFlag==0);
    GPIOPinWrite(GPIO_PORTB_BASE, GreenNs, 0);
    GPIOPinWrite(GPIO_PORTB_BASE, YellowNs, YellowNs);

    CarTimerBegin(tyNs);
    NsGreenFlag = 0;
    while(NsYellowFlag==0);
    GPIOPinWrite(GPIO_PORTB_BASE, YellowNs, 0);
    GPIOPinWrite(GPIO_PORTB_BASE, RedNs, RedNs);

    CarTimerBegin(tEw);
    NsYellowFlag = 0;
    while(StartEw==0);

    StartNext2 = 1;

}


void EwFunc(){
    GPIOPinWrite(GPIO_PORTB_BASE, RedEw, 0);
    GPIOPinWrite(GPIO_PORTB_BASE, YellowEw, 0);
    GPIOPinWrite(GPIO_PORTA_BASE, RedPdNs, RedPdNs);
    GPIOPinWrite(GPIO_PORTA_BASE, RedPdEw, RedPdEw);
    GPIOPinWrite(GPIO_PORTB_BASE, RedNs, RedNs);
    GPIOPinWrite(GPIO_PORTB_BASE, GreenEw, GreenEw);


    CarTimerBegin(tgEw);
    while(EwGreenFlag==0);
    GPIOPinWrite(GPIO_PORTB_BASE, GreenEw, 0);
    GPIOPinWrite(GPIO_PORTB_BASE, YellowEw, YellowEw);

    CarTimerBegin(tyEw);
    EwGreenFlag = 0;
    while(EwYellowFlag==0);
    GPIOPinWrite(GPIO_PORTB_BASE, YellowEw, 0);
    GPIOPinWrite(GPIO_PORTB_BASE, RedEw, RedEw);

    CarTimerBegin(tNs);
    EwYellowFlag = 0;
    while(StartNs==0);

    StartNext = 1;

}


void PedHandler(){

    if( (GPIO_PORTF_DATA_R & sw1) == 0)
    {
        GPIOPinWrite(GPIO_PORTB_BASE, RedNs, RedNs);
        GPIOPinWrite(GPIO_PORTB_BASE, YellowNs, 0);
        GPIOPinWrite(GPIO_PORTB_BASE, GreenNs, 0);
        GPIOPinWrite(GPIO_PORTA_BASE, RedPdNs, 0);
        GPIOPinWrite(GPIO_PORTA_BASE, GreenPdNs, GreenPdNs);

        PedTimerBegin(tcross);
        while(PdTimeFlag == 0);
        GPIOPinWrite(GPIO_PORTA_BASE, GreenPdNs, 0);
        GPIOPinWrite(GPIO_PORTA_BASE, RedPdNs, RedPdNs);

        PdTimeFlag = 0;

    }


    if( (GPIO_PORTF_DATA_R & sw2) == 0)
    {
        GPIOPinWrite(GPIO_PORTB_BASE, RedEw, RedEw);
        GPIOPinWrite(GPIO_PORTB_BASE, YellowEw, 0);
        GPIOPinWrite(GPIO_PORTB_BASE, GreenEw, 0);
        GPIOPinWrite(GPIO_PORTA_BASE, RedPdEw, 0);
        GPIOPinWrite(GPIO_PORTA_BASE, GreenPdEw, GreenPdEw);

        PedTimerBegin(tcross);
        while(PdTimeFlag == 0);
        GPIOPinWrite(GPIO_PORTA_BASE, GreenPdEw, 0);
        GPIOPinWrite(GPIO_PORTA_BASE, RedPdEw, RedPdEw);

        PdTimeFlag = 0;

    }

}



int main(void){
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    EW_NS_Init();
    Ped_Init();
    PortF_Init();
    EwFunc();

    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_INT_PIN_0 | GPIO_INT_PIN_4);
    GPIOIntRegister(GPIO_PORTF_BASE, PedHandler);
    GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_FALLING_EDGE);

    while(1)
    {

    }

}

