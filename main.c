#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"



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

#define sw2 (1U)    //Port F
#define sw1 (1U<<4)


#define tg  (5000)
#define ty  (2000)
#define tr  (1000)
#define tcross (2000)

#define tgNs 0
#define tgEw 1
#define tyNs 2
#define tyEw 3
#define trNs 4
#define trEw 5


int NsGreenFlag=0;
int NsYellowFlag=0;
int NsRedFlag = 0;
int EwGreenFlag=0;
int EwYellowFlag=0;
int EwRedFlag = 0;
//int PdTimeFlag=0;

int GNs = 0;
int GEw = 0;
int YNs = 0;
int YEw = 0;
int RNs = 0;
int REw = 0;

//uint8_t Ped = 0;
uint8_t PedEw = 0;
uint8_t PedNs = 0;
uint8_t PdHandler_Flag=0;


uint32_t counter = 0;
uint32_t counter1 = 0;
uint32_t counter2 = 0;

uint8_t RedNs_Value=0;
uint8_t YellowNs_Value=0;
uint8_t GreenNs_Value=0;
uint8_t RedEw_Value=0;
uint8_t YellowEw_Value=0;
uint8_t GreenEw_Value=0;


void EW_NS_Init(void);
void Ped_Init(void);
void PortF_Init(void);
void Timer0_Init(void);
//void Timer1_Init(void);
//void SysTick_Init(void);

void CarTimerBegin(int time);
//void PedTimerBegin(int time);
void EWCars_TrafficLight(void);
void NSCars_TrafficLight(void);



/* Interrupt handler for traffic timer */
void Timer0_Handler(void)
{
    //uint32_t status = 0;
    //status = TimerIntStatus(TIMER0_BASE,true);
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);


    if(PdHandler_Flag==0)
    {
        counter++;



    if(GNs==1 && counter==tg)
    {
        NsGreenFlag = 1;
        GNs = 0;
        counter=0;
        TimerDisable(TIMER0_BASE, TIMER_A);
    }

    else if(YNs==1 && counter==ty)
    {
        NsYellowFlag = 1;
        YNs = 0;
        counter=0;
        TimerDisable(TIMER0_BASE, TIMER_A);
    }

    else if(RNs==1 && counter==tr)
    {
        NsRedFlag = 1;
        RNs = 0;
        counter=0;
        TimerDisable(TIMER0_BASE, TIMER_A);
    }

    else if(GEw==1 && counter==tg)
    {
        EwGreenFlag = 1;
        GEw = 0;
        counter=0;
        TimerDisable(TIMER0_BASE, TIMER_A);
    }

    else if(YEw==1 && counter==ty)
    {
        EwYellowFlag = 1;
        YEw = 0;
        counter=0;
        TimerDisable(TIMER0_BASE, TIMER_A);
    }


    else if(REw==1 && counter==tr)
    {
        EwRedFlag = 1;
        REw = 0;
        counter=0;
        TimerDisable(TIMER0_BASE, TIMER_A);
    }

    }


    else {

        if(PedEw==1)
        {
            counter1++;
            if(counter1==tcross) {

                PedEw=0;
                counter1=0;
                GPIOPinWrite(GPIO_PORTA_BASE, GreenPdEw, 0);
                GPIOPinWrite(GPIO_PORTA_BASE, RedPdEw, RedPdEw);

                GPIOPinWrite(GPIO_PORTB_BASE, RedEw_Value, RedEw_Value);
                GPIOPinWrite(GPIO_PORTB_BASE, YellowEw_Value, YellowEw_Value);
                GPIOPinWrite(GPIO_PORTB_BASE, GreenEw_Value, GreenEw_Value);
                if(PedNs==0)
                    PdHandler_Flag=0;
            }
        }

        if(PedNs==1) {
            counter2++;
            if(counter2==tcross) {

                PedNs=0;
                counter2=0;
                GPIOPinWrite(GPIO_PORTA_BASE, GreenPdNs, 0);
                GPIOPinWrite(GPIO_PORTA_BASE, RedPdNs, RedPdNs);


                GPIOPinWrite(GPIO_PORTB_BASE, RedNs_Value, RedNs_Value);
                GPIOPinWrite(GPIO_PORTB_BASE, YellowNs_Value, YellowNs_Value);
                GPIOPinWrite(GPIO_PORTB_BASE, GreenNs_Value, GreenNs_Value);
                if(PedEw==0)
                    PdHandler_Flag=0;
            }
        }


    }//end of else


}


/* Car Timer Setup */
void CarTimerBegin(int time)
{

    if(time==tgNs)
        GNs=1;
    else if(time==tyNs)
        YNs=1;
    else if(time==tgEw)
        GEw=1;
    else if(time==tyEw)
        YEw=1;
    else if(time==trNs)
        RNs=1;
    else if(time==trEw)
        REw=1;


    TimerEnable(TIMER0_BASE, TIMER_A);

}


void EWCars_TrafficLight(){
    GPIOPinWrite(GPIO_PORTB_BASE, RedEw, 0);
    GPIOPinWrite(GPIO_PORTB_BASE, YellowEw, 0);
    GPIOPinWrite(GPIO_PORTA_BASE, RedPdEw, RedPdEw);
    GPIOPinWrite(GPIO_PORTA_BASE, RedPdNs, RedPdNs);
    GPIOPinWrite(GPIO_PORTB_BASE, RedNs, RedNs);
    GPIOPinWrite(GPIO_PORTB_BASE, GreenEw, GreenEw);

    CarTimerBegin(tgEw);
    while(EwGreenFlag==0);
    EwGreenFlag = 0;

    GPIOPinWrite(GPIO_PORTB_BASE, GreenEw, 0);
    GPIOPinWrite(GPIO_PORTB_BASE, YellowEw, YellowEw);
    CarTimerBegin(tyEw);
    while(EwYellowFlag==0);
    EwYellowFlag = 0;

    GPIOPinWrite(GPIO_PORTB_BASE, YellowEw, 0);
    GPIOPinWrite(GPIO_PORTB_BASE, RedEw, RedEw);
    CarTimerBegin(trEw);
    while(EwRedFlag==0);
    EwRedFlag=0;


}



void NSCars_TrafficLight(void){
    GPIOPinWrite(GPIO_PORTB_BASE, RedNs, 0);
    GPIOPinWrite(GPIO_PORTB_BASE, YellowNs, 0);
    GPIOPinWrite(GPIO_PORTA_BASE, RedPdNs, RedPdNs);
    GPIOPinWrite(GPIO_PORTA_BASE, RedPdEw, RedPdEw);
    GPIOPinWrite(GPIO_PORTB_BASE, RedEw, RedEw);
    GPIOPinWrite(GPIO_PORTB_BASE, GreenNs, GreenNs);

    CarTimerBegin(tgNs);
    while(NsGreenFlag==0);
    NsGreenFlag = 0;

    GPIOPinWrite(GPIO_PORTB_BASE, GreenNs, 0);
    GPIOPinWrite(GPIO_PORTB_BASE, YellowNs, YellowNs);
    CarTimerBegin(tyNs);
    while(NsYellowFlag==0);
    NsYellowFlag = 0;

    GPIOPinWrite(GPIO_PORTB_BASE, YellowNs, 0);
    GPIOPinWrite(GPIO_PORTB_BASE, RedNs, RedNs);
    CarTimerBegin(trNs);
    while(NsRedFlag==0);
    NsRedFlag = 0;


}


void PedTraffic(){

    // Clear the asserted interrupts.
    GPIOIntClear(GPIO_PORTF_BASE, GPIO_INT_PIN_0 | GPIO_INT_PIN_4);
    PdHandler_Flag=1;

    if( (GPIO_PORTF_DATA_R & sw2) == 0)
    {
        RedNs_Value = GPIOPinRead(GPIO_PORTB_BASE,RedNs);
        YellowNs_Value = GPIOPinRead(GPIO_PORTB_BASE,YellowNs);
        GreenNs_Value = GPIOPinRead(GPIO_PORTB_BASE,GreenNs);

        GPIOPinWrite(GPIO_PORTB_BASE, RedNs, RedNs);
        GPIOPinWrite(GPIO_PORTB_BASE, YellowNs, 0);
        GPIOPinWrite(GPIO_PORTB_BASE, GreenNs, 0);
        GPIOPinWrite(GPIO_PORTA_BASE, RedPdNs, 0);
        GPIOPinWrite(GPIO_PORTA_BASE, GreenPdNs, GreenPdNs);

        PedNs=1;


        /*
        PedTimerBegin(tcross);
        while(PdTimeFlag == 0);
        PdTimeFlag = 0;

        GPIOPinWrite(GPIO_PORTA_BASE, GreenPdNs, 0);
        GPIOPinWrite(GPIO_PORTA_BASE, RedPdNs, RedPdNs);
        */
    }


    else if( (GPIO_PORTF_DATA_R & sw1) == 0)
    {
        RedEw_Value = GPIOPinRead(GPIO_PORTB_BASE,RedEw);
        YellowEw_Value = GPIOPinRead(GPIO_PORTB_BASE,YellowEw);
        GreenEw_Value = GPIOPinRead(GPIO_PORTB_BASE,GreenEw);

        GPIOPinWrite(GPIO_PORTB_BASE, RedEw, RedEw);
        GPIOPinWrite(GPIO_PORTB_BASE, YellowEw, 0);
        GPIOPinWrite(GPIO_PORTB_BASE, GreenEw, 0);
        GPIOPinWrite(GPIO_PORTA_BASE, RedPdEw, 0);
        GPIOPinWrite(GPIO_PORTA_BASE, GreenPdEw, GreenPdEw);

        PedEw=1;


        /*
        PedTimerBegin(tcross);
        while(PdTimeFlag == 0);
        PdTimeFlag = 0;

        GPIOPinWrite(GPIO_PORTA_BASE, GreenPdEw, 0);
        GPIOPinWrite(GPIO_PORTA_BASE, RedPdEw, RedPdEw);
        */
    }


}



int main(void){

    EW_NS_Init();
    Ped_Init();
    PortF_Init();
    Timer0_Init();
    //Timer1_Init();
    //SysTick_Init();

    while(1)
    {
        EWCars_TrafficLight();
        NSCars_TrafficLight();

    }

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
    GPIO_PORTF_DIR_R |= 0x0E;
    GPIO_PORTF_DEN_R |= 0x0E;

    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_INT_PIN_0 | GPIO_INT_PIN_4);
    GPIOIntRegister(GPIO_PORTF_BASE, PedTraffic);
    GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_FALLING_EDGE);

}

void Timer0_Init() {
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC);
    //we set the load value so the timer interrupts each 1ms
    TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet() / 1000);


    IntMasterEnable();          // Enable processor interrupts.
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    IntEnable(INT_TIMER0A);
    TimerIntRegister(TIMER0_BASE, TIMER_A, Timer0_Handler);
}





/*
void Timer1_Init() {
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    TimerConfigure(TIMER1_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC);
    //we set the load value so the timer interrupts each 1ms
    TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet() / 1000);

    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    IntEnable(INT_TIMER1A);
    TimerIntRegister(TIMER1_BASE, TIMER_A, Timer1_Handler);

}

void SysTick_Init() {
    SysTickDisable();
    SysTickPeriodSet(SysCtlClockGet());        //  Sets the period of the SysTick counter.
    SysTickIntEnable();         //  Enable the SysTick Interrupt.
    SysTickIntRegister(SysTick_Handler);  //  Registers an interrupt handler for the SysTick interrupt

}
*/




/* Interrupt handler for Ped timer */
/*
void Timer1_Handler(void)
{
    //uint32_t status = 0;
    //status = TimerIntStatus(TIMER1_BASE,true);
    TimerIntClear(TIMER0_BASE, TIMER_TIMB_TIMEOUT);

    GPIOPinWrite(GPIO_PORTF_BASE, 2, 2);

    counter1++;

    if(Ped==1 && counter1==tcross) {
        PdTimeFlag=1;
        Ped = 0;
        counter1=0;
        TimerDisable(TIMER0_BASE, TIMER_B);
    }

}

void SysTick_Handler(void)
{

    counter1++;

    if(Ped==1 && counter1==2) {
        PdTimeFlag=1;
        Ped = 0;
        counter1=0;
        SysTickDisable();
    }

}
*/



/* Pedestrian Timer Setup */
/*
void PedTimerBegin(int time)
{
    if(time==tcross)
        Ped=1;

    SysTickEnable();
    //TimerEnable(TIMER0_BASE, TIMER_B);

}
*/

