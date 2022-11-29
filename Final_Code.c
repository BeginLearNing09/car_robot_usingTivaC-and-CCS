/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>

/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "driverlib/PWM.h"
#include "utils/uartstdio.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/timer.h"

#define MAXSPEED 100
#define BUFFERSIZE 40
#define STARTCLOCK 18446744073709551615

extern const ti_sysbios_knl_Semaphore_Handle semaphore0;
extern const Swi_Handle error_data_collect;

volatile char letter[2];
volatile int DutyCycle = 100;
volatile int i;
volatile float K_TEST = 1.5;
volatile float error_prev = 0;
volatile float I_prev = 0;
volatile uint32_t tape_cycle;
volatile uint32_t timer0A_count = 2000000 - 1;// 50 ms for PID  (40000000/1000)*50 =50 ms
volatile uint32_t timer0B_count = 200000 - 1;//5 ms for reflectance sensor  (40000000/1000)*5 = 5 ms
volatile uint64_t timer1A_count = 18446744073709551615;// max number of 64 bits of timer (2^64 -1)
volatile uint64_t discharge_time;
volatile uint64_t start_clock;
volatile uint32_t end_clock;
volatile uint32_t Frequency = 2000;//Systemfrequency/BrushDCmotorfrequency(20000) = 2000
volatile int32_t error = 0;
volatile int current_buffer_size = 0;
volatile int first_tape = 0;
volatile char ping_buffer[BUFFERSIZE];
volatile char pong_buffer[BUFFERSIZE];
volatile char *current_buffer = ping_buffer;
volatile char overhead[] = "19CRLF";

void GF_Function()// goForward function
{
    DutyCycle = 100;//maximum Duty function

    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, (int)(Frequency));//PWM leftwheel A enable (pinE4)
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, (int)(Frequency));//PWM Rightwheel B enable (pinE5)
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, 0);//Pin A6 A phase , leftWheel, 0 is forward
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0);//Pin A7 B phase , leftWheel, 0 is forward
    PWMOutputState(PWM0_BASE, (PWM_OUT_4_BIT | PWM_OUT_5_BIT), true);// PWM run when true and otherwise
    TimerEnable(WTIMER0_BASE, TIMER_A);// enable TIMER_0A is for PID every 50ms
    TimerEnable(WTIMER0_BASE, TIMER_B);// enable TIMER_0B is for black tape every 5ms
    TimerEnable(WTIMER1_BASE, TIMER_A);// enable Timer 1 for counting how long the robot run the maze
}



void ES_Function()// Emergency stop function
{
    PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT | PWM_OUT_5_BIT, false);// PWM both wheel stop when the boolean is false
    end_clock = (uint32_t)(((uint64_t)STARTCLOCK - TimerValueGet64(WTIMER1_BASE)) / 400000);//it counts down timer with start at (2^64-1) - (currentTimer)/ 400000 
    UARTprintf("%d", end_clock);// Print out the time to run a maze
    TimerDisable(WTIMER1_BASE, TIMER_A);//disable Timer 1 for counting how long the robot run the maze
    TimerDisable(WTIMER0_BASE, TIMER_A);// disable TIMER_0A is for PID every 50ms
    TimerDisable(WTIMER0_BASE, TIMER_B);// disable TIMER_0B is for black tape every 5ms
    TimerLoadSet64(WTIMER1_BASE, timer1A_count);// go back to 2^64- 1
    TimerLoadSet(WTIMER0_BASE, TIMER_A, timer0A_count);// 50ms PID 
    TimerLoadSet(WTIMER0_BASE, TIMER_B, timer0B_count);//5 ms for reflectance sensor
}



//BLUETOOTH MODULE 
//create a struct name Command_struct has string (2 char) and pointer to function 
typedef struct Command_Struct
{
    char the_command[2];
    void (*fun_ptr)();
} Command_Struct;
//lookup the table and we can have 15 functions maximum
Command_Struct Command_List[15] = {
    {"GF", GF_Function}, {"IC", IC_Function}, {"DC", DC_Function}, {"ES", ES_Function}};


void UART1Init()//use UART1 to communicate blue tooth
//UART1 B0 receive, B1 Transmitted
{
    // Enable the UART1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    // Enable the GPIOB 
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    // Wait for the GPIOB and UART1 to be ready.
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_UART1) || !SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB))
    {
    }
    // Configure GPIO Port B pins 0 and 1 to be used as UART1.
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    // Enable UART1 functionality on GPIO Port B pins 0 and 1.
    GPIOPinConfigure(GPIO_PB0_U1RX);
    GPIOPinConfigure(GPIO_PB1_U1TX);
    //Master interrupt enable API for all interrupts
    IntMasterEnable();
    // Initialize the UART. Set the baud rate, number of data bits, turn off
    // parity, one stop bits, and stick mode. The UART is enabled by the
    //function call
    UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 9600, UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);
    //interrupt enable UART1
    IntEnable(INT_UART1);
    //Enables the transmit and receive FIFOs (UART1)
    UARTFIFOEnable(UART1_BASE);
    //Buffer have 16 bytes, each letter is a bytes (8 bits, 256 ascii characters available)
    //Our command is 2 chars so make the interrupt happens every 2 characters receive, set FIFO level to 1/8 since 16*1/8 = 2 (2 characters)
    UARTFIFOLevelSet(UART1_BASE, UART_FIFO_TX1_8, UART_FIFO_RX1_8);
    UARTIntEnable(UART1_BASE, UART_INT_RX);// just need interrupt when receice commamnd and Tiva C excute the function
    UARTStdioConfig(1, 9600, 40000000);
}
void UART1IntHandler()//UART1 interrupt when user send command from bluetooth to tivaC, interrupt happens and
//exucute function 
{
    UARTIntClear(UART1_BASE, UART_INT_RX);//a write buffer procecor may take 
    //several clock to clear previous command, so it need to be cleared early
    //before the interrupt sources is actually clear
    int i;

    while (UARTCharsAvail(UART1_BASE))// run until 2 chars is placed into the 
        //receive FIFO
    {
        for (i = 0; i < 2; i++)
        {
            letter[i] = UARTCharGet(UART1_BASE);
        }
    }
    for (i = 0; i < 15; i++)// check the string in letter is match with the_command?
    {
        if (strcmp((char*)letter, Command_List[i].the_command) == 0)// if ==0 , so they are matching 
        {
            (*Command_List[i].fun_ptr)();// excute the function as command on table 
        }
    }
}




void GPIOAInit()//GPIOA output for MODE, Phase A, Phase B
{
    // Enable the GPIOA
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    // Wait for the GPIOA  to be ready.
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA))
    {
    }
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);// out put Pin5, 6,7 if GPIO A
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, GPIO_PIN_5);//MODE ON (high) Pin A5
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, 0);//Pin A6 A phase , leftWheel, 0 is forward
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0);//Pin A7 B phase , leftWheel, 0 is forward
}

//Use to turn on the blue light while collecting data from 1st black tape to 2nd tape
void GPIOFInit()
{
    // Enable the GPIOF
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    // Wait for the GPIOF  to be ready.
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
    {
    }
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);// output 3 Pin 1,2,3 in GPIO F
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0x00);
}

void UART0Init()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTStdioConfig(0, 115200, 16000000);
}



void PWM_Init()//PWM initialize
{
    // Enable the GPIOE
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    // Wait for the GPIOE  to be ready.
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE))
    {
    }
    //Configure GPIO Port E pins 4 and 5 to be used as PWM
    GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    //PWM module 0, Generator 2 with PWM4,5
    GPIOPinConfigure(GPIO_PE4_M0PWM4);
    GPIOPinConfigure(GPIO_PE5_M0PWM5);
    // Enable the PWM0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    // Wait for the PWM0 module to be ready.
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0))
    {
    }
    // Configure the PWM generator for count down mode with immediate updates
// to the parameters.dont sync with any else generator 
    PWMGenConfigure(PWM0_BASE, PWM_GEN_2,PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
    // Set the period. For a  DC recommend frequency 20 KHz frequency, the period = 1/20000, or 50
// microseconds. For a 40 MHz clock, this translates to 2000 clock ticks.
// Use this value to set the period. Frequency =2000 clock ticks
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, Frequency);


    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, (int)(Frequency * DutyCycle * 0.01));//PWM leftwheel A enable 
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, (int)(Frequency * (DutyCycle)*0.01));//PWM rightWheel B enable
    // Start the timers in generator 2
    PWMGenEnable(PWM0_BASE, PWM_GEN_2);
}

void Timer0_init(void)// use interrupt every 50ms for PID and 5ms for black tape
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0);//enable the Wide time 0 32/64 bits
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_WTIMER0))// Wait for the WTIMER0  to be ready.
    {
    }
    TimerClockSourceSet(WTIMER0_BASE, TIMER_CLOCK_SYSTEM);//set a specific timer clock to WTIMER0
    //configure the timer, use haft-width periodic timer A,B on WTIME0, with 2 half-width timer and they count down 
    TimerConfigure(WTIMER0_BASE, TIMER_CFG_A_PERIODIC | TIMER_CFG_B_PERIODIC | TIMER_CFG_SPLIT_PAIR);
    // TIMER_0A is for PID every 50ms
    IntEnable(INT_WTIMER0A);//enable the interrupt WTIMER0A
    TimerIntEnable(WTIMER0_BASE, TIMER_TIMA_TIMEOUT);//count down until 50ms and interrupt 
    TimerLoadSet(WTIMER0_BASE, TIMER_A, timer0A_count);//set the time count down from 50ms
    // TIMER_0B is for black tape every 5ms
    IntEnable(INT_WTIMER0B);//enable the interrupt WTIMER0B
    TimerIntEnable(WTIMER0_BASE, TIMER_TIMB_TIMEOUT);//count down until 50ms and interrupt 
    TimerLoadSet(WTIMER0_BASE, TIMER_B, timer0B_count);//set the time count down from 50ms
}

// Timer 1 for counting how long the robot run the maze use 64bits
void Timer1_init(void)
{
    uint64_t timer1A_count = pow(2, 64) - 1;// maxvalue of timer 64 bit
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER1);//enable the Wide time 1 32/64 bits
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_WTIMER1))// Wait for the WTIMER1  to be ready.
    {
    }
    TimerClockSourceSet(WTIMER1_BASE, TIMER_CLOCK_SYSTEM);//set a specific timer clock to WTIMER1
    // TimerA
    TimerConfigure(WTIMER1_BASE, TIMER_CFG_PERIODIC);//configure the timer use full-width periodic timer
    TimerLoadSet64(WTIMER1_BASE, timer1A_count);//load value of 64 bit to timer when it runs
}

// Timer 2 for send error to PC every 100ms use 32bits
void Timer2_init(void)
{
    uint32_t timer2A_count = 4000000;//set 100ms as (40000000/1000)*100 
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER2);//enable the Wide time 2 32/64 bits
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_WTIMER2))// Wait for the WTIMER2  to be ready.
    {
    }
    TimerClockSourceSet(WTIMER2_BASE, TIMER_CLOCK_SYSTEM);//set a specific timer clock to WTIMER2
    //configure the timer, use haft-width periodic timer A
    TimerConfigure(WTIMER2_BASE, TIMER_CFG_A_PERIODIC | TIMER_CFG_SPLIT_PAIR);
    TimerLoadSet(WTIMER2_BASE, TIMER_A, timer2A_count);//load value 100ms to timer when it runs
    IntEnable(INT_WTIMER2A);//enable the interrupt WTIMER2A
    TimerIntEnable(WTIMER2_BASE, TIMER_TIMA_TIMEOUT);//count down until 100ms and interrupt 
}

void ADC_init(void)// initailize the ADC sensor front, right 
{   
    // Enable the GPIOD
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    // Enable the ADC0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    // Wait for the GPIOD and ADC0  to be ready.
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0) || !SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD))
    {
    }
    //Configure GPIO Port D pins 0 and 1 to be used as ADC0
    GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    //use sequence 2 and trigger processor(generared by processor), and highest priority in sequence
    ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_CH6);// use channel 6 as D0(right sensor) for 1st step
    ADCSequenceStepConfigure(ADC0_BASE, 2, 1, ADC_CTL_CH7 | ADC_CTL_END);// use channel 7 as D1(front sensor) for 2nd step
    // ADCIntEnable(ADC0_BASE, 2);
    ADCSequenceEnable(ADC0_BASE, 2);//enable the sample sequence 2
}


void error_data_collect_handler()//software interrupt handler
{
    char *prev_buffer = (char *)current_buffer;//make another buffer pointer
    int trans_size = current_buffer_size;// buffer transmit size
    current_buffer_size = 0;// set current buffer size back to 0
    if (current_buffer == ping_buffer)// if current is ping switch to pong and vice versa
    {
        current_buffer = pong_buffer;

    }
    else
    {
        current_buffer = ping_buffer;
    }
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);//turn off blue led 
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);// turn on blue led
    int i = 0;
    while (i < trans_size)// send all the available data in the buffer
    {
        UARTCharPut(UART1_BASE, prev_buffer[i]);
        i++;
    }
    i = 0;
    while (i < 6)// 6 characters of the overhead for the modbus
    {
        UARTCharPut(UART1_BASE, overhead[i]);
        i++;
    }
    if (first_tape != 2)
	// if the flag is not 2 meaning it did not cross the second black tape and this is not the last transmit so we turn blue led back on
    // if it is 2 meaning this is the last transmit and we should not turn blue led back on
    {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
    }
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);// done transmitting so turn off green


}

void Timer0A_Int_Handler()

{
    TimerIntClear(WTIMER0_BASE, TIMER_TIMA_TIMEOUT);// timer interrupt for PID
    Semaphore_post(semaphore0);// post semaphore for PID task
}

void Timer0B_Int_Handler()// timer interrupt for reading reflectance sensor
{
    TimerIntClear(WTIMER0_BASE, TIMER_TIMB_TIMEOUT);
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_4);//charge the sensor using GPIO port A pin 4 by output high
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, GPIO_PIN_4);
    SysCtlDelay(SysCtlClockGet() / 600000); // delay for 10[us] because according to data sheet, it is recommended to charge for 10[us]

    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_4);// change the output pin into input to read discharge time
    GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_4);

    start_clock = TimerValueGet64(WTIMER1_BASE);// initial clock for discharge
    while (GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_4))// ask long ask discharge value still high, wait
    {
    }
    discharge_time = start_clock - TimerValueGet64(WTIMER1_BASE);// when it done discharge, read the clock value to get the discharge time from initial clock
    if (discharge_time > 18000)// if the timeto discharge is more than 18000 clock cycle, it is a black tape
    {
        tape_cycle++; // add a cycle to tape_cycle
        if (first_tape == 0) // if this is first black tape
        {
            first_tape = 1;// change the flag to 1 
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);//turn on Blue lED
            TimerEnable(WTIMER2_BASE, TIMER_A);// enable error collection timer
        }
        else if (first_tape == -1)// flag = -1 mean that robot already pass first tape and this is the second tape
        {
            first_tape = 2;// set flag =2 to tell that second tape have pass
            TimerDisable(WTIMER2_BASE, TIMER_A);// disable error collection timer
            Swi_post(error_data_collect);// transmit the rest of the data inside buffer
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);// turn off Blue LED

        }
        if (tape_cycle > 18)// if there are 18 consecutive black tape cycles meaning thick black tape
        {
            ES_Function();// stop robot
        }
    }
    else// not a black tape, white surface,etc
    {
        if (first_tape == 1)// just get over the first black tape and see the white surface
        {
            first_tape = -1;// set the flag to -1 to tell that robot have already pass first tape
        }
        tape_cycle = 0;// reset tape cycle
    }
}

void Timer2A_Int_Handler()//100 [ms] timer interupt error collection handler
{
    TimerIntClear(WTIMER2_BASE, TIMER_TIMA_TIMEOUT);
    //ADC value is 12 bits, 2^12-1, therefore there will be 2 charcaters slit from 12 bits
    current_buffer[current_buffer_size] = (char)(abs(error) >> 8);//convert the first 4 bits into a HEX Ascii char 
    current_buffer[current_buffer_size + 1] = (char)(abs(error) & 0xFF);//convert the last 8 bits into a HEX Ascii char 
    current_buffer_size += 2;// update size
    if (current_buffer_size > 39)//if buffer is full post software interrupt
    {
        Swi_post(error_data_collect);
    }
}


void PID_Task()
{
    while (1)
    {
        Semaphore_pend(semaphore0, BIOS_WAIT_FOREVER);
        uint32_t Input[2];
        ADCProcessorTrigger(ADC0_BASE, 2); // Trigger ADC sensor
        while (!ADCSequenceDataGet(ADC0_BASE, 2, Input))//read sensors
        {
        }
        int fval = Input[0];//front sensor
        int rval = Input[1];//right sensor
        if (fval > 2200) // car is too close front wall , then it will U-turn
        {
            if (first_tape == -1)//meaning it already over the first black tape and is collecting data,
                //aka the timer is enable, pingpong buffer is on
            {
                TimerDisable(WTIMER2_BASE, TIMER_A);// disable 100ms timer for collecting data pingpong
            }

            TimerDisable(WTIMER0_BASE, TIMER_A);//disable TIMER_0A is for PID every 50ms
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6);//change the phase of leftWheel to high(reverse)
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, (int)(Frequency));//set wheel to 100% duty cycle
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, (int)(Frequency));//set wheel to 100% duty cycle
            while (fval > 1200)// if front still see the wall , it keep checking the front sensor every 5ms while u-turning
            {
                ADCProcessorTrigger(ADC0_BASE, 2);
                while (!ADCSequenceDataGet(ADC0_BASE, 2, Input))
                {
                }
                fval = Input[0];
                SysCtlDelay(SysCtlClockGet() / 600);//SysCtlDelay alway 3s so 40000000/600 = 5ms
            }
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, 0x0);// change the phase of leftWheel to low(forward)
            TimerEnable(WTIMER0_BASE, TIMER_A);//enable TIMER_0A is for PID every 50ms
            if (first_tape == -1)//meaning it already over the first black tape and is collecting data,
            //aka the timer is enable, pingpong buffer is on
            {
                TimerEnable(WTIMER2_BASE, TIMER_A);// Re-enable 100ms timer for collecting data pingpong
            }
            error_prev = 0;//reset error for PID
            I_prev = 0;//reset I for PID
        }
        else
        {
            error = 1750 - rval;//Desire ADC value - Actual ADC  right value
            float Kp = 0.12;// constant propostial 
            float Ki = 0.001;//constant integral 
            float Kd = 0.003;//contant derivative
            float I = I_prev + (float)error * 0.05;
            float D = ((float)error - error_prev) / 0.05;
            float steering = fabs(Kp * (float)error + Ki * I + Kd * D);//calculate PID value,PID is amount of dutyCycle we adjust on PWM
            if (steering >= 100)// since we use PID value as dutyCycle , just ensure its not over 100
            {
                steering = 99;
            }
            if (error > 0)//steering right
            {
                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, (int)(Frequency));              // Left Wheel run at 100 % dutyCycle
                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, (int)(Frequency * (MAXSPEED - steering) * 0.01)); // Right Wheel
            }
            else//steering left
            {
                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, (int)(Frequency * (MAXSPEED - steering) * 0.01)); // Left Wheel
                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, (int)(Frequency));              // Right Wheel run at 100 % dutyCycle
            }

            error_prev = (float)error;
            I_prev = I;
        }
    }
}

int main(void)
{
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    GPIOAInit();
    GPIOFInit();
    UART1Init();
    ADC_init();
    PWM_Init();
    Timer0_init();
    Timer1_init();
    Timer2_init();
    BIOS_start();
    return (0);
}
