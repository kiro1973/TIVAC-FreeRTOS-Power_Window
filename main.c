#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "FreeRTOSConfig.h"
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "queue.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include <stdbool.h> 
#include "tm4c123gh6pm.h"


/*-----------------------------------------------------------*/
/* The priority of the software interrupt.  The interrupt service routine uses
an (interrupt safe) FreeRTOS API function, so the priority of the interrupt must
be equal to or lower than the priority set by
configMAX_SYSCALL_INTERRUPT_PRIORITY - remembering that on the Cortex M3 high
numeric values represent low priority values, which can be confusing as it is
counter intuitive. */


/*
Call vPortEnterCritical() and vPortExitCritical()
	around code sections that should not
	be interrupted. This will mask global interrupts.
*/
/* The tasks to be created. */

void UpButton( void *pvParameters);
void DownButton( void *pvParameters);
	void manualDownButton(void *pvParameters);
	void manualUpButton(void *pvParameters);
void	limiter (void *pvParameters);
void pushAndLock(void *pvParameters) ; 
void vApplicationIdleHook();
/* Enable the software interrupt and set its priority. */
//static void prvSetupSoftwareInterrupt( void );

/* The service routine for the interrupt.  This is the interrupt that the
task will be synchronized with. */

/*-----------------------------------------------------------*/

xSemaphoreHandle xSemaphoreManualUp ; 
xSemaphoreHandle xSemaphoreManualDown;
SemaphoreHandle_t xSemaphore;
SemaphoreHandle_t xSemaphore2;
xSemaphoreHandle xMutex;
xSemaphoreHandle limit; 
xSemaphoreHandle Pushandlock; 
// Task handle for the task to be suspended

TaskHandle_t xTaskToSuspend;
// Task handle for the task to be suspended
TaskHandle_t xTaskToSuspend2;
/*-----------------------------------------------------------*/

void GPIOE_Handler(void)
{
	
		portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
		if (GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_0) == 0){
	 xSemaphoreGiveFromISR( limit, &xHigherPriorityTaskWoken );
		GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_0);
	}
	if (GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_1) == 0){
	 xSemaphoreGiveFromISR( limit, &xHigherPriorityTaskWoken );
		GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_1);
	}
	if (GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_2) == 0){
	 xSemaphoreGiveFromISR( Pushandlock, &xHigherPriorityTaskWoken );
		GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_2);
	}
		GPIO_PORTE_ICR_R |= 0xFF;
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}
/*-----------------------------------------------------------*/





void GPIOF_Handler(void) 
{	
	//ccc++ ; 
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	unsigned int state = GPIOF->DATA & 0x11; 
	//if (GPIOF-> DATA == ~GPIO_PIN_0 ){
if(state == 0x10){	//pin0 = 0x01 SW2 Down
		//xSemaphoreGiveFromISR( xSemaphore, &xHigherPriorityTaskWoken );
		xSemaphoreGiveFromISR( xSemaphoreManualDown, &xHigherPriorityTaskWoken );
    // Clear the interrupt flag
    GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_0);
    // GPIO Port A interrupt handler logic here
	}
	else if (state == 0x01) {   //pin4 = 0x10 SW1 Up
		
	 xSemaphoreGiveFromISR( xSemaphore, &xHigherPriorityTaskWoken ); // kan bta3 semaphore2 
	GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_4); 
	}
	else{
		GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_0);
	GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_4); 
	}

	   portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}


/*-----------------------------------------------------------*/

void GPIOB_Handler(void) 
{	
	
//	if ((GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_0) == 0 || GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_1) == 0 )){
	
	
	//}
	//else{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
unsigned int state = GPIOB->DATA & 0b11; 

	
	/*
	if (GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_4) == 0){
		int u =5 ; 
	}
	if (GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_5) == 0){
		int u = 5 ;
	}
	
	
		if (GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_6) == 0){
	 xSemaphoreGiveFromISR( limit, &xHigherPriorityTaskWoken );
		GPIOIntClear(GPIO_PORTB_BASE, GPIO_PIN_6);
	}
	if (GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_7) == 0){
	 xSemaphoreGiveFromISR( limit, &xHigherPriorityTaskWoken );
		GPIOIntClear(GPIO_PORTB_BASE, GPIO_PIN_7);
	}
	*/
	  if(state==0b10){
		xSemaphoreGiveFromISR( xSemaphore, &xHigherPriorityTaskWoken );
	
    // Clear the interrupt flag
    GPIOIntClear(GPIO_PORTB_BASE, GPIO_PIN_0);
   
	}
//	else if (GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_1) == 0) {   //pinB1 = 0x10 DOWN up
	else 	 if (state == 0b01) { 
	//if(GPIO_PORTB_RIS_R & 0x02){	
	 xSemaphoreGiveFromISR( xSemaphoreManualDown, &xHigherPriorityTaskWoken ); // kan xsemaphore2
	GPIOIntClear(GPIO_PORTB_BASE, GPIO_PIN_1); 
	}
	GPIO_PORTB_ICR_R |= 0xFF;
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
//}
	/*
	if (GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_6) == 0){
	 xSemaphoreGiveFromISR( limit, &xHigherPriorityTaskWoken );
		GPIOIntClear(GPIO_PORTB_BASE, GPIO_PIN_6);
	}
	if (GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_7) == 0){
	 xSemaphoreGiveFromISR( limit, &xHigherPriorityTaskWoken );
		GPIOIntClear(GPIO_PORTB_BASE, GPIO_PIN_7);
	}
	//GPIO_PORTD_ICR_R |= 0xFF;
	   
*/
}
/*-----------------------------------------------------------*/

void GPIOD_Handler(void)
{
		portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    if(GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_0) == 0) // Check if PD0 has triggered the interrupt //B6
    {
			// Auto up button sawa2 
			xSemaphoreGiveFromISR( xSemaphore2, &xHigherPriorityTaskWoken );
			GPIOIntClear(GPIO_PORTD_BASE, GPIO_PIN_0);
        // PD0 has triggered the interrupt, do something here
    }
    if(GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_1) == 0) // Check if PD1 has triggered the interrupt //B7
    {
			xSemaphoreGiveFromISR( xSemaphore2, &xHigherPriorityTaskWoken ); // passenger PD1 Automatic Up
			GPIOIntClear(GPIO_PORTD_BASE, GPIO_PIN_1);
			//Auto up button passenger
        // PD1 has triggered the interrupt, do something here
    }
    if(GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_2) == 0) // Check if PD2 has triggered the interrupt // passenger PD2 Manual 
    {
			xSemaphoreGiveFromISR( xSemaphoreManualUp, &xHigherPriorityTaskWoken );
			GPIOIntClear(GPIO_PORTD_BASE, GPIO_PIN_2);
			// Manual up button sawa2
        // PD2 has triggered the interrupt, do something here
    }
    if(GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_3) == 0) // Check if PD3 has triggered the interrupt
    {
			xSemaphoreGiveFromISR( xSemaphoreManualUp, &xHigherPriorityTaskWoken );
				GPIOIntClear(GPIO_PORTD_BASE, GPIO_PIN_3);
			//Manul up button passenger
        // PD3 has triggered the interrupt, do something here
    }
		    
    GPIO_PORTD_ICR_R |= 0x0F; // Clear the interrupt status for PD0-PD3
		   portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
	//}

}
/*-----------------------------------------------------------*/


void InitializeF (void){
SYSCTL ->RCGCGPIO |=0xFF; // Enable clock for all the ports
GPIOF -> LOCK = 0X4C4F434B; //Unlock PORTF
GPIOF -> CR = 0XFF; //Enable modifying PORTF registers
GPIOF -> DIR = 0X0E; //PORTF as output
GPIOF -> PUR = 0x11; //Pull Up resistor
GPIOF -> DEN = 0XFF; //Enable digital function at PORTF
}

void InitializeB (void){
	 // Enable the GPIOB peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Set PB0 and PB1 as inputs
  //  GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
GPIOB->LOCK = 0x4C4F434B;  // Unlock PortB
GPIOB->CR = 0xFF;        // Allow changes to Port B
//	GPIOB -> DEN = 0XFF;
    // Enable the internal pull-up resistors for PB0 and PB1

//	Set all PORTB pins as outputs
    //GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5);
	GPIOB -> DIR = 0x30; //PORTF as output
	//GPIOPinTypeGPIOInput(GPIO_PORTB_BASE ,GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_6 | GPIO_PIN_7 );
	    GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	GPIOB -> DIR = 0x030; //PORTF as output

	GPIOPinTypeGPIOInput(GPIO_PORTB_BASE ,GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_6 | GPIO_PIN_7 );
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5);
	 GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
		GPIOB -> PUR = 0xC3; 
	GPIOB -> DEN = 0xFF;
	    // Set interrupt priority for PB6 and PB7 to level 3
    NVIC_PRI0_R = (NVIC_PRI0_R & 0xFFFCFFFF) | 0x00030000;
//		NVIC_PRI0_R = (NVIC_PRI0_R & 0xFFFCFFFF) | 0x00060000;
// Set interrupt priority for PB0 and PB1 to level 6
    NVIC_PRI0_R = (NVIC_PRI0_R & 0xFFFF00FF) | 0x00006000;
    NVIC_EN0_R |= 0x00000002; // Enable interrupt 1 (corresponds to Port B) in NVIC

	
	
}


void PortE_Init(void) {
SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

GPIOE->CR = 0xFF;        // Allow changes to Port B
	GPIOE -> DIR = 0x00; //PORTF as output
	    GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1| GPIO_PIN_2  , GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU); //Pull up resistance
	GPIOE -> DIR = 0x000; //PORTF as output
	GPIOPinTypeGPIOInput(GPIO_PORTE_BASE ,GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 );
	 GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 , GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
		GPIOE -> PUR = 0x07; 
	GPIOE -> DEN = 0xFF;
	
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
			GPIOIntEnable(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 |  GPIO_PIN_2 );
			 NVIC_EnableIRQ(GPIOE_IRQn);
GPIOIntTypeSet(GPIO_PORTE_BASE,  GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2, GPIO_FALLING_EDGE);
//	GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_2, GPIO_FALLING_EDGE); //GPIO_HIGH_LEVEL
}


void PortD_Init(void)
{
	
	    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    // Set PB0 and PB1 as inputs
  //  GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
GPIOD->LOCK = 0x4C4F434B;  // Unlock PortB
GPIOD->CR = 0x1F;        // Allow changes to Port B
	GPIOD -> DIR = 0x00; //PORTD as intput
	//GPIOPinTypeGPIOInput(GPIO_PORTB_BASE ,GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_6 | GPIO_PIN_7 );
	    GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3| GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	GPIOPinTypeGPIOInput(GPIO_PORTD_BASE ,GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3| GPIO_PIN_4 );
	 GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3| GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
		GPIOD -> PUR = 0x1F; 
	GPIOD -> DEN = 0x1F;
	NVIC_EN0_R |= 0x00000008; // Enable interrupt 2 (corresponds to Port B) in NVIC
SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	GPIOIntEnable(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |GPIO_PIN_4);
			
	
	/*
	
//    SYSCTL_RCGCGPIO_R |= 0x08;    // Enable clock to GPIO Port D
//    while((SYSCTL_PRGPIO_R & 0x08) == 0){} // Wait for clock to stabilize
 SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIO_PORTD_LOCK_R = 0x4C4F434B; // Unlock PortD
    GPIO_PORTD_CR_R = 0x1F; // Allow changes to PD0-PD3

    //GPIO_PORTD_AMSEL_R &= ~0x0F; // Disable analog function on PD0-PD3
   // GPIO_PORTD_PCTL_R &= ~0x0000FFFF; // GPIO clear bit PCTL on PD0-PD3
    GPIO_PORTD_DIR_R &= ~0x1F; // Set PD0-PD3 as inputs
  //  GPIO_PORTD_AFSEL_R &= ~0x0F; // Disable alternate function on PD0-PD3
    GPIO_PORTD_PUR_R |= 0x1F; // Enable pull-up resistors on PD0-PD3
    GPIO_PORTD_DEN_R |= 0x1F; // Enable digital function on PD0-PD3
		//	 GPIO_PORTD_IS_R &= ~0x0F; // Set PD0-PD3 as edge-sensitive
   // GPIO_PORTD_IBE_R &= ~0x0F; // Disable both edges
    //GPIO_PORTD_IEV_R &= ~0x0F; // Set PD0-PD3 to trigger on falling edge
    GPIO_PORTD_ICR_R |= 0x1F; // Clear any prior interrupt
    GPIO_PORTD_IM_R |= 0x1F; // Enable interrupt for PD0-PD3
    NVIC_EN0_R |= 0x00000008; // Enable interrupt 3 (corresponds to Port D) in NVIC
			SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
			GPIOIntEnable(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |GPIO_PIN_4);
			 NVIC_EnableIRQ(GPIOD_IRQn);
 //GPIOIntTypeSet(GPIO_PORTD_BASE,  GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |GPIO_PIN_4, GPIO_FALLING_EDGE);
*/
}

int main( void )
{
	// Initialize system clock
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
	/*-----------------------------------------------------------*/
	//__asm("cpsie i");
	InitializeF();
	InitializeB() ; 
	PortD_Init() ;
	PortE_Init() ;
	
	 // Initialize GPIO and interrupt
	//////////////////////////////////////////////////////
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	////////////////////////////////////////////////////
	// Set PF4 as input (SWITCH1)
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_0);
    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_0);
    GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_FALLING_EDGE);
    
    // Enable interrupts for GPIO Port F Pin 0
   NVIC_EnableIRQ(GPIOF_IRQn); // heya dih 
	 NVIC_EnableIRQ(GPIOB_IRQn);
	  NVIC_EnableIRQ(GPIOD_IRQn);
    NVIC_SetPriority(GPIOF_IRQn, 6);
		 NVIC_SetPriority(GPIOD_IRQn, 6);
		// NVIC_SetPriority(GPIOB_IRQn, 6);
	//	 NVIC_SetPriority(GPIOB_IRQn, 3);
////////////////////////////////////SWTICH 1 PORTF ////////////////
   
	 // Set PF4 as input (SWITCH1)
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);
	 // Enable the internal pull-up resistor for PF4
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

// Enable interrupts for PF4 (SWITCH1)
    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_4);
    GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_FALLING_EDGE);

////////////////////////////////////SWTICH 1 PORTF ///////////////

IntEnable(INT_GPIOF);
IntEnable(INT_GPIOB);
IntEnable(INT_GPIOD);
IntEnable(INT_GPIOE);
///////////////////////////////////////////////////////////////PORTB
//GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 |GPIO_PIN_6 | GPIO_PIN_7 );

GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_0 , GPIO_FALLING_EDGE);
GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_1 , GPIO_FALLING_EDGE);
GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_6 , GPIO_FALLING_EDGE);
GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_7 , GPIO_FALLING_EDGE);


// Enable global interrupts
IntMasterEnable();
    __enable_irq();
	
	
	/*-----------------------------------------------------------*/
    /* Before a semaphore is used it must be explicitly created.  In this example
	a counting semaphore is created.  The semaphore is created to have a maximum
	count value of 10, and an initial count value of 0. */
	//	xQueue = xQueueCreate( 10, sizeof( long ) );
    //xCountingSemaphore = xSemaphoreCreateCounting( 10, 0 );
		limit =  xSemaphoreCreateBinary();
    xSemaphore = xSemaphoreCreateBinary();
		  xSemaphoreManualDown = xSemaphoreCreateBinary();
		xSemaphore2 = xSemaphoreCreateBinary();
		   xMutex = xSemaphoreCreateMutex();
			 xSemaphoreManualUp = xSemaphoreCreateBinary();
			 Pushandlock = xSemaphoreCreateBinary();
	/* Check the semaphore was created successfully. */
	if( xSemaphoreManualDown != NULL && xSemaphore != NULL && xSemaphore2 != NULL && xMutex != NULL && xSemaphoreManualUp != NULL && Pushandlock != NULL )
	{
    	/* Enable the software interrupt and set its priority. */
    //	prvSetupSoftwareInterrupt();

	//	xTaskCreate( vHandlerTask, "Handler", 240, NULL, 3, NULL );

		/* Create the task that will periodically generate a software interrupt.
		This is created with a priority below the handler task to ensure it will
		get preempted each time the handler task exist the Blocked state. */
		xTaskCreate( manualDownButton, "manualDown", 240, NULL, 1, &xTaskToSuspend );
		xTaskCreate( UpButton, "PeriodicUp", 240, NULL, 1, NULL );
    xTaskCreate( DownButton, "PeriodicDown", 240, NULL, 1, NULL );
		xTaskCreate( manualUpButton, "manualUp", 240, NULL, 1, &xTaskToSuspend2 );
		xTaskCreate( limiter, "Limiter", 240, NULL, 1, NULL );
		xTaskCreate( pushAndLock, "PushAndLock", 240, NULL, 1, NULL );
		/* Start the scheduler so the created tasks start executing. */
		vTaskStartScheduler();
	}

    /* If all is well we will never reach here as the scheduler will now be
    running the tasks.  If we do reach here then it is likely that there was
    insufficient heap memory available for a resource to be created. */
	for( ;; );
}

/*-----------------------------------------------------------*/

static void Motor(int mut ){

xSemaphoreTake( xMutex, portMAX_DELAY );
	{
		if (mut==0){
		GPIOB -> DATA &= 0b001111 ;
			GPIOF->DATA &= ~0x06 ; 
				//GPIOF->DATA = 0x00;
		}
 if (mut ==2){
GPIOB -> DATA |= 0b100000 ; // SW2 DOWN
	GPIOF->DATA &= ~0x04 ; 
	 GPIOF->DATA |= 0x02 ; 
	
	
}		
if(mut ==1) {
GPIOB -> DATA |= 0b010000 ; //SW1 UP
	GPIOF->DATA &= ~0x02 ; 
	GPIOF->DATA |= 0x04 ; 

}
		/* The following line will only execute once the semaphore has been
		successfully obtained - so standard out can be accessed freely. */
		
	}
xSemaphoreGive( xMutex );


}


void limiter ( void *pvParameters){

xSemaphoreTake( limit, 0 );
for(;;){
	
xSemaphoreTake( limit, portMAX_DELAY );
	/*
	while ((GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_0) == 0 || GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_1) == 0 ))
	{
	xSemaphoreTake( limit, portMAX_DELAY );
	}
	*/
  //		taskENTER_CRITICAL() ;
	
	while (GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_0) == 0 || GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_1) == 0){
		 Motor(0);
		vTaskSuspend(xTaskToSuspend) ; 
		vTaskSuspend(xTaskToSuspend2) ; 
   

		
	}
		vTaskResume(xTaskToSuspend); 
		vTaskResume(xTaskToSuspend2);
	/*
		while (GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_6) == 0 || GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_7) == 0 ){
	    GPIOB -> DATA &= 0b001111 ;
			GPIOF->DATA &= ~0x06 ; 
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
			
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
		//	while (GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_6) == 0 || GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_7) == 0 );
	//
	//	taskEXIT_CRITICAL();
		}
		*/
}


}


void manualDownButton( void *pvParameters ) //SW2 manual sawa2  portF
{
	
	//taskENTER_CRITICAL() ;


			xSemaphoreTake( xSemaphoreManualDown, 0 );
	
	for( ;; )
	{
		xSemaphoreTake( xSemaphoreManualDown, portMAX_DELAY );
		while(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_0) == 0){	  // SW2 
			Motor(2) ;
		}
		
		while (GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_1) == 0)
		{
			Motor(2) ;	 
		}
		
			Motor(0);
	//		taskEXIT_CRITICAL();
	}

}


 void DownButton( void *pvParameters ) //SW1 Automatic
{
	//	taskENTER_CRITICAL() ;
	// bool isAuto = true;



			xSemaphoreTake( xSemaphore, 0 );

	for( ;; )
	{
		xSemaphoreTake( xSemaphore, portMAX_DELAY );
//	 isAuto = true;

		while(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4) == 0){	 //sw1
		////////////////////////////////////mut =2; 
			Motor(2) ;
		// downButton
		}
		
		while (GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_0) == 0)
		{
		////////////////////////////////////mut =2; //down
			Motor(2) ;	 
		}
		/*
			while(1){
			Motor(2);
			//if (GPIOB->DATA & 0b1000000 == 0b0000000 || GPIOB->DATA & 0b10000000 == 0b00000000){
				if(GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_6) == 0 || GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_7) == 0){
					Motor(0);
					break ; 
					//isAuto = false ;
			////////////////////////////////////mut=0  ; 
					
			}			
			}
			*/

		//	taskEXIT_CRITICAL();
	}

}

/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/
 
 void UpButton( void *pvParameters ) //SW1
{
	  
//	taskENTER_CRITICAL() ;
			xSemaphoreTake( xSemaphore2, 0 );
	for( ;; )
	{
		xSemaphoreTake( xSemaphore2, portMAX_DELAY );
 //isAuto = true;
//	while ((GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_6) == 0 || GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_7) == 0 )) //D0 and D1 byzra2 lawe badelna while b if
	///{
	//xSemaphoreTake( xSemaphore2, portMAX_DELAY );
	//}
		while (GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_0) == 0){

			Motor(1);

		//	state = GPIOF->DATA & 0x10 ;
			
		}
		
			while (GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_1) == 0)
		{
	//////////////////////////////	mut =1; //UP
			Motor(1) ;	 

		
		}
		
	
	

	}

}

/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/


void manualUpButton ( void *pvParameters ) { 



			xSemaphoreTake( xSemaphoreManualUp, 0 );
	
	for( ;; )
	{
		xSemaphoreTake( xSemaphoreManualUp, portMAX_DELAY );
		while(GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_2) == 0){	  // SW2 
			Motor(1) ;
		}
		
		while (GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_3) == 0)
		{
			Motor(1) ;	 
		}
		
			Motor(0);
	//		taskEXIT_CRITICAL();
	}






}

/*-----------------------------------------------------------*/
void pushAndLock(void *pvParameters){ // Disable passenger from controlling
int counter = 0 ;
	xSemaphoreTake( Pushandlock, 0 );
for( ;; )
	{
		xSemaphoreTake( Pushandlock, portMAX_DELAY );
		if (GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_2) == 0){
		while(GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_2) == 0){
		// Disable interrupts on Port D pins 1 and 2
GPIOIntDisable(GPIO_PORTD_BASE, GPIO_PIN_1 | GPIO_PIN_2);

// Disable interrupts on Port B pins 0 and 1
GPIOIntDisable(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
GPIOF->DATA |= 0x04 ;
			if (counter == 0 ){
			Motor(0);
				counter++ ; 
		}
			
		}
		GPIOF->DATA &= ~0x04 ;
	}
		else {
		GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
		 //InitializeB();
		GPIOIntEnable(GPIO_PORTD_BASE, GPIO_PIN_1 | GPIO_PIN_2);
		//PortD_Init();
			counter = 0 ; 
		}
		//GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_2);
	}



}
/*-----------------------------------------------------------*/



void vApplicationIdleHook( void )
{
	/* This example does not use the idle hook to perform any processing.  The
	idle hook will only be called if configUSE_IDLE_HOOK is set to 1 in 
	FreeRTOSConfig.h. */
}
/*-----------------------------------------------------------*/





