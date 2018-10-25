/*
===============================================================================
 Name        : main.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
 */

#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#endif
#endif

#include <cr_section_macros.h>
#include "FreeRTOS.h"
#include "task.h"
#include "DigitalIoPin.h"
#include "semphr.h"
#include <mutex>
#include <stdio.h>
#include "queue.h"
#include <string>
#include <cmath>
#include "Fmutex.h"
#include "user_vcom.h"
#include <cstring>

#define LEFT true			//counter clockwise
#define RIGHT false			//clockwise

struct profile {
	bool motorX_dir;		//0: clockwidse, 1: counter-clockwise
	bool motorY_dir;
	int speed;				//0 - 100%
	int penUp;				//0 - 255
	int penDown;
	int penPos;
	bool limXmax;
	bool limXmin;
	bool limYmax;
	bool limYmin;
	int height;
	int width;
	double X;
	double Y;
	bool A;
};

volatile uint32_t XRIT_count;
volatile uint32_t YRIT_count;

xSemaphoreHandle sbRIT = xSemaphoreCreateBinary();
xSemaphoreHandle limXmax_sem = xSemaphoreCreateBinary();
xSemaphoreHandle limXmin_sem = xSemaphoreCreateBinary();
xSemaphoreHandle limYmax_sem = xSemaphoreCreateBinary();
xSemaphoreHandle limYmin_sem = xSemaphoreCreateBinary();
xSemaphoreHandle calib_donesem = xSemaphoreCreateBinary();
xSemaphoreHandle inten_donesem = xSemaphoreCreateBinary();
//xSemaphoreHandle ok_sentsem = xSemaphoreCreateBinary();
QueueHandle_t xQueue;
QueueHandle_t xQueue_servo;

static void prvSetupHardware(void);
void RIT_start(int xcount, int ycount, int us);
void SetupInt(int port, int pin, int index);
void GotoPos(DigitalIoPin xdir, DigitalIoPin ydir, int &xcurrent_pulse, int &ycurrent_pulse, double x, double y, double pulseOnwidth, double pulseOnheight);
static void vTask1(void *pvParameters);
static void vTask2(void *pvParameters);
static void vTask4(void *pvParameters);




int main(void)
{
	xQueue = xQueueCreate(1, sizeof(profile));
	xQueue_servo = xQueueCreate(1, sizeof(profile));
	if(xQueue != NULL && xQueue_servo != NULL) {
		prvSetupHardware();
		xTaskCreate(vTask1, "serial",
				configMINIMAL_STACK_SIZE + 450, NULL, (tskIDLE_PRIORITY + 1UL),
				(TaskHandle_t *) NULL);

		xTaskCreate(vTask2, "stepper",
				configMINIMAL_STACK_SIZE + 300, NULL, (tskIDLE_PRIORITY + 1UL),
				(TaskHandle_t *) NULL);

		xTaskCreate(vTask4, "servo",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),
				(TaskHandle_t *) NULL);
		xTaskCreate(cdc_task, "CDC",
				configMINIMAL_STACK_SIZE + 150, NULL, (tskIDLE_PRIORITY + 1UL),
				(TaskHandle_t *) NULL);
		vTaskStartScheduler();
	}
	else
		return 1;
}

static void vTask1(void *pvParameters) {		//serial
	char command[26] = {0};
	unsigned char reply[60];
	const unsigned char OK_reply[] = "OK\r\n";
	int i = 0, j, pos, num = 0, cmd_len;
	uint8_t c;
	double coordinate;
	std::string str;
	profile plotter;
	plotter.motorX_dir = 0;
	plotter.motorY_dir = 0;
	plotter.penDown = 80;
	plotter.penUp = 120;
	plotter.penPos = 100;
	plotter.speed = 50;
	plotter.height = 310;
	plotter.width = 380;
	plotter.X = 0.0;
	plotter.Y = 0.0;
	plotter.A = 0;

	bool flow = true;
	while(xSemaphoreTake(calib_donesem, 0) != pdTRUE) {				//wait until calibration is done
	}

	while(1) {
		cmd_len = USB_receive((uint8_t *) command, 25);

		if(command[cmd_len-1] == '\n') {
			str = command;
			str[str.length()-1] = '\0';

			if((pos = str.find("M10")) != -1) {
				snprintf((char*) reply, 60, "M10 XY 380 310 0.00 0.00 A%d B%d H0 S%d U%d D%d\r\nOK\r\n",
						plotter.motorX_dir, plotter.motorY_dir, plotter.speed, plotter.penUp, plotter.penDown);

				USB_send(reply, strlen((char*) reply));
				str = "";
			}
			if((pos = str.find("M11")) != -1) {
				snprintf((char*)reply, 60, "M11 %d %d %d %d\r\nOK\r\n",
						plotter.limXmin, plotter.limXmax, plotter.limYmin, plotter.limYmax);
				USB_send(reply, strlen((char*) reply));
				str = "";
			}
			if((pos = str.find("M2 U")) != -1) {					//send to task 4
				pos += 4;											//first digit after "M2 U"
				for(j = pos; str[j] >= '0' && str[j] <= '9'; j++) {
				}
				num = 0;
				for(; pos < j; pos++ ) {
					num += (str[pos] - '0') * pow(10, j - pos - 1);
				}
				plotter.penUp = num;
				pos += 2;
				for(j = pos; str[j] >= '0' && str[j] <= '9'; j++) {
				}
				num = 0;
				for(; pos < j; pos++ ) {
					num += (str[pos] - '0') * pow(10, j - pos - 1);
				}
				plotter.penDown = num;
				USB_send((uint8_t*) OK_reply, 4);
				str = "";
			}
			if((pos = str.find("M1 ")) != -1) {						//task 4
				pos += 3;											//first digit after "M1 "
				for(j = pos; str[j] >= '0' && str[j] <= '9'; j++) {
				}
				num = 0;
				for(; pos < j; pos++ ) {
					num += (str[pos] - '0') * pow(10, j - pos - 1);
				}
				plotter.penPos = num;
				USB_send((uint8_t*) OK_reply, 4);
				xQueueSend(xQueue_servo, &plotter, 0);
				str = "";
			}
			if((pos = str.find("M5 A")) != -1) {					//task 3
				pos += 4;											//first digit after "M5 A"
				num = str[pos] - '0';
				plotter.motorX_dir = (bool) num;

				pos += 3;
				num = str[pos] - '0';
				plotter.motorY_dir = (bool) num;

				pos += 3;
				for(j = pos; str[j] >= '0' && str[j] <= '9'; j++) {
				}
				num = 0;
				for(; pos < j; pos++ ) {
					num += (str[pos] - '0') * pow(10, j - pos - 1);
				}
				plotter.height = num;
				pos += 2;
				for(j = pos; str[j] >= '0' && str[j] <= '9'; j++) {
				}
				num = 0;
				for(; pos < j; pos++ ) {
					num += (str[pos] - '0') * pow(10, j - pos - 1);
				}
				plotter.width = num;
				pos += 2;
				for(j = pos; str[j] >= '0' && str[j] <= '9'; j++) {
				}
				num = 0;
				for(; pos < j; pos++ ) {
					num += (str[pos] - '0') * pow(10, j - pos - 1);
				}
				plotter.speed = num;
				USB_send((uint8_t*) OK_reply, 4);
				str = "";
				xQueueSend(xQueue, &plotter, 0);

			}
			if((pos = str.find("G28")) != -1) {								//task 3
				plotter.X = 0.0;
				plotter.Y = 0.0;
				str = "";
				xQueueSend(xQueue, &plotter, 0);
			}
			if((pos = str.find("G1 X")) != -1) {							//task 3
				pos += 4;						//first digit after "M2 U"
				for(j = pos; str[j] >= '0' && str[j] <= '9'; j++) {			//get final digit pos of xx. part
				}
				coordinate = 0;
				for(; pos < j; pos++ ) {									//collect the xx.
					coordinate += (str[pos] - '0') * pow(10, j - pos - 1);
				}
				for(j = pos + 2; str[j] >= '0' && str[j] <= '9'; j++) {		//get final digit pos of .xx part
				}
				for(pos++; pos < j; pos++) {								//collect the .xx
					coordinate += (str[pos] - '0') * pow(10, j - pos - 3);
				}
				plotter.X = coordinate;

				pos += 2;													//first digit after "Y"
				for(j = pos; str[j] >= '0' && str[j] <= '9'; j++) {			//get final digit pod of xx. part
				}
				coordinate = 0;
				for(; pos < j; pos++ ) {									//collect the xx.
					coordinate += (str[pos] - '0') * pow(10, j - pos - 1);
				}
				for(j = pos + 2; str[j] >= '0' && str[j] <= '9'; j++) {		//get final digit pos of .xx part
				}
				for(pos++; pos < j; pos++) {								//collect the .xx
					coordinate += (str[pos] - '0') * pow(10, j - pos - 3);
				}
				plotter.Y = coordinate;

				pos += 2;
				num = str[pos] - '0';
				plotter.A = (bool) num;
				str = "";
				USB_send((uint8_t*) OK_reply, 4);
				xQueueSend(xQueue, &plotter, 0);
			}

		}




		if(xSemaphoreTake(limXmin_sem, 0) == pdPASS) {			//take sem when lim is reached
			plotter.limXmin = false;
			flow = false;
		}
		if(xSemaphoreTake(limXmax_sem, 0) == pdPASS) {			//take sem when lim is reached
			plotter.limXmax = false;
			flow = false;
		}
		if(xSemaphoreTake(limYmin_sem, 0) == pdPASS) {			//take sem when lim is reached
			plotter.limYmin = false;
			flow = false;
		}
		if(xSemaphoreTake(limYmax_sem, 0) == pdPASS) {			//take sem when lim is reached
			plotter.limYmax = false;
			flow = false;
		}

	}
}


static void vTask2(void *pvParameters) {					//motors
	DigitalIoPin YSTEP(0, 27, DigitalIoPin::output, false);
	DigitalIoPin YDIR(0, 28, DigitalIoPin::output, false);
	DigitalIoPin XSTEP(0, 24, DigitalIoPin::output, false);
	DigitalIoPin XDIR(1, 0, DigitalIoPin::output, false);
	int Xpulse_count = 0, Ypulse_count = 0, Xcurrent_pulse = 0, Ycurrent_pulse = 0;
	double XpulseOverwidth, YpulseOverheight;
	profile plotter2;


	XDIR.write(RIGHT);
	while(xSemaphoreTake(limXmin_sem, 0) == pdFALSE) {		//go to X = 0
		RIT_start(10, 0, 200);
	}

	YDIR.write(RIGHT);
	while(xSemaphoreTake(limYmin_sem, 0) == pdFALSE) {		//go to Y = 0

		RIT_start(0, 10, 200);
	}

//	XDIR.write(LEFT);
//	while(xSemaphoreTake(limXmax_sem, 0) == pdFALSE) {		//go to X = max
//		Xpulse_count++;
//		RIT_start(16, 0, 200);
//	}
//
//	YDIR.write(LEFT);
//	while(xSemaphoreTake(limYmax_sem, 0) == pdFALSE) {		//go to Y  = max
//		Ypulse_count++;
//		RIT_start(0, 16, 200);
//	}



//	Xpulse_count = Xpulse_count*8;
//	Ypulse_count = Ypulse_count*8;
	Xpulse_count = 27000;
	Ypulse_count = 30000;
	Xcurrent_pulse = Xpulse_count/2;
	Ycurrent_pulse = Ypulse_count/2;
	XpulseOverwidth = (double) Xpulse_count/340;
	YpulseOverheight = (double) Ypulse_count/310;

	XDIR.write(LEFT);
	YDIR.write(LEFT);
	vTaskDelay(1000);

	RIT_start(0, Ycurrent_pulse*2, 500);
	vTaskDelay(1000);
	RIT_start(Xcurrent_pulse*2, 0, 500);
	xSemaphoreGive(calib_donesem);							//give semaphore to task 1

	while(1) {
		if(xQueueReceive(xQueue, &plotter2, 10) == pdTRUE) {
			GotoPos(XDIR, YDIR, Xcurrent_pulse, Ycurrent_pulse, plotter2.X, plotter2.Y, XpulseOverwidth, YpulseOverheight);


		}

		//vTaskDelay(configTICK_RATE_HZ/50);


	}

}


static void vTask4(void *pvParameters) {		//servo and lazer
	profile plotter4;
	uint32_t  ticks, penup, pendown;
	/* Initialize the SCT as PWM and set frequency */
	Chip_SCTPWM_Init(LPC_SCT0);
	Chip_SCTPWM_SetRate(LPC_SCT0, 50);					//50 Hz
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);		// Enable SWM clock before altering SWM

	Chip_SWM_MovablePinAssign(SWM_SCT0_OUT1_O, 10);		//Set PIO0_10 as PWM out
	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);
	Chip_SCTPWM_SetOutPin(LPC_SCT0, 1, 1);				//index: 1, pinout: 1
	Chip_SCTPWM_SetDutyCycle(LPC_SCT0, 1, Chip_SCTPWM_PercentageToTicks(LPC_SCT0, 8));					//start at 8% duty cycle
	Chip_SCTPWM_Start(LPC_SCT0);

	penup = Chip_SCTPWM_GetTicksPerCycle(LPC_SCT0)*10/100;
	pendown = Chip_SCTPWM_GetTicksPerCycle(LPC_SCT0)*5/100;

	while(1) {
		if(xQueueReceive(xQueue_servo, &plotter4, 10) == pdTRUE) {
			ticks = plotter4.penPos*(penup - pendown)/248 + pendown;
			Chip_SCTPWM_SetDutyCycle(LPC_SCT0, 1, ticks);


		}
		vTaskDelay(configTICK_RATE_HZ/50);
	}
}

static void prvSetupHardware(void)
{
	SystemCoreClockUpdate();
	Board_Init();
	// initialize RIT (= enable clocking etc.)
	Chip_RIT_Init(LPC_RITIMER);
	// set the priority level of the interrupt
	// The level must be equal or lower than the maximum priority specified in FreeRTOS config
	// Note that in a Cortex-M3 a higher number indicates lower interrupt priority
	NVIC_SetPriority( RITIMER_IRQn, 5 );

	/* Initialize PININT driver */
	Chip_PININT_Init(LPC_GPIO_PIN_INT);
	/* Enable PININT clock */
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_PININT);

	/* Reset the PININT block */
	Chip_SYSCTL_PeriphReset(RESET_PININT);

	SetupInt(0, 29, 0);			//Xmin
	SetupInt(0, 9, 1);			//Xmax
	SetupInt(1, 3, 2);			//Ymin
	SetupInt(0, 0, 3);			//Ymax

	/* Enable interrupt in the NVIC */
	NVIC_ClearPendingIRQ(PIN_INT0_IRQn);	//Xmin
	NVIC_EnableIRQ(PIN_INT0_IRQn);

	NVIC_ClearPendingIRQ(PIN_INT1_IRQn);	//Xmax
	NVIC_EnableIRQ(PIN_INT1_IRQn);

	NVIC_ClearPendingIRQ(PIN_INT2_IRQn);	//Ymin
	NVIC_EnableIRQ(PIN_INT2_IRQn);

	NVIC_ClearPendingIRQ(PIN_INT3_IRQn);	//Ymax
	NVIC_EnableIRQ(PIN_INT3_IRQn);
}

void RIT_start(int xcount, int ycount, int us)
{
	uint64_t cmp_value;
	// Determine approximate compare value based on clock rate and passed interval
	cmp_value = (uint64_t) Chip_Clock_GetSystemClockRate() * (uint64_t) us / 1000000;
	// disable timer during configuration
	Chip_RIT_Disable(LPC_RITIMER);
	XRIT_count = xcount;
	YRIT_count = ycount;
	// enable automatic clear on when compare value==timer value
	// this makes interrupts trigger periodically
	Chip_RIT_EnableCompClear(LPC_RITIMER);
	// reset the counter
	Chip_RIT_SetCounter(LPC_RITIMER, 0);
	Chip_RIT_SetCompareValue(LPC_RITIMER, cmp_value);
	// start counting
	Chip_RIT_Enable(LPC_RITIMER);
	// Enable the interrupt signal in NVIC (the interrupt controller)
	NVIC_EnableIRQ(RITIMER_IRQn);
	// wait for ISR to tell that we're done
	if(xSemaphoreTake(sbRIT, portMAX_DELAY) == pdTRUE) {
		// Disable the interrupt signal in NVIC (the interrupt controller)
		NVIC_DisableIRQ(RITIMER_IRQn);
	}
	else {
		// unexpected error
	}
}

void SetupInt(int port, int pin, int index) {


	/* Set pin back to GPIO (on some boards may have been changed to something
		   else by Board_Init()) */
	Chip_IOCON_PinMuxSet(LPC_IOCON, port, pin,
			(IOCON_DIGMODE_EN | IOCON_MODE_PULLUP) );					//cu: IOCON_MODE_INACT

	/* Configure GPIO pin as input */
	Chip_GPIO_SetPinDIRInput(LPC_GPIO, port, pin);

	/* Configure interrupt channel for the GPIO pin in INMUX block */
	Chip_INMUX_PinIntSel(index, port, pin);

	/* Configure channel interrupt as edge sensitive and falling edge interrupt */
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(index));
	Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH(index));
	Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT, PININTCH(index));

}

void GotoPos(DigitalIoPin xdir, DigitalIoPin ydir, int &xcurrent_pulse, int &ycurrent_pulse, double x, double y, double pulseOnwidth, double pulseOnheight) {
	int Xpulse, Ypulse, Xpulse_relative, Ypulse_relative;
	Xpulse = (int) round(x*pulseOnwidth);
	Ypulse = (int) round(y*pulseOnheight);
	if(Xpulse >= xcurrent_pulse) {
		xdir.write(LEFT);
		Xpulse_relative = Xpulse - xcurrent_pulse;
	}
	else {
		xdir.write(RIGHT);
		Xpulse_relative = xcurrent_pulse - Xpulse;
	}

	if(Ypulse >= ycurrent_pulse) {
		ydir.write(LEFT);
		Ypulse_relative = Ypulse - ycurrent_pulse;
	}
	else {
		ydir.write(RIGHT);
		Ypulse_relative = ycurrent_pulse - Ypulse;
	}
	RIT_start(Xpulse_relative, 0, 500);
	RIT_start(0, Ypulse_relative, 500);
	xcurrent_pulse = Xpulse;
	ycurrent_pulse = Ypulse;
}





/* the following is required if runtime statistics are to be collected */
extern "C" {

void vConfigureTimerForRunTimeStats( void ) {
	Chip_SCT_Init(LPC_SCTSMALL1);
	LPC_SCTSMALL1->CONFIG = SCT_CONFIG_32BIT_COUNTER;
	LPC_SCTSMALL1->CTRL_U = SCT_CTRL_PRE_L(255) | SCT_CTRL_CLRCTR_L; // set prescaler to 256 (255 + 1), and start timer
}

void PIN_INT0_IRQHandler(void)			//Xmin limit
{
	portBASE_TYPE xHigherPriorityWoken = pdFALSE;
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(0));
	XRIT_count = 0;
	if(xSemaphoreGiveFromISR(limXmin_sem, &xHigherPriorityWoken) == pdTRUE) {
		//do sth to notify
	}
	Board_LED_Toggle(0);
}

void PIN_INT1_IRQHandler(void)			//Xmax limit
{
	portBASE_TYPE xHigherPriorityWoken = pdFALSE;
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(1));
	XRIT_count = 0;
	if(xSemaphoreGiveFromISR(limXmax_sem, &xHigherPriorityWoken) == pdTRUE) {
		//do sth to notify
	}
	Board_LED_Toggle(1);
}

void PIN_INT2_IRQHandler(void)			//Ymin limit
{
	portBASE_TYPE xHigherPriorityWoken = pdFALSE;
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(2));
	YRIT_count = 0;
	if(xSemaphoreGiveFromISR(limYmin_sem, &xHigherPriorityWoken) == pdTRUE) {
		//do sth to notify
	}
	Board_LED_Toggle(2);
}

void PIN_INT3_IRQHandler(void)			//Ymax limit
{
	portBASE_TYPE xHigherPriorityWoken = pdFALSE;
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(3));
	YRIT_count = 0;
	if(xSemaphoreGiveFromISR(limYmax_sem, &xHigherPriorityWoken) == pdTRUE) {
		//do sth to notify
	}
	Board_LED_Toggle(2);
}

void RIT_IRQHandler(void)
{
	DigitalIoPin XSTEP(0, 24, DigitalIoPin::output, false);
	DigitalIoPin YSTEP(0, 27, DigitalIoPin::output, false);
	// This used to check if a context switch is required
	portBASE_TYPE xHigherPriorityWoken = pdFALSE;
	// Tell timer that we have processed the interrupt.
	// Timer then removes the IRQ until next match occurs
	Chip_RIT_ClearIntStatus(LPC_RITIMER); // clear IRQ flag
	if(XRIT_count > 0) {
		XRIT_count--;
		XSTEP.write((XRIT_count % 2) == 1);
	}
	if(YRIT_count > 0) {
		YRIT_count--;
		YSTEP.write((YRIT_count % 2) == 1);
	}
	if(XRIT_count == 0 && YRIT_count == 0) {
		Chip_RIT_Disable(LPC_RITIMER); // disable timer
		// Give semaphore and set context switch flag if a higher priority task was woken up
		xSemaphoreGiveFromISR(sbRIT, &xHigherPriorityWoken);
	}
	// End the ISR and (possibly) do a context switch
	portEND_SWITCHING_ISR(xHigherPriorityWoken);
}

}












