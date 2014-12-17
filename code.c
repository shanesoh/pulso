// ECE 4760 Final Project
// Acoustic Wayfinding Device with Haptic Feedback for the Visually Impaired
// Shane Soh (xs46) & Eileen Liu (ebl43)

// Notes
// at start of device turn on, point the front sensor vertically to the ground for calibration
// in the code, sonar0 refers to left sonar, 1 = right, 2 = front

#include "trtSettings.h"
#include "trtkernel_1284.c"
#include <util/delay.h>
#include <stdio.h>

// serial communication library
#define SEM_RX_ISR_SIGNAL 1
#define SEM_STRING_DONE 2 
#include "trtUart.h"
#include "trtUart.c"

// UART file descriptor 
FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

// semaphore to protect reading (only one active transceiver/ at a time)
#define SEM_RANGE 3

// ----- Sonar and navigation definitions ------
#define sonarFreq 0.25 			// Freq at which sonar ranges (in seconds)
#define navFreq 0.3 			// Freq at which navigation logic refreshes (in seconds)
#define sonar3FeedbackFreq 0.25  // Freq at which sensor 3's motor refreshes (in seconds)
#define windowSize 3 			// Window size of median filter (odd numbers only)
#define nCalibCycles 3 			// Number of front sonar calibration cycles

float F_calibrated = 0; 		// calibrated vertical distance from front sensor to ground
int firstRun = 1;				// flag for first cycle through tasks (for inits)

// Sonar ranges in meters
float sonarL_range, sonarR_range, sonarF_range; 

// Saved sonar ranges in arrays
float rangeL[windowSize], rangeR[windowSize], rangeF[windowSize];

// Indices for the arrays above (if array not full, do not apply median filter)
uint8_t iL = 0;
uint8_t iR = 0;
uint8_t iF = 0;

// input arguments
int args[4] ;

/* ------------------ Supporting Functions -----------------------------*/
// Reads adc value at pin ch
// Code borrowed from ADC tutorial: http://maxembedded.com/2011/06/20/the-adc-of-the-avr/
// @param ch which pin (which transceiver) to read
//			 0 = left, 1 = right, 2 = front
// @return	 adc-converted reading
uint8_t adc_read(uint8_t ch)
{
    // select the corresponding channel 0~7
    // ANDing with '7' will always keep the value
    // of 'ch' between 0 and 7
    ch &= 0b00000111;  			// AND operation with 7
    ADMUX = (ADMUX & 0xF8)|ch;  // clears the bottom 3 bits before ORing
 
    // start single conversion - write '1' to ADSC
    ADCSRA |= (1<<ADSC);
 
    // wait for conversion to complete
    // ADSC becomes '0' again
    // till then, run loop continuously
    while(ADCSRA & (1<<ADSC));

    return (ADCH);
}

// Takes an array of float values and finds the median value
// @param array float array of size windowSize to 
// @return		median float value of array
float medianFilter(float *array)
{	
	if ( ((array[0] >= array[1]) && (array[0] <= array[2])) || ((array[0] <= array[1]) && (array[0] >= array[2])))
		return array[0];
	else if ( ((array[1] >= array[0]) && (array[1] <= array[2])) || ((array[1] <= array[0]) && (array[1] >= array[2])))
		return array[1];
	else 
		return array[2];
}

// Calibrate the vertical distance (in meters) of the front sensor to the floor
// people typically hold a walking stick at a 45-degree angle
// 		calibrated threshold for front sensor is the hypotenuse of a 45-45-90 triangle,
// 		with the vertical distance as a leg of the triangle
void calibrateFront(void)	
{
	float sonarFront = 0;
	float calibArray[nCalibCycles];
	
	for (int i = 0; i < 2*nCalibCycles; i++)
	{
		//turn on RX of sonar3
		PORTC = PORTC ^ 0x08;
		_delay_ms(80);	

		// read ADC of front sensor and convert to meters
		sonarFront = adc_read(2); 
		calibArray[i%nCalibCycles] = (float)sonarFront * 0.0508;// distance = adc reading * (5/256)*(512/5)*.0254
		
		_delay_ms(30);
		
		//turn off RX
		PORTC = PORTC ^ 0x08;
		_delay_ms(30);
	}
	
	F_calibrated = medianFilter(calibArray) * 1.41; // hypotenuse of 45-45-90 triangle -> sqrt(2) = 1.41

	//fprintf(stdout, "val: %f, %f, %f\n\r", calibArray[0], calibArray[1], calibArray[2]);
	//fprintf(stdout, "val: %f\n\r", F_calibrated);
}

/* ------------------ Real Time Functions -----------------------------*/

// Navigation logic
// send haptic feedback depending on distance from sensors
void navLogic(void* args) 
{	
	// where is the object?
	#define NONE 0
	#define LEFT 1
	#define RIGHT 2

	int L_thresh = 2;
	int R_thresh = 2;

	uint8_t state = NONE;

 	uint32_t rel, dead ;
	
	while(1)
	{
		if (firstRun == 0) // not the first run of this task
		{
			// Median filter 
			if (iL >= 3) sonarL_range = medianFilter(rangeL);
			if (iR >= 3) sonarR_range = medianFilter(rangeR);
			if (iF >= 3) sonarF_range = medianFilter(rangeF); 
		
			// obstacle found within threshold
			if ((sonarL_range < L_thresh) || (sonarR_range < R_thresh)) 
			{
				if (sonarL_range < sonarR_range) 	  state = LEFT;
				else if (sonarR_range < sonarL_range) state = RIGHT;
			}
			else
			{
				state = NONE;
			}
		
			switch (state)
			{
				case LEFT:
				{
					float durationL = 1/sonarL_range * 30;

					//pulse left
					PORTC = PORTC ^ 0x40; // PIN C6
					//_delay_ms(90);
					_delay_ms(durationL);
					PORTC = PORTC ^ 0x40;
		
					state = NONE;
				} break;

				case RIGHT:
				{
					float durationR = 1/sonarR_range * 30;

					//pulse right
					PORTC = PORTC ^ 0x80; // PIN C7
					//_delay_ms(90);
					_delay_ms(durationR);
					PORTC = PORTC ^ 0x80;
		
					state = NONE;		
				} break;
		
				default:
				{} break;
			} // end of -- switch (state)
		} // end of -- if (firstRun == 0)
		
		// Sleep
		rel = trtCurrentTime() + SECONDS2TICKS(navFreq);
		dead = trtCurrentTime() + SECONDS2TICKS(navFreq+.1);
		trtSleepUntil(rel, dead);	
	} // end of -- while
}

// Read the left sonar
void readSonar1(void* args) 
{
	uint32_t rel, dead ;
	uint8_t sonar_adc1;

	while(1)
	{
		if (firstRun == 0) // not the first run of this task
		{
			trtWait(SEM_RANGE);
			
			//turn on RX of sonar1
			PORTC = PORTC ^ 0x02;
			_delay_ms(50);	

			// read ADC0 and convert to meters
			sonar_adc1 = adc_read(0); 
			sonarL_range = (float)sonar_adc1 * 0.0508; // (5/256)*(512/5)*.0254 * adc reading

			//turn off RX
			PORTC = PORTC ^ 0x02;
			_delay_ms(20);

			trtSignal(SEM_RANGE);

			// Save ranges in array
			rangeL[((iL++)%windowSize)] = sonarL_range;
		} // end of -- if (firstRun == 0);

		// Sleep
		rel = trtCurrentTime() + SECONDS2TICKS(sonarFreq);
		dead = trtCurrentTime() + SECONDS2TICKS(sonarFreq);
		trtSleepUntil(rel, dead);
	} // end of -- if (firstRun == 0)
} // end of -- while(1)


// Read the right sonar
void readSonar2(void* args) 
{
	uint32_t rel, dead ;
	uint8_t sonar_adc2;

	while(1)
	{
		if (firstRun == 0) // not the first run of this task
		{
			trtWait(SEM_RANGE);

			//turn on RX of sonar2
			PORTC = PORTC ^ 0x04;
			_delay_ms(50);	

			// read ADC1 and convert to meters
			sonar_adc2 = adc_read(1); 
			sonarR_range = (float)sonar_adc2 * 0.0508;// (5/256)*(512/5)*.0254 * adc reading

			//turn off RX
			PORTC = PORTC ^ 0x04;
			_delay_ms(20);
		
			trtSignal(SEM_RANGE);

			// Save ranges
			rangeR[((iR++)%windowSize)] = sonarR_range;
		} // end of -- if (firstRun == 0)

		// Sleep
		rel = trtCurrentTime() + SECONDS2TICKS(sonarFreq);
		dead = trtCurrentTime() + SECONDS2TICKS(sonarFreq);
		trtSleepUntil(rel, dead);
	} // end of -- while(1)
} 



// Read Sonar3, i.e. fwd sonar
void readSonar3(void* args) 
{
	uint32_t rel, dead ;
	uint8_t sonar_adc3;

	while(1)
	{
		if (firstRun) // not the first run of this task
		{
			// Calibrate front sensor
			calibrateFront();
			firstRun = 0;
		}

		if (firstRun == 0) // not the first run of this task
		{
			trtWait(SEM_RANGE);

			//turn on RX of sonar1
			PORTC = PORTC ^ 0x08;
			_delay_ms(50);	

			// read ADC0 and convert to meters
			sonar_adc3 = adc_read(2); 
			sonarF_range = (float)sonar_adc3 * 0.0508; // (5/256)*(512/5)*.0254 * adc reading

			//turn off RX
			PORTC = PORTC ^ 0x08;
			_delay_ms(20);
			
			trtSignal(SEM_RANGE);
	
			// Save ranges 
			rangeF[((iF++) % windowSize)] = sonarF_range;
		} // end of -- if (fristRun == 0)

		// Sleep
		rel = trtCurrentTime() + SECONDS2TICKS(sonarFreq);
		dead = trtCurrentTime() + SECONDS2TICKS(sonarFreq);
		trtSleepUntil(rel, dead);
	} // end of -- while(1)
}

// front sonar haptic feedback
void sonar3Feedback(void* args) 
{
	uint32_t rel, dead ;
	float duration = 0;
	
	while(1)
	{
		if ((sonarF_range < F_calibrated) && (firstRun == 0))
		{
			duration = 1/sonarF_range * 20;
			PORTC = PORTC ^ 0x20; // PINC5
			_delay_ms(duration);
			PORTC = PORTC ^ 0x20;
		}
		
		// Sleep
		rel = trtCurrentTime() + SECONDS2TICKS(sonar3FeedbackFreq);
		dead = trtCurrentTime() + SECONDS2TICKS(sonar3FeedbackFreq);
		trtSleepUntil(rel, dead);
	}
}

// initialize adc
void adc_init()
{
	// AREF = AVcc
  ADMUX |= (1<<REFS0);
 	ADMUX |= (1<<ADLAR); // left align ADC value in data registers (turn measurement from 10 bits to 8 bits)
  
	// ADC Enable and prescaler of 32
  ADCSRA = (1<<ADEN)|(1<<ADPS2)|(0<<ADPS1)|(1<<ADPS0);
}

// --- Main Program ----------------------------------
int main(void) {
	//init PortC
	//Pins C.1 to C.3 for controlling sonars
	//Pins C.5, C.6 and C.7 for front, left and right motors respectively
	DDRC = 0xff;

	//init ADC
	adc_init();

	//init the UART -- trt_uart_init() is in trtUart.c
	trt_uart_init();
	stdout = stdin = stderr = &uart_str;
	fprintf(stdout,"\n\r Start TRT\n\r\n\r");
		
	// start TRT
	trtInitKernel(80); // 80 bytes for the idle task stack
	
	// --- create semaphores ----------
	trtCreateSemaphore(SEM_RANGE, 1) ; // protect ranging
	
	// --- create tasks  ----------------
	trtCreateTask(readSonar1, 500, SECONDS2TICKS(0.1), SECONDS2TICKS(0.1), &(args[0]));
	trtCreateTask(readSonar2, 500, SECONDS2TICKS(0.1), SECONDS2TICKS(0.1), &(args[1]));
	trtCreateTask(readSonar3, 500, SECONDS2TICKS(0.1), SECONDS2TICKS(0.1), &(args[2]));
	trtCreateTask(navLogic, 400, SECONDS2TICKS(0.1), SECONDS2TICKS(0.1), &(args[3]));
	trtCreateTask(sonar3Feedback, 400, SECONDS2TICKS(0.1), SECONDS2TICKS(0.1), &(args[0]));
	
	// --- Idle task --------------------------------------
	while (1) 
	{
		// Do Nothing
	}
}
