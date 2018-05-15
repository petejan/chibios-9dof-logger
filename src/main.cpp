/*
 Chibios-SAMD-datalogger - Copyright (C) 2017 Pete Jansen

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

 http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
 */

#include <asf.h>

#include "ch.h"

#include <pio/samd21j18a.h>
#include <port.h>

#include "user_board.h"

#include "xprintf.h"

#include "ff.h"
#include "diskio.h"

#include <string.h>

extern uint16_t nmiCount;

// GPS uart sercom1
// EDBG Console is sercom2

// I2C interface is sercom3
// SPI interface is sercom4 to sd-card

#if !defined(SYSTEM_CLOCK)
#define SYSTEM_CLOCK 8000000U
#endif

//#define SERIAL_BUFFERS_SIZE 128

//static uint8_t ibuf[SERIAL_BUFFERS_SIZE];
//static volatile uint8_t iRxInIdx;
//static volatile uint8_t iRxOutIdx;

unsigned int date2doy(unsigned int year, unsigned int month, unsigned int day)
{
	unsigned int doy = day;

	int daysMonth[12] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};

	if( month >= 3 && ((year % 400 == 0) || ((year % 4 == 0) && (year % 100 != 0)) ))
	  doy++;

	doy += daysMonth[month-1];

	return doy;
}


void SERCOM0_Handler(void)
{
}

#define FILE_BUFFER_SIZE 8192
static uint8_t fileBuffer[FILE_BUFFER_SIZE];
volatile int fileBufferWriteIdx = 0;
volatile int fileBufferReadIdx = 0;
volatile char writing = 0;

void writeFileChar(char c)
{
	fileBuffer[fileBufferWriteIdx++] = c;
	if (fileBufferWriteIdx >= FILE_BUFFER_SIZE)
		fileBufferWriteIdx = 0;
}

#define TYPE_COUNT 8
uint16_t writeTypeCount[TYPE_COUNT];
extern "C" void writeFile(uint8_t type, int len, const void *data);

void writeFile(uint8_t type, int len, const void *data)
{
	system_interrupt_enter_critical_section();

	uint8_t *p = (uint8_t *)data;
	uint8_t sum = 0;

	if (type < TYPE_COUNT)
		writeTypeCount[type]++;

	writeFileChar(type);
	writeFileChar(len & 0xff); // samd21 is little endian
	writeFileChar(len >> 8);

	while (len > 0)
	{
		sum += *p;
		writeFileChar(*p++);
		len--;
	}
	writeFileChar(sum);

	system_interrupt_leave_critical_section();
}

extern "C"
{
void delay_ms(unsigned long);
void delay_ms(unsigned long num_ms)
{
	chThdSleepMilliseconds(num_ms);
}
}

//
// RTC
//

void configure_rtc_calendar(void);

struct rtc_module rtc_instance;

void configure_rtc_calendar(void)
{
	struct rtc_calendar_config config_rtc_calendar;
	rtc_calendar_get_config_defaults(&config_rtc_calendar);

	struct rtc_calendar_time alarm;
	rtc_calendar_get_time_defaults(&alarm);
	alarm.year   = 2017;
	alarm.month  = 1;
	alarm.day    = 2;
	alarm.hour   = 0;
	alarm.minute = 0;
	alarm.second = 0;

	config_rtc_calendar.alarm[0].time = alarm;
	config_rtc_calendar.alarm[0].mask = RTC_CALENDAR_ALARM_MASK_YEAR;
	config_rtc_calendar.clock_24h 	  = true;

	rtc_calendar_init(&rtc_instance, RTC, &config_rtc_calendar);

	rtc_calendar_enable(&rtc_instance);
}

uint32_t get_fattime(void);

uint32_t get_fattime(void)
{
	uint32_t ul_time;
	struct rtc_calendar_time current_time;

	/* Retrieve date and time */
	rtc_calendar_get_time(&rtc_instance, &current_time);

	ul_time = ((current_time.year - 1980) << 25)
 			 | (current_time.month << 21)
			 | (current_time.day << 16)
			 | (current_time.hour << 11)
			 | (current_time.minute << 5)
		    | ((current_time.second >> 1) << 0);

	return ul_time;
}
extern "C" void disk_timerproc (void);

/*
 * @brief   System Timer handler.
 */
CH_IRQ_HANDLER(SysTick_Handler)
{
	CH_IRQ_PROLOGUE();

	//port_pin_set_output_level(LED_0_PIN, true);

	disk_timerproc();

	chSysLockFromISR();

	// NMI is used to wake the IMU thread, using this code in NMI_Handler hangs sometimes
	if (nmiCount > 0)
	{
		chEvtSignalI(&nil.threads[0], 1);
		nmiCount = 0;
	}
	chSysTimerHandlerI();
	chSysUnlockFromISR();

	//port_pin_set_output_level(LED_0_PIN, false);

	CH_IRQ_EPILOGUE();

}

//static uint32_t seconds_counter;
static uint32_t minutes_counter;

void put_rc (FRESULT rc)
{
	const char *str =
		"OK\0" "DISK_ERR\0" "INT_ERR\0" "NOT_READY\0" "NO_FILE\0" "NO_PATH\0"
		"INVALID_NAME\0" "DENIED\0" "EXIST\0" "INVALID_OBJECT\0" "WRITE_PROTECTED\0"
		"INVALID_DRIVE\0" "NOT_ENABLED\0" "NO_FILE_SYSTEM\0" "MKFS_ABORTED\0" "TIMEOUT\0"
		"LOCKED\0" "NOT_ENOUGH_CORE\0" "TOO_MANY_OPEN_FILES\0" "INVALID_PARAMETER\0";
	UINT i;

	for (i = 0; i != (UINT)rc && *str; i++)
	{
		while (*str++) ;
	}
	xprintf("rc=%u FR_%s\n", (UINT)rc, str);
}

#define TASK_STACK 1024

/*
 * Minutes counter thread.
 */
static THD_WORKING_AREA(waThreadMinute, TASK_STACK);
static THD_FUNCTION(ThreadMinute, arg)
{
	(void) arg;
	struct rtc_calendar_time time;
	//int16_t lastHour = -1;

	chThdSleepMilliseconds(200);
	xprintf("MinuteThread\n");
	chThdSleepMilliseconds(500);

	while (true)
	{
//		xprintf("Thread Minutes %d\r\n", minutes_counter);
//		for (int i=0;i<TYPE_COUNT;i++)
//		{
//			xprintf("count %d\n", writeTypeCount[i]);
//		}

		rtc_calendar_get_time(&rtc_instance, &time);

		xprintf("time %4d-%02d-%02d %02d:%02d:%02d\n", time.year, time.month, time.day, time.hour, time.minute, time.second);

		writeFile(0x04, sizeof(time), &time);

		chThdSleepSeconds(60);

		minutes_counter++;
	}
}

DWORD AccSize;				/* Work register for fs command */
WORD AccFiles, AccDirs;

static FATFS fatFS;
FIL loggingFile;				/* loggingFile object */

static THD_WORKING_AREA(waThreadUsart, TASK_STACK);
#define BUFFER_TO_WRITE 1024

static const char startString[] = "Startup";
static const char newFileString[] = "newFile";

extern bool setTime;
static THD_WORKING_AREA(waThreadSD, TASK_STACK);
static THD_FUNCTION(ThreadSD, arg)
{
	(void) arg;
	struct rtc_calendar_time time;

	FRESULT res;
	char fileName[36];
	char dirName[36];
	int lastMin = -1;
	int lastHour = -1;

	//DRESULT dr;
	//uint16_t ioctrldata[8];

	writeFile(0x07, sizeof(startString), startString);

	chThdSleepMilliseconds(400);
	xprintf("SD Thread\n");
	chThdSleepMilliseconds(500);

	//uint8_t pdrv = 0;
	//DSTATUS ds;
	unsigned int s2;

    /* Initialization */

	xprintf("-I- Mount disk %d\n\r", (int)0);

	/* Clear file system object */
	memset(&fatFS, 0, sizeof(FATFS));
	res = f_mount(&fatFS, "", 1);
	if (res != FR_OK)
	{
		xprintf("-E- f_mount pb: 0x%X\n\r", res);
	}
	chThdSleepMilliseconds(2000);

	rtc_calendar_get_time(&rtc_instance, &time);

	// Create file name from this time??, wait 1 second for GPS?

	int doy = date2doy(time.year, time.month, time.day);
	xsprintf(dirName, "%04d-%03d", time.year, doy);
	xsprintf(fileName, "%s/DATA %04d-%02d-%02d H%02d.TXT", dirName, time.year, time.month, time.day, time.hour);

	res = f_mkdir(dirName);
	xprintf("mkdir... %s rc=%d\n", dirName, res);

	lastMin = time.minute;
	lastHour = time.hour;

	res = f_open(&loggingFile, fileName, FA_OPEN_APPEND | FA_WRITE);
	xprintf("Open... %s rc=%d\n", fileName, res);

	int bytesInBuffer;
	while (true)
	{
		bytesInBuffer = fileBufferWriteIdx - fileBufferReadIdx;

		if (bytesInBuffer <= 0)
			bytesInBuffer += FILE_BUFFER_SIZE;

		//xprintf("BytesInBuffer %d read Idx %d\n", bytesInBuffer, fileBufferReadIdx);

		while (bytesInBuffer > BUFFER_TO_WRITE)
		{
			writing = 1;
			//xprintf("Write...\n");
			port_pin_set_output_level(LED_0_PIN, LED_0_INACTIVE);

			res = f_write(&loggingFile, &fileBuffer[fileBufferReadIdx], BUFFER_TO_WRITE, &s2);
			if (res != FR_OK) // count these
			{
				xprintf("Write error %d\n", res);
			}
			else
			{
				//port_pin_set_output_level(LED_0_PIN, LED_0_INACTIVE);
			}
			//xprintf("Write %d\n", s2);

			writing = 0;
			fileBufferReadIdx += BUFFER_TO_WRITE;
			if (fileBufferReadIdx >= FILE_BUFFER_SIZE)
				fileBufferReadIdx = 0;

			bytesInBuffer -= BUFFER_TO_WRITE;

			//xprintf("Write Remaining %d\n", bytesInBuffer);

			chThdSleepMilliseconds(10);
		}

		rtc_calendar_get_time(&rtc_instance, &time);

		if (time.minute != lastMin)
		{
			// TODO: how to break file at packet?
			// can we just write the last of the buffer to the file,
			// then have to write rest to align to 512 byte boundary, or reset the pointers
			doy = date2doy(time.year, time.month, time.day);
			xsprintf(dirName, "%04d-%03d", time.year, doy);
			xsprintf(fileName, "%s/DATA %04d-%02d-%02d H%02d.TXT", dirName, time.year, time.month, time.day, time.hour);
			if ((lastHour != time.hour))
			{
				writeFile(0x06, sizeof(writeTypeCount), &writeTypeCount);
				for(int i=0;i<TYPE_COUNT;i++)
				{
					writeTypeCount[i] = 0;
				}

				bytesInBuffer = fileBufferWriteIdx - fileBufferReadIdx;

				if (bytesInBuffer <= 0)
					bytesInBuffer += FILE_BUFFER_SIZE;

				lastHour = time.hour;

				// write the remaining buffer
				res = f_write(&loggingFile, &fileBuffer[fileBufferReadIdx], bytesInBuffer, &s2);
				if (res != FR_OK)
				{
					xprintf("Write Remaining Error %d\n", res);
				}
				else
				{
					//port_pin_set_output_level(LED_0_PIN, LED_0_INACTIVE);
				}
				xprintf("Write remaining %d\n", s2);

				xprintf("Open new file : %s\n", fileName);

				// restart buffers
				fileBufferReadIdx = 0;
				fileBufferWriteIdx = 0;
				writeFile(0x07, sizeof(newFileString), newFileString);

				// reset the time from GPS
				//if (time.hour == 0)
				//	setTime = true;
			}
			res = f_close(&loggingFile);
			xprintf("Close.. rc=%d\n", res);

			res = f_mkdir(dirName);
			xprintf("mkdir... %s rc=%d\n", dirName, res);
			res = f_open(&loggingFile, fileName, FA_OPEN_APPEND | FA_WRITE);
			xprintf("Open... %s rc=%d\n", fileName, res);

			xprintf("file pointer %d\n", loggingFile.fptr);

			lastMin = time.minute;
		}

		chThdSleepMilliseconds(10);

//		const char msg[] = "The quick brown fox jumps over the lazy dog\n";
//		writeFile(0x01, sizeof(msg), msg);
	}
}
struct port_config pin_conf;
static THD_WORKING_AREA(waThreadGPS, TASK_STACK);
static THD_WORKING_AREA(waThreadADC, TASK_STACK);
#ifdef ADA_FEATHER_M0
static THD_WORKING_AREA(waThreadFX, TASK_STACK);
#else
static THD_WORKING_AREA(waThreadMPU, TASK_STACK);
#endif
static THD_WORKING_AREA(waThreadUSB, TASK_STACK);
#ifdef ADA_FEATHER_M0
extern THD_FUNCTION(ThreadFX, arg);
#else
extern THD_FUNCTION(ThreadMPU, arg);
#endif
extern THD_FUNCTION(ThreadGPS, arg);
extern THD_FUNCTION(ThreadUSB, arg);
extern THD_FUNCTION(ThreadUsart, arg);

extern "C" THD_FUNCTION(ThreadADC, arg);

extern void 	configure_usart_console();

bool alarm = false;
void RTC_Handler(void)
{
	rtc_instance.hw->MODE2.INTENCLR.bit.ALARM0 = 1;
	alarm = true;
	xprintf("RTC Interrupt\n");
}

/*
 * Application entry point.
 */
int main(void)
{
	struct system_clock_source_dfll_config config_dfll;
	system_clock_source_dfll_get_config_defaults(&config_dfll);
	config_dfll.on_demand       = false; // this is a bug in SAMD21, Errata reference: 9905
	system_clock_source_dfll_set_config(&config_dfll);

	system_init();

	/* Configure LEDs as outputs, turn them off */
	pin_conf.direction = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(LED_0_PIN, &pin_conf);
	port_pin_set_output_level(LED_0_PIN, LED_0_INACTIVE);

#ifdef ADA_FEATHER_M0
	// inputs for the interrupt from the FXOS and FXSA IMU
	pin_conf.direction = PORT_PIN_DIR_INPUT;
	port_pin_set_config(PIN_PB08, &pin_conf);
	port_pin_set_config(PIN_PB09, &pin_conf);
#endif


//	port_pin_set_config(PIN_PA06, &pin_conf);
//	port_pin_set_output_level(PIN_PA06, false);

	/*
	 * Hardware initialization, in this simple demo just the systick timer is
	 * initialized.
	 */
	uint32_t sysclock = system_cpu_clock_get_hz();
	SysTick->LOAD = sysclock / CH_CFG_ST_FREQUENCY - (systime_t)1;
	//SysTick->LOAD = SYSTEM_CLOCK / CH_CFG_ST_FREQUENCY - (systime_t)1;
	SysTick->VAL = (uint32_t)0;
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk;

	// how to detect if clock is already set, recovery from reset vs power on?
	struct rtc_calendar_time time;
	time.year   = 2000;
	time.month  = 01;
	time.day    = 01;
	time.hour   = 00;
	time.minute = 00;
	time.second = 00;

	configure_rtc_calendar();

	/* Set current time. */
	rtc_calendar_set_time(&rtc_instance, &time);

	system_interrupt_enable_global();

	configure_usart_console();

	//  uint8_t string[] = "Hello World!\r\n";
	//  usart_write_buffer_wait(&usart_console, string, sizeof(string));
#ifdef ADA_FEATHER_M0
	xprintf("\nAdalogger Feather M0 ChibiOS NIL v" CH_KERNEL_VERSION "\r\n");
#else
	xprintf("\n9DOF IMU, GPS logger ChibiOS NIL v" CH_KERNEL_VERSION "\r\n");
#endif
	xprintf("Device ID %d\r\n", system_get_device_id());
	xprintf("ATMEL xdk-asf-3.34.2\n");

	xprintf("Build " __DATE__ " " __TIME__ "\n\n");

	/*
	 * System initializations.
	 * - Kernel initialization, the main() function becomes a thread and the
	 *   RTOS is active.
	 */
	chSysInit();

	/* This is now the idle thread loop, you may perform here a low priority
	 task but you must never try to sleep or wait in this loop. Note that
	 this tasks runs at the lowest priority level so any instruction added
	 here will be executed after all other tasks have been started.*/
	while (true)
	{
	}
}

/*
 * Threads static table, one entry per thread. The number of entries must
 * match NIL_CFG_NUM_THREADS.
 */
THD_TABLE_BEGIN
#ifdef ADA_FEATHER_M0
THD_TABLE_ENTRY(waThreadFX,     "FX",      ThreadFX, NULL)
#else
THD_TABLE_ENTRY(waThreadMPU,    "MPU",     ThreadMPU, NULL)
#endif
THD_TABLE_ENTRY(waThreadGPS,    "GPS",     ThreadGPS, NULL)
THD_TABLE_ENTRY(waThreadADC,    "ADC",     ThreadADC, NULL)
THD_TABLE_ENTRY(waThreadUsart,  "USART",   ThreadUsart, NULL)
THD_TABLE_ENTRY(waThreadUSB,    "USB",     ThreadUSB, NULL)
THD_TABLE_ENTRY(waThreadMinute, "Minutes", ThreadMinute, NULL)
THD_TABLE_ENTRY(waThreadSD,     "SD",      ThreadSD, NULL)
THD_TABLE_END

