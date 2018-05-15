
#include <asf.h>

#include "ch.h"
#include "xprintf.h"

#include "ff.h"
#include "diskio.h"
#include "TinyGPSPlus-0.95/TinyGPS++.h"

#include "util/inv_mpu.h"
#include "util/inv_mpu_dmp_motion_driver.h"

extern "C"
{
#include "board_definitions.h"

#include "sam_ba_usb.h"
#include "sam_ba_cdc.h"
}

extern uint16_t writeTypeCount[];

extern unsigned int nmeaCount;
extern unsigned int ubxCount;
//extern unsigned int adcCount;
extern unsigned int imuCount;

#define LINE_LEN 128
static char line[LINE_LEN];
static uint16_t lineIdx;

extern "C" void uart1_putc(uint8_t d);
extern "C" unsigned char uart1_getc(void);

extern void put_rc (FRESULT rc);

struct usart_module usart_console;

#define TX_BUFFER_SIZE 1024
#define RX_BUFFER_SIZE 128

typedef struct _ringBuffer
{
	volatile uint8_t rx_buffer[RX_BUFFER_SIZE];
	volatile uint8_t tx_buffer[TX_BUFFER_SIZE];
	volatile uint16_t tx_write_idx;
	volatile uint16_t rx_write_idx;
	volatile uint16_t tx_read_idx;
	volatile uint16_t rx_read_idx;
} ringBuffer;

ringBuffer console_buffer;


extern "C" void uart1_putc(uint8_t d);
extern "C" unsigned char uart1_getc(void);
extern "C" unsigned char cdcUSBgetc(void);

void configure_usart_console(void)
{
	struct usart_config config_usart;
	usart_get_config_defaults(&config_usart);

	config_usart.baudrate = 115200;
	config_usart.mux_setting = EDBG_CDC_SERCOM_MUX_SETTING;
	config_usart.pinmux_pad0 = EDBG_CDC_SERCOM_PINMUX_PAD0;
	config_usart.pinmux_pad1 = EDBG_CDC_SERCOM_PINMUX_PAD1;
	config_usart.pinmux_pad2 = EDBG_CDC_SERCOM_PINMUX_PAD2;
	config_usart.pinmux_pad3 = EDBG_CDC_SERCOM_PINMUX_PAD3;

	while (usart_init(&usart_console, EDBG_CDC_MODULE, &config_usart) != STATUS_OK)
	{
	}
	usart_enable(&usart_console);

	xdev_out(uart1_putc);
	xdev_in(uart1_getc);

	SercomUsart *usart_hw = &(usart_console.hw->USART);

	/* Enable the RX Complete Interrupt */
	usart_hw->INTENSET.reg = SERCOM_USART_INTFLAG_RXC;

	/* Enable Global interrupt for module SERCOM2 */
	system_interrupt_enable(SYSTEM_INTERRUPT_MODULE_SERCOM2);

}

//void usart_read_callback(struct usart_module * const usart_module);
void uart1_putc(uint8_t d)
{
	//uint16_t p = d;
	uint16_t interrupt_status;

	system_interrupt_enter_critical_section();

	SercomUsart *const usart_hw = &(usart_console.hw->USART);

	/* Wait until synchronization is complete */
	_usart_wait_for_sync(&usart_console);

	if (console_buffer.tx_write_idx >= TX_BUFFER_SIZE)
		console_buffer.tx_write_idx = 0;
	console_buffer.tx_buffer[console_buffer.tx_write_idx++] = d;

	// check if we need to re-enable the interrupt
	interrupt_status = usart_hw->INTENSET.reg;
	if (!(interrupt_status & SERCOM_USART_INTFLAG_DRE))
	{
		/* Enable the Data Register Empty Interrupt */
		usart_hw->INTENSET.reg = SERCOM_USART_INTFLAG_DRE;

		if (console_buffer.tx_read_idx >= TX_BUFFER_SIZE)
			console_buffer.tx_read_idx = 0;

		// load the data register, this triggers a DRE interrupt as the USART is double buffered
		usart_hw->DATA.reg = console_buffer.tx_buffer[console_buffer.tx_read_idx++];
	}
	system_interrupt_leave_critical_section();

	//while (usart_write_wait(&usart_console, p) == STATUS_BUSY);
}

unsigned char uart1_getc(void)
{
	uint8_t ch;

	if (console_buffer.rx_read_idx != console_buffer.rx_write_idx)
	{
		if (console_buffer.rx_read_idx >= RX_BUFFER_SIZE)
			console_buffer.rx_read_idx = 0;

		ch = console_buffer.rx_buffer[console_buffer.rx_read_idx++];
	}
	else
		return 0;

	return ch;

}


void SERCOM2_Handler(void)
{
	uint16_t interrupt_status;
	uint8_t d;

	CH_IRQ_PROLOGUE();

	/* Pointer to the hardware module instance */
	SercomUsart *const usart_hw	= &(usart_console.hw->USART);

	/* Wait for the synchronization to complete */
	_usart_wait_for_sync(&usart_console);

	/* Read and mask interrupt flag register */
	interrupt_status = usart_hw->INTFLAG.reg;
	interrupt_status &= usart_hw->INTENSET.reg;

	// check for transmit data register ready
	if (interrupt_status & SERCOM_USART_INTFLAG_DRE)
	{
		// if no more bytes to send, clear interrupt enable
		if (console_buffer.tx_write_idx == console_buffer.tx_read_idx)
		{
			usart_hw->INTENCLR.reg = SERCOM_USART_INTFLAG_DRE;
		}
		else
		{
			// more to send, get next byte from byffer
			if (console_buffer.tx_read_idx >= TX_BUFFER_SIZE)
				console_buffer.tx_read_idx = 0;
			d = console_buffer.tx_buffer[console_buffer.tx_read_idx++];

			usart_hw->DATA.reg = d;
		}
	}

	// check for receive characters
	if (interrupt_status & SERCOM_USART_INTFLAG_RXC)
	{
		if (console_buffer.rx_write_idx >= RX_BUFFER_SIZE)
			console_buffer.rx_write_idx = 0;

		console_buffer.rx_buffer[console_buffer.rx_write_idx++] = usart_hw->DATA.reg;

		chSysLockFromISR();
		chEvtSignalI(&nil.threads[3], 1);
		chSysUnlockFromISR();
	}

	CH_IRQ_EPILOGUE();
}

DIR Dir;					/* Directory object */
FILINFO Finfo;
static FATFS *fs;
FIL File;				/* File object */
extern FIL loggingFile;				/* loggingFile object */

extern struct rtc_module rtc_instance;
extern volatile int fileBufferWriteIdx;
extern volatile int fileBufferReadIdx;
extern volatile char writing;
extern unsigned int date2doy(unsigned int year, unsigned int month, unsigned int day);
extern TinyGPSPlus tgps;
extern bool alarm;

extern void uart2_putc(uint8_t);

void consoleCharRx(char temp)
{
	long p1;
	FRESULT res;
	UINT s1, s2;
	char *ptr;

	//xprintf("console ch %d\n", temp);

	if (temp == '\r')
	{
		line[lineIdx] = 0;
		lineIdx = 0;
		xprintf("\nline : %s\n", line);
		if (strncmp("dir", line, 3) == 0)
		{
			ptr = &line[3];

			while (*ptr == ' ') ptr++;
			res = f_opendir(&Dir, ptr);

			if (res)
			{
				put_rc(res);
				return;
			}
			p1 = s1 = s2 = 0;
			for (;;)
			{
				res = f_readdir(&Dir, &Finfo);
				if ((res != FR_OK) || !Finfo.fname[0])
					break;
				if (Finfo.fattrib & AM_DIR)
				{
					s2++;
				}
				else
				{
					s1++;
					p1 += Finfo.fsize;
				}
				xprintf("%c%c%c%c%c %u/%02u/%02u %02u:%02u %9lu  %s\n",
						(Finfo.fattrib & AM_DIR) ? 'D' : '-',
						(Finfo.fattrib & AM_RDO) ? 'R' : '-',
						(Finfo.fattrib & AM_HID) ? 'H' : '-',
						(Finfo.fattrib & AM_SYS) ? 'S' : '-',
						(Finfo.fattrib & AM_ARC) ? 'A' : '-',
						(Finfo.fdate >> 9) + 1980,
						(Finfo.fdate >> 5) & 15, Finfo.fdate & 31,
						(Finfo.ftime >> 11), (Finfo.ftime >> 5) & 63,
						Finfo.fsize, Finfo.fname);
			}
			xprintf("%4u File(s),%10lu bytes total\n%4u Dir(s) ", s1, p1, s2);
			res = f_getfree("0", (DWORD*) &p1, &fs);
			if (res == FR_OK)
				xprintf(", %10lu bytes free\n", p1 * fs->csize * 512);
			else
				put_rc(res);

		}
		else if (strncmp("buff", line, 4) == 0)
		{
			xprintf("Buffer Read %d Write %d\n", fileBufferReadIdx, fileBufferWriteIdx);
		}
		else if (strncmp("count", line, 5) == 0)
		{
			for (int i=0;i<8;i++)
			{
				xprintf("count %d\n", writeTypeCount[i]);
			}
		}
		else if (strncmp("flush", line, 5) == 0)
		{
			while (writing)
			{
				chThdSleepMilliseconds(1000);
			}
			xprintf("sync...\n");

			put_rc(f_sync(&loggingFile));
		}
		else if (strncmp("thread", line, 5) == 0)
		{
			for(int i=0;i<CH_CFG_NUM_THREADS;i++)
			{
				xprintf("thread %-8s : state %d sp 0x%08lx base 0x%08lx end 0x%08lx used %ld\n", nil_thd_configs[i].namep, nil.threads[i].state,
						nil.threads[i].ctx.sp,
						nil_thd_configs[i].wbase,
						nil_thd_configs[i].wend,
						(long)nil_thd_configs[i].wend - (long)nil.threads[i].ctx.sp) ;
			}
		}
		else if (strncmp("time", line, 4) == 0)
		{
			struct rtc_calendar_time rtc_time;

			ptr = &line[4];

			while (*ptr == ' ') ptr++;
			if (*ptr != '\0')
			{
				struct rtc_calendar_time time;
				time.year   = strtol(ptr, NULL, 10);
				time.month  = strtol(&ptr[5], NULL, 10);
				time.day    = strtol(&ptr[8], NULL, 10);
				time.hour   = strtol(&ptr[11], NULL, 10);
				time.minute = strtol(&ptr[14], NULL, 10);
				time.second = strtol(&ptr[17], NULL, 10);

				/* Set current time. */
				rtc_calendar_set_time(&rtc_instance, &time);

				xprintf("set time %d-%02d-%02d %02d:%02d:%02d\n", time.year, time.month, time.day,
						time.hour, time.minute, time.second);
			}

			rtc_calendar_get_time(&rtc_instance, &rtc_time);
			xprintf("doy %d\n", date2doy(rtc_time.year, rtc_time.month, rtc_time.day));

			xprintf("rtc time %4d-%02d-%02d %02d:%02d:%02d\n", rtc_time.year, rtc_time.month, rtc_time.day,
					rtc_time.hour, rtc_time.minute, rtc_time.second);

			xprintf("gps time %d-%02d-%02d %02d:%02d:%02d\n", tgps.date.year(), tgps.date.month(), tgps.date.day(),
					tgps.time.hour(), tgps.time.minute(), tgps.time.second());

			rtc_calendar_frequency_correction(&rtc_instance, -90);

		}
		else if (strncmp("ubx", line, 3) == 0)
		{
			ubxCount = 4;
		}
		else if (strncmp("nmea", line, 4) == 0)
		{
			nmeaCount = 4;
		}
		else if (strncmp("gps", line, 3) == 0)
		{
			nmeaCount = 4;
			ubxCount = 4;
		}
//		else if (strncmp("adc", line, 3) == 0)
//		{
//			adcCount = 4;
//		}
		else if (strncmp("imu", line, 3) == 0)
		{
			imuCount = 4;
		}
		else if (strncmp("sleep", line, 5) == 0)
		{
//			byte message[] = { 0xb5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x4d, 0x3b};

			struct rtc_calendar_alarm_time rtcAlarm;
			rtcAlarm.time.year = 2013;
			rtcAlarm.time.month = 1;
			rtcAlarm.time.day = 1;
			rtcAlarm.time.hour = 0;
			rtcAlarm.time.minute = 0;
			rtcAlarm.time.second = 0;
			rtcAlarm.mask = RTC_CALENDAR_ALARM_MASK_SEC;

			rtc_calendar_set_alarm(&rtc_instance, &rtcAlarm, RTC_CALENDAR_ALARM_0);

			/* Enable Global interrupt for module RTC */
			system_interrupt_enable(SYSTEM_INTERRUPT_MODULE_RTC);
			rtc_instance.hw->MODE2.INTENSET.bit.ALARM0 = 1;

			// GPS, should move to the gpsThread, because then can send at end of message
//			for(uint8_t i=0;i<sizeof(message);i++)
//				uart2_putc(message[i]);

			// IMU
			//mpu_set_sensors(0);

			// ADC

			alarm = false;
			xprintf("Sleeping...");
			chThdSleepMilliseconds(1000);

			SCB->SCR |= 1<<2; // Enable deep-sleep mode
			EIC->WAKEUP.reg = EIC_WAKEUP_WAKEUPEN11;
			__WFI(); // This is the WFI (Wait For Interrupt) function call.

			xprintf("Woken...\n");

			chThdSleepMilliseconds(10000);

//			mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS); // Restart IMU
//
//			uart2_putc(0); // Wake GPS

			//			imu.setSensors(0); // disable sensors, sets sleep mode
			//
			//			Serial2.write(message, sizeof(message));
			//
						//delay(1000);
			//
			//			imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS); // restart the IMU
			//
			//			Serial2.write((byte)0x0); // this wakes the GPS
		}
		lineIdx = 0;
	}
	else if (lineIdx < LINE_LEN)
	{
		line[lineIdx++] = temp;
	}

}

THD_FUNCTION(ThreadUsart, arg)
{
	(void) arg;

	chThdSleepMilliseconds(100);
	xprintf("UsartThread\n");
	chThdSleepMilliseconds(500);

	int16_t temp;
	while (true)
	{
		chEvtWaitAnyTimeout(ALL_EVENTS, 1000);

		while ((temp = xfunc_in()) != 0)
		{
			consoleCharRx(temp);
			uart1_putc(temp);
		}
	}
}

P_USB_CDC pCdc;

THD_FUNCTION(ThreadUSB, arg)
{
	(void) arg;

	chThdSleepMilliseconds(350);
	xprintf("USBThread\n");
	chThdSleepMilliseconds(450);

	pCdc = usb_init();

	bool connected = false;
	bool configured = false;
	while (true)
	{
		if (pCdc->IsConfigured(pCdc) != 0)
		{
			if (!configured)
			{
				xprintf("USB configured\n");

				configured = true;
			}
			/* Check if a USB enumeration has succeeded and if comm port has been opened */
			while (pCdc->currentConnection != 0)
			{
				if (!connected)
				{
					xprintf("USB connected\n");

					cdc_write_buf("USB connected\r\n\n", sizeof("USB connected\r\n\n"));
					connected = true;

					xdev_out(cdc_putc);
					chThdSleepMilliseconds(200);
					xprintf("USB set input\n");

					//xdev_in(cdc_getc);
				}
				bool rdy = cdc_is_rx_ready();
				uint16_t ch = cdc_getc();

				if (rdy)
				{
					//xprintf("USB rdy ch\n");
					if (ch != 0)
					{
						//xprintf("CDC char %d\n", ch);

						cdc_putc(ch);

						consoleCharRx(ch);
					}
				}

				chThdSleepMilliseconds(10);
			}

			if (connected)
			{
				xdev_out(uart1_putc);
				xdev_in(uart1_getc);

				xprintf("USB connection dropped\n");
				connected = false;
			}

			chThdSleepMilliseconds(100);
		}

		chThdSleepMilliseconds(100);
	}

}

