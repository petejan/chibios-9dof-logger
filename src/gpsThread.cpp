
#include <asf.h>

#include "ch.h"

#include <pio/samd21j18a.h>
#include <port.h>

#include "user_board.h"

#include "xprintf.h"

#include "TinyGPSPlus-0.95/TinyGPS++.h"

#include <string.h>

void configure_usart_gps(void);
extern "C" void writeFile(char , int , const void *);
extern struct rtc_module rtc_instance;

struct usart_module usart_gps;

#define TX_BUFFER_SIZE 64
//#define RX_BUFFER_SIZE 128

typedef struct _ringBuffer
{
//	volatile uint8_t rx_buffer[RX_BUFFER_SIZE];
	volatile uint8_t tx_buffer[TX_BUFFER_SIZE];
	volatile uint16_t tx_write_idx;
//	volatile uint16_t rx_write_idx;
	volatile uint16_t tx_read_idx;
//	volatile uint16_t rx_read_idx;
} ringBuffer;

ringBuffer gps_buffer;

void configure_usart_gps()
{
	struct usart_config config_usart;
	usart_get_config_defaults(&config_usart);

	config_usart.baudrate = 115200;
	config_usart.mux_setting = USART_RX_0_TX_2_XCK_3;
	config_usart.pinmux_pad0 = PINMUX_PA16C_SERCOM1_PAD0;
	config_usart.pinmux_pad1 = PINMUX_UNUSED;
	config_usart.pinmux_pad2 = PINMUX_PA18C_SERCOM1_PAD2;
	config_usart.pinmux_pad3 = PINMUX_UNUSED;

	while (usart_init(&usart_gps, SERCOM1, &config_usart) != STATUS_OK)
	{
	}

	struct system_pinmux_config config_pinmux;
	system_pinmux_get_config_defaults(&config_pinmux);

	config_pinmux.mux_position = MUX_PA16C_SERCOM1_PAD0;
	config_pinmux.direction = SYSTEM_PINMUX_PIN_DIR_INPUT;
	config_pinmux.input_pull = SYSTEM_PINMUX_PIN_PULL_UP;

	system_pinmux_pin_set_config(PIN_PA16, &config_pinmux);

	//usart_register_callback(&usart_gps, usart_read_callback, USART_CALLBACK_BUFFER_RECEIVED);
	//usart_enable_callback(&usart_gps, USART_CALLBACK_BUFFER_RECEIVED);

	usart_enable(&usart_gps);

	SercomUsart *usart_hw = &(usart_gps.hw->USART);

	/* Enable the RX Complete Interrupt */
	usart_hw->INTENSET.reg = SERCOM_USART_INTFLAG_RXC;

	/* Enable Global interrupt for module SERCOM1 */
	//system_interrupt_enable(SYSTEM_INTERRUPT_MODULE_SERCOM1);
}


void uart2_putc(uint8_t d)
{
	//uint16_t p = d;
	uint16_t interrupt_status;

	system_interrupt_enter_critical_section();

	SercomUsart *const usart_hw = &(usart_gps.hw->USART);

	/* Wait until synchronization is complete */
	_usart_wait_for_sync(&usart_gps);

	if (gps_buffer.tx_write_idx >= TX_BUFFER_SIZE)
		gps_buffer.tx_write_idx = 0;

	gps_buffer.tx_buffer[gps_buffer.tx_write_idx++] = d;

	// check if we need to re-enable the interrupt
	interrupt_status = usart_hw->INTENSET.reg;
	if (!(interrupt_status & SERCOM_USART_INTFLAG_DRE))
	{
		/* Enable the Data Register Empty Interrupt */
		usart_hw->INTENSET.reg = SERCOM_USART_INTFLAG_DRE;

		// load the data register, this triggers a DRE interrupt as the USART is double buffered
		usart_hw->DATA.reg = gps_buffer.tx_buffer[gps_buffer.tx_read_idx++];
		if (gps_buffer.tx_read_idx >= TX_BUFFER_SIZE)
			gps_buffer.tx_read_idx = 0;
	}
	system_interrupt_leave_critical_section();

	//while (usart_write_wait(&usart_gps, p) == STATUS_BUSY);
}

#define DMA

volatile char *curr_desc;
volatile char *cur_ptr = NULL;

COMPILER_ALIGNED(16)
DmacDescriptor gps_dma_descriptor_rx1 SECTION_DMAC_DESCRIPTOR;
DmacDescriptor gps_dma_descriptor_rx2 SECTION_DMAC_DESCRIPTOR;

volatile char *last_rx_desc = NULL;
volatile char *desc_to_send = NULL;
volatile char *rx_desc = NULL;

struct dma_resource usart_dma_resource_rx;
#define BUFFER_LEN 512
char string1[BUFFER_LEN];
char string2[BUFFER_LEN];

int16_t uart2_getc(void)
{
	//uint8_t ch;

#ifdef DMA
	if (curr_desc == desc_to_send)
	{
		if (cur_ptr == desc_to_send)
		{
			return -1;
		}
	}
	else
	{
//		if (last_rx_desc == rx_desc)
//		{
//			return -1;
//		}
		//port_pin_set_output_level(LED_0_PIN, 0);

		curr_desc = desc_to_send;
		cur_ptr = desc_to_send;
		cur_ptr -= BUFFER_LEN;
	}
	return *cur_ptr++;

//	if (last_rx_desc == &gps_dma_descriptor_rx1)
//	{
//		_write(0, string1, BUFFER_LEN);
//	}
//	if (last_rx_desc == &gps_dma_descriptor_rx2)
//	{
//		_write(0, string2, BUFFER_LEN);
//	}
#else
	if (gps_buffer.rx_read_idx != gps_buffer.rx_write_idx)
	{
		if (gps_buffer.rx_read_idx >= RX_BUFFER_SIZE)
			gps_buffer.rx_read_idx = 0;

		ch = gps_buffer.rx_buffer[gps_buffer.rx_read_idx++];
	}
	else
		return -1;

	return ch;
#endif

}

unsigned int usart_gps_errors = 0;
uint16_t usart_gps_lasterror = 0;

volatile uint32_t rx_count = 0;

void SERCOM1_Handler(void)
{
	uint16_t interrupt_status;
	uint8_t d;

	CH_IRQ_PROLOGUE();

	/* Pointer to the hardware module instance */
	SercomUsart *const usart_hw	= &(usart_gps.hw->USART);

	/* Wait for the synchronization to complete */
	_usart_wait_for_sync(&usart_gps);

	// read STATS reg, to check for errors
	uint16_t status = usart_hw->STATUS.reg;
	if ((status & 0x0f) != 0)
	{
		usart_gps_lasterror = status;
		usart_hw->STATUS.reg = 0xff; // clear any error bits
		usart_gps_errors ++;
	}

#ifdef DMA
	rx_desc = (volatile char*)_write_back_section[0].DSTADDR.reg;
	if (last_rx_desc != rx_desc)
	{
		port_pin_set_output_level(LED_0_PIN, 1);

		rx_count++;
		if (last_rx_desc != NULL)
		{
			desc_to_send = last_rx_desc;
		}
		last_rx_desc = rx_desc;

	}
	/* Read and mask interrupt flag register */
	interrupt_status = usart_hw->INTFLAG.reg;
	interrupt_status &= usart_hw->INTENSET.reg;

	//if (interrupt_status & SERCOM_USART_INTFLAG_RXC)
	{
		chSysLockFromISR();
		chEvtSignalI(&nil.threads[1], 1);
		chSysUnlockFromISR();
	}

	// check for transmit data register ready
	if (interrupt_status & SERCOM_USART_INTFLAG_DRE)
	{
		// if no more bytes to send, clear interrupt enable
		if (gps_buffer.tx_write_idx == gps_buffer.tx_read_idx)
		{
			usart_hw->INTENCLR.reg = SERCOM_USART_INTFLAG_DRE;
		}
		else
		{
			// more to send, get next byte from byffer
			d = gps_buffer.tx_buffer[gps_buffer.tx_read_idx++];
			if (gps_buffer.tx_read_idx >= TX_BUFFER_SIZE)
				gps_buffer.tx_read_idx = 0;

			usart_hw->DATA.reg = d;
		}
	}
#else

	// check for receive characters
	if (interrupt_status & SERCOM_USART_INTFLAG_RXC)
	{
		//port_pin_set_output_level(LED_0_PIN, true);

		if (gps_buffer.rx_write_idx >= RX_BUFFER_SIZE)
			gps_buffer.rx_write_idx = 0;

		gps_buffer.rx_buffer[gps_buffer.rx_write_idx++] = usart_hw->DATA.reg;

		// read STATS reg, to check for errors
		uint16_t status = usart_hw->STATUS.reg;
		if ((status & 0x0f) != 0)
		{
			usart_gps_lasterror = status;
			usart_hw->STATUS.reg = 0xff; // clear any error bits
			usart_gps_errors ++;
		}

		chSysLockFromISR();
		chEvtSignalI(&nil.threads[1], 1);
		chSysUnlockFromISR();

		//port_pin_set_output_level(LED_0_PIN, false);

	}
#endif
	CH_IRQ_EPILOGUE();
}

uint8_t gpsMessage[512];

unsigned int nmeaCount = 0;
unsigned int ubxCount = 0;
unsigned int ubxSumError = 0;
unsigned int gpsMsgIdx = 0;
unsigned int gpsLen = 0;

TinyGPSPlus tgps;
volatile bool setTime = true;

static void configure_dma_resource_rx(struct dma_resource *resource)
{
	struct dma_resource_config config;

	dma_get_config_defaults(&config);

	config.peripheral_trigger = SERCOM1_DMAC_ID_RX; // EDBG_CDC_SERCOM_DMAC_ID_RX;
	config.trigger_action = DMA_TRIGGER_ACTION_BEAT;

	config.event_config.event_output_enable = true;

	dma_allocate(resource, &config);
}

static void setup_transfer_descriptor_rx(DmacDescriptor *descriptor, void *data)
{
	struct dma_descriptor_config descriptor_config;

	dma_descriptor_get_config_defaults(&descriptor_config);

	descriptor_config.beat_size = DMA_BEAT_SIZE_BYTE;
	descriptor_config.src_increment_enable = false;
	descriptor_config.block_transfer_count = BUFFER_LEN;
	descriptor_config.destination_address =	(uint32_t)data + BUFFER_LEN; // who knows why this is the end of the buffer address
	descriptor_config.source_address = (uint32_t)(&usart_gps.hw->USART.DATA.reg);

	dma_descriptor_create(descriptor, &descriptor_config);
}

THD_FUNCTION(ThreadGPS, arg)
{
	(void) arg;

	chThdSleepMilliseconds(300);
	xprintf("GPSThread\n");
	chThdSleepMilliseconds(500);

	configure_usart_gps();

	configure_dma_resource_rx(&usart_dma_resource_rx);

	setup_transfer_descriptor_rx(&gps_dma_descriptor_rx1, string1);
	setup_transfer_descriptor_rx(&gps_dma_descriptor_rx2, string2);

	gps_dma_descriptor_rx2.DESCADDR.reg = (uint32_t)&gps_dma_descriptor_rx1; //this creates a circular buffer

	dma_add_descriptor(&usart_dma_resource_rx, &gps_dma_descriptor_rx1);
	dma_add_descriptor(&usart_dma_resource_rx, &gps_dma_descriptor_rx2);

	dma_start_transfer_job(&usart_dma_resource_rx);

	system_interrupt_enable(SYSTEM_INTERRUPT_MODULE_SERCOM1);

	uint8_t msgType = 0;
	int16_t temp;
	while (true)
	{
		//port_pin_set_output_level(LED_0_PIN, LED_0_INACTIVE);

		chEvtWaitAnyTimeout(ALL_EVENTS, TIME_INFINITE);

		port_pin_set_output_level(LED_0_PIN, LED_0_INACTIVE);

		while ((temp = uart2_getc()) != -1)
		{
			//xprintf("gps %d char %d idx %d\n", msgType, temp, gpsMsgIdx);

			// should just save chars in buffer until buffer is full, or a end character is received
			// then write the buffer to SD card
			// no need for file types NMEA and UBX, just need GPS, would this do for a general serial receiver?
			if (gpsMsgIdx < (sizeof(gpsMessage) - 1))
			{
				gpsMessage[gpsMsgIdx++] = temp;
			}
			else
			{
				// this should probably trigger a write to file
				gpsMsgIdx = 0;
			}
			// need to write buffer regardless of error or not
			switch (msgType)
			{
				case 0:
					if (temp == '$') // NMEA message
					{
						tgps.encode(temp);
						msgType = 1;
					}
					else if (temp == 0xb5) // UBX message
					{
						msgType = 2;
					}
					else
					{
						xprintf("gps unknown char %d idx %d\n", temp, gpsMsgIdx);
						msgType = 0;
						gpsMsgIdx = 0;
					}
					//xprintf("gps %d char %d idx %d\n", msgType, temp, gpsMsgIdx);
					break;
				case 1: // NMEA message, maximum length is 82 bytes http://www.catb.org/gpsd/NMEA.html
					tgps.encode(temp);
					if (temp == 10) // LF
					{
						gpsMessage[gpsMsgIdx] = 0;
						writeFile(0x00, gpsMsgIdx, gpsMessage);

						gpsMessage[20] = 0; // truncate message for printing
						if (nmeaCount > 0)
						{
							xprintf("NMEA %d %-20s failed %d usart errors %d\n", gpsMsgIdx, gpsMessage, tgps.failedChecksum(), usart_gps_errors);
							nmeaCount--;
						}

						if (tgps.date.isValid())
						{
							if (setTime)
							{
								xprintf("gps date %d-%02d-%02d %02d:%02d:%02d\n", tgps.date.year(), tgps.date.month(), tgps.date.day(),
										tgps.time.hour(), tgps.time.minute(), tgps.time.second());

								struct rtc_calendar_time time;
								time.year   = tgps.date.year();
								time.month  = tgps.date.month();
								time.day    = tgps.date.day();
								time.hour   = tgps.time.hour();
								time.minute = tgps.time.minute();
								time.second = tgps.time.second();

								/* Set current time. */
								rtc_calendar_set_time(&rtc_instance, &time);

								setTime = false;
							}
						}

						gpsMsgIdx = 0;
						msgType = 0;
					}
					// check temp == \n (end of NMEA message)
					break;
				case 2:
					if (gpsMsgIdx == 1)
					{
						xprintf("UBX Msg Error\n");
						if (temp != 0x62)
						{
							gpsMsgIdx = 0;
							gpsMessage[gpsMsgIdx++] = temp;
							msgType = 0;
						}
					}
					else
					{
						if (gpsMsgIdx <= 6) // store the data until we get the length
						{
							if (gpsMsgIdx == 6)
							{
								gpsLen = (int) gpsMessage[4] + (((int) gpsMessage[5]) * (int) 256) + 8; // including SYNC(2) CLASS ID LEN(2) CK(2)

								if (ubxCount > 0)
									xprintf("UBX msg Len %d\n", gpsLen);

								if (gpsLen > sizeof(gpsMessage))
								{
									xprintf("UBX msg too long : Len %d\n", gpsLen);
									msgType = 0;
									gpsMsgIdx = 0;
								}
							}
						}
						else if (gpsMsgIdx >= gpsLen)
						{
							writeFile(0x01, gpsMsgIdx, gpsMessage);

							uint8_t CK_A = 0, CK_B = 0;
							for(uint16_t I=2;I<gpsLen-2;I++)
							{
							    CK_A = CK_A + gpsMessage[I];
							    CK_B = CK_B + CK_A;
							}
							if (!(CK_A == gpsMessage[gpsMsgIdx-2]) && (CK_B == gpsMessage[gpsMsgIdx-1]))
								ubxSumError++;

							if (ubxCount > 0)
							{
								xprintf("UBX %d ID %d len %d CH errors %d\n", gpsMessage[2], gpsMessage[3], gpsLen, ubxSumError);
								ubxCount--;
							}

							gpsMsgIdx = 0;
							msgType = 0;
						}
					}

					break;
				default:
					msgType = 0;
					gpsMsgIdx = 0;
					break;
			} // switch
		} // while chars in buffer
	}
}

