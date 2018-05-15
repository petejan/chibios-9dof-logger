

#include <asf.h>

#include "ch.h"
#include "xprintf.h"

extern void writeFile(char , int , const void *);

#define TASK_STACK 1024

extern struct port_config pin_conf;

volatile uint8_t eic = 0;
volatile int32_t data;
void extint_detection_callback(void)
{
	CH_IRQ_PROLOGUE();

	uint8_t d;

	port_pin_set_output_level(LED_0_PIN, 1);
	eic = 1;
	struct extint_chan_conf config_extint_chan;
	extint_chan_get_config_defaults(&config_extint_chan);

	config_extint_chan.gpio_pin           = PIN_PA04;
	config_extint_chan.gpio_pin_mux       = MUX_PA04A_EIC_EXTINT4;
	config_extint_chan.gpio_pin_pull      = EXTINT_PULL_UP;
	config_extint_chan.detection_criteria = EXTINT_DETECT_NONE;

	extint_chan_set_config(4, &config_extint_chan);

	extint_chan_disable_callback(4, EXTINT_CALLBACK_TYPE_DETECT);
	data = 0;
	for (int i=0;i<25;i++)
	{
		port_pin_set_output_level(PIN_PA05, 1);
		if (i < 24)
		{
			if (port_pin_get_input_level(PIN_PA04))
			{
				d = 1;
			}
			else
			{
				d = 0;
			}
			data = (data << 1) + d;
		}
		port_pin_set_output_level(PIN_PA05, 0);
	}
	config_extint_chan.gpio_pin           = PIN_PA04;
	config_extint_chan.gpio_pin_mux       = MUX_PA04A_EIC_EXTINT4;
	config_extint_chan.gpio_pin_pull      = EXTINT_PULL_UP;
	config_extint_chan.detection_criteria = EXTINT_DETECT_FALLING;

	extint_chan_set_config(4, &config_extint_chan);

	extint_chan_enable_callback(4, EXTINT_CALLBACK_TYPE_DETECT);

	chSysLockFromISR();
	chEvtSignalI(&nil.threads[2], 1);
	chSysUnlockFromISR();

	CH_IRQ_EPILOGUE();
}

unsigned int adcCount = 0;

THD_FUNCTION(ThreadADC, arg)
{
	(void) arg;

	chThdSleepMilliseconds(400);
	xprintf("ADCThread\n");
	chThdSleepMilliseconds(100);

	pin_conf.direction = PORT_PIN_DIR_INPUT;
	pin_conf.input_pull = PORT_PIN_PULL_UP;
	port_pin_set_config(PIN_PA04, &pin_conf);

	pin_conf.direction = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(PIN_PA05, &pin_conf);
	port_pin_set_output_level(PIN_PA05, 0);

	struct extint_chan_conf config_extint_chan;
	extint_chan_get_config_defaults(&config_extint_chan);

	config_extint_chan.gpio_pin           = PIN_PA04;
	config_extint_chan.gpio_pin_mux       = MUX_PA04A_EIC_EXTINT4;
	config_extint_chan.gpio_pin_pull      = EXTINT_PULL_UP;
	config_extint_chan.detection_criteria = EXTINT_DETECT_FALLING;

	extint_chan_set_config(4, &config_extint_chan);

	extint_register_callback(extint_detection_callback, 4, EXTINT_CALLBACK_TYPE_DETECT);
	extint_chan_enable_callback(4, EXTINT_CALLBACK_TYPE_DETECT);

	int32_t dat;
	while (true)
	{
		chEvtWaitAnyTimeout(ALL_EVENTS, TIME_INFINITE);

		dat = (data & 0x7FFFFFL); // * (1 - ((data & 0x800000L) >> 22));
		if (data & 0x800000L)
		{
			dat = 0xFF800000L | dat;
		}
		writeFile(0x05, 4, &dat);
		if (adcCount > 0)
		{
			xprintf("ADC %ld\n", dat);
			adcCount--;
		}
		port_pin_set_output_level(LED_0_PIN, 0);

	}

}
