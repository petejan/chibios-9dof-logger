#include <asf.h>

#include "ch.h"

#include <pio/samd21j18a.h>
#include <port.h>

#include "user_board.h"

#include "xprintf.h"

#include "util/inv_mpu.h"
#include "util/inv_mpu_dmp_motion_driver.h"

#include "MPU9250_RegisterMap.h"

#define TASK_STACK 1024

int16_t imuStatus;
volatile uint16_t nmiCount;

extern "C" void writeFile(char , int , const void *);

#include "MadgwickAHRS.h"

Madgwick filter;

/*
 * IMU thread.
 */

struct
{
	short compass[3];
	short gyro[3];
	short accel[3];
	long quat[4];
	long temp;
} imuData;

CH_IRQ_HANDLER(NMI_Handler)
{
//	CH_IRQ_PROLOGUE();
//
//	chSysLockFromISR();

	//mpu_get_int_status(&imuStatus);
	extint_nmi_clear_detected(0);
	port_pin_set_output_level(LED_0_PIN, LED_0_ACTIVE);

	nmiCount++;

//	chEvtSignalI(&nil.threads[0], 1);
//
//	chSysUnlockFromISR();
//
//	CH_IRQ_EPILOGUE();
}

struct i2c_master_packet packet = { .address = 0, .data_length = 0, .data = NULL, .ten_bit_address = false, .high_speed = false, .hs_master_code = false };

struct i2c_master_module i2c_master_instance;

extern "C"
{
extern int i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data);
extern int i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data);

int i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data)
{
	int status;

	packet.address = slave_addr;
	packet.data_length = 1;
	packet.data = &reg_addr;

	//system_interrupt_enter_critical_section();
	//chSysLock();

	status = i2c_master_write_packet_wait_no_stop(&i2c_master_instance,	&packet);

	for (int i = 0; i < length; i++)
	{
		status = i2c_master_write_byte(&i2c_master_instance, data[i]);
	}

	i2c_master_send_stop(&i2c_master_instance);

	//chSysUnlock()
	//system_interrupt_leave_critical_section();

	return status;
}

int i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data)
{
	int status;

	packet.address = slave_addr;
	packet.data_length = 1;
	packet.data = &reg_addr;

	//system_interrupt_enter_critical_section();
	//chSysLock()

	status = i2c_master_write_packet_wait_no_stop(&i2c_master_instance,	&packet);

	if (status == STATUS_OK)
	{
		packet.address = slave_addr;
		packet.data_length = length;
		packet.data = data;

		status = i2c_master_read_packet_wait(&i2c_master_instance, &packet);
	}
	else
	{
		i2c_master_send_stop(&i2c_master_instance);
	}
	//chSysUnlock();
	//system_interrupt_leave_critical_section();

	return status;
}
}

void configure_i2c_master(void);

void configure_i2c_master(void)
{
	/* Initialize config structure and software module. */
	struct i2c_master_config config_i2c_master;
	i2c_master_get_config_defaults(&config_i2c_master);
	config_i2c_master.pinmux_pad0 = PINMUX_PA22C_SERCOM3_PAD0;
	config_i2c_master.pinmux_pad1 = PINMUX_PA23C_SERCOM3_PAD1;

	/* Initialize and enable device with config, and enable i2c. */
	i2c_master_init(&i2c_master_instance, SERCOM3, &config_i2c_master);

	//	struct system_pinmux_config config_pinmux;
	//	system_pinmux_get_config_defaults(&config_pinmux);
	//
	//	config_pinmux.mux_position = MUX_PA22C_SERCOM3_PAD0;
	//	config_pinmux.direction    = SYSTEM_PINMUX_PIN_DIR_INPUT;
	//	config_pinmux.input_pull   = SYSTEM_PINMUX_PIN_PULL_UP;
	//
	//	system_pinmux_pin_set_config(PIN_PA22, &config_pinmux);
	//
	//	config_pinmux.mux_position = MUX_PA23C_SERCOM3_PAD1;
	//	config_pinmux.direction    = SYSTEM_PINMUX_PIN_DIR_INPUT;
	//	config_pinmux.input_pull   = SYSTEM_PINMUX_PIN_PULL_UP;
	//
	//	system_pinmux_pin_set_config(PIN_PA23, &config_pinmux);

	i2c_master_enable(&i2c_master_instance);
}

typedef int inv_error_t;

void tap(unsigned char dir, unsigned char count)
{
	unsigned char data[2];
	data[0] = dir;
	data[1] = count;

	writeFile(0x02, 2, data);

	xprintf("tap count %d dir %d\n", count, dir);
}

int8_t configure_imu(void)
{
	inv_error_t result;
	struct int_param_s int_param;
	struct port_config pin_conf;

	//	config_pinmux.mux_position = SYSTEM_PINMUX_GPIO;
	//	config_pinmux.direction    = SYSTEM_PINMUX_PIN_DIR_INPUT;
	//	config_pinmux.input_pull   = SYSTEM_PINMUX_PIN_PULL_UP;
	//
	//	system_pinmux_pin_set_config(PIN_PA08, &config_pinmux);

	// INT pin
	pin_conf.direction = PORT_PIN_DIR_INPUT;
	port_pin_set_config(PIN_PA08, &pin_conf);

	// FSYNC pin
	pin_conf.direction = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(PIN_PA09, &pin_conf);
	port_pin_set_output_level(PIN_PA09, 0);

	result = mpu_init(&int_param);
	if (result != 0)
	{
		xprintf("mpu init failed\n");

		return -1;
	}

	mpu_set_bypass(1); // Place all slaves (including compass) on primary bus

	mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

	// Set up MPU-9250 interrupt:
	//set_int_enable(1); // Enable interrupt output
	set_int_enable(0x02); // Data READY (0x01) and DMP (0x02)

	mpu_set_int_level(1);    // Set interrupt to active-low
	mpu_set_int_latched(0);  // Latch interrupt output

	// Configure sensors:
	// Set gyro full-scale range: options are 250, 500, 1000, or 2000:
	mpu_set_gyro_fsr(2000);
	// Set accel full-scale range: options are 2, 4, 8, or 16 g
	mpu_set_accel_fsr(4);
	// Set gyro/accel LPF: options are 5, 10, 20, 42, 98, 188 Hz
	mpu_set_lpf(20);
	// Set gyro/accel sample rate: must be between 4-1000Hz
	// (note: this value will be overridden by the DMP sample rate)
	mpu_set_sample_rate(120);
	// Set compass sample rate: between 4-100Hz
	mpu_set_compass_sample_rate(10);

	uint16_t rate;
	int res;
	res = mpu_get_compass_sample_rate(&rate);
	xprintf("compass %d rate %d\n", res, rate);
	mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

	// Configure digital motion processor. Use the FIFO to get
	// data from the DMP.
	unsigned short dmpFeatureMask = 0;
	if (true)
	{
		// Gyro calibration re-calibrates the gyro after a set amount
		// of no motion detected
		dmpFeatureMask |= DMP_FEATURE_SEND_CAL_GYRO;
	}
	else
	{
		// Otherwise add raw gyro readings to the DMP
		dmpFeatureMask |= DMP_FEATURE_SEND_RAW_GYRO;
	}
	// Add accel and quaternion's to the DMP
	dmpFeatureMask |= DMP_FEATURE_SEND_RAW_ACCEL;
	dmpFeatureMask |= DMP_FEATURE_6X_LP_QUAT;

	// Initialize the DMP, and set the FIFO's update rate:
	dmp_load_motion_driver_firmware();
	dmp_enable_6x_lp_quat(1);
	dmp_set_fifo_rate(40);
	dmp_get_fifo_rate(&rate);
	xprintf("FiFo Rate %d\n", rate);

	unsigned short xThresh = 0;   // Disable x-axis tap
	unsigned short yThresh = 0;   // Disable y-axis tap
	unsigned short zThresh = 100; // Set z-axis tap thresh to 100 mg/ms
	unsigned char taps = 1;       // Set minimum taps to 1
	unsigned short tapTime = 100; // Set tap time to 100ms
	unsigned short tapMulti = 1000;// Set multi-tap time to 1s

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

	unsigned char axes = 0;
	if (xThresh > 0)
	{
		axes |= TAP_X;
		dmp_set_tap_thresh(1<<X_AXIS, xThresh);
	}
	if (yThresh > 0)
	{
		axes |= TAP_Y;
		dmp_set_tap_thresh(1<<Y_AXIS, yThresh);
	}
	if (zThresh > 0)
	{
		axes |= TAP_Z;
		dmp_set_tap_thresh(1<<Z_AXIS, zThresh);
	}
	dmp_set_tap_axes(axes);
	dmp_set_tap_count(taps);
	dmp_set_tap_time(tapTime);
	dmp_set_tap_time_multi(tapMulti);

	dmp_register_tap_cb(tap);

	dmp_enable_feature(dmpFeatureMask |= DMP_FEATURE_TAP);
	mpu_set_dmp_state(1);

	// Enable NMI interrupt
	/* Initialize EIC for NMI */
	struct extint_nmi_conf eint_nmi_conf;

	eint_nmi_conf.gpio_pin            = PIN_PA08A_EIC_NMI;
	eint_nmi_conf.gpio_pin_mux        = PINMUX_PA08A_EIC_NMI;
	eint_nmi_conf.detection_criteria  = EXTINT_DETECT_FALLING;
	eint_nmi_conf.filter_input_signal = false;

	extint_nmi_set_config(0, &eint_nmi_conf);

	pin_conf.direction = PORT_PIN_DIR_INPUT;
	pin_conf.input_pull = PORT_PIN_PULL_UP;
	port_pin_set_config(PIN_PA08, &pin_conf);

	return 0;
}

unsigned int imuCount = 0;
unsigned short _aSense;
float _gSense, _mSense;

float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;

THD_FUNCTION(ThreadMPU, arg)
{
	(void) arg;
	int result;

	configure_i2c_master();

	chThdSleepMilliseconds(400);
	xprintf("mpuThread\n");
	chThdSleepMilliseconds(500);

	if (configure_imu() < 0)
	{
		while(1)
		{
			chThdSleepMilliseconds(1000);
		}
	}

	mpu_get_accel_sens(&_aSense);
	mpu_get_gyro_sens(&_gSense);
	_mSense = 6.665f; // Constant - 4915 / 32760

	while (true)
	{
		//port_pin_set_output_level(LED_0_PIN, LED_0_INACTIVE);

		chEvtWaitAnyTimeout(ALL_EVENTS, TIME_INFINITE);

//		while (nmiCount == 0)
//		{
//			chThdSleepMilliseconds(90);
//		}
//		nmiCount = 0;

		//while (port_pin_get_input_level(PIN_PA08) != 0)
//		{
//			chThdSleepMilliseconds(1000);
//		}

		port_pin_set_output_level(LED_0_PIN, LED_0_INACTIVE);

		//xprintf("IMU event %d\n", evt);

		mpu_get_int_status(&imuStatus);

		if ((imuStatus & 0x02) != 0)
		{
			//xprintf("IMU status 0x%02x\n", imuStatus);

			result = mpu_get_temperature(&imuData.temp, NULL);

			result = mpu_get_compass_reg(imuData.compass, NULL);
		}

		if ((imuStatus & 0x100) != 0)
		{
			unsigned char fifoH, fifoL;

			mpu_read_reg(MPU9250_FIFO_COUNTH, &fifoH);
			mpu_read_reg(MPU9250_FIFO_COUNTL, &fifoL);

			int fifo = (fifoH << 8) | fifoL;

			//xprintf("IMU Fifo %d\n", fifo);

			short sensors;
			unsigned char more;

			result = 0;
			more = 1;
			for (int i = 0; ((i < fifo) & (result == 0) & (more > 0)); i++)
			{
				result = dmp_read_fifo(imuData.gyro, imuData.accel, imuData.quat, NULL, &sensors, &more);

				ax = (float)imuData.accel[0] / (float)_aSense;
				ay = (float)imuData.accel[1] / (float)_aSense;
				az = (float)imuData.accel[2] / (float)_aSense;
				gx = (float)imuData.gyro[0] / _gSense;
				gy = (float)imuData.gyro[1] / _gSense;
				gz = (float)imuData.gyro[2] / _gSense;
				mx = (float)imuData.compass[0] / _mSense;
				my = (float)imuData.compass[1] / _mSense;
				mz = (float)imuData.compass[2] / _mSense;

				filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);

				if (imuCount > 0)
				{
					xprintf("fifo %d more %2d", result, more);
					xprintf(" compass : %5d %5d %5d", imuData.compass[0], imuData.compass[1], imuData.compass[2]);
					xprintf(" sensors 0x%02x", sensors);
					xprintf(" accel : %5d %5d %5d", imuData.accel[0], imuData.accel[1], imuData.accel[2]);
					xprintf(" quant : %11d %11d %11d %11d", imuData.quat[0], imuData.quat[1], imuData.quat[2], imuData.quat[3]);
					xprintf(" gryo : %5d %5d %5d\n", imuData.gyro[0], imuData.gyro[1], imuData.gyro[2]);

					int h, p, r;

					h = filter.getYaw() * 100.0f;
					p = filter.getPitch() * 100.0f;
					r = filter.getRoll() * 100.0f;

					xprintf(" HPR : %5d %5d %5d\n", h, p, r);

					imuCount--;
				}

				writeFile(0x03, sizeof(imuData), &imuData);
			}
			chThdSleepMilliseconds(10);
		}
	}
}

