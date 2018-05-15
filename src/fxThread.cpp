#include <asf.h>

#include "ch.h"

#include <pio/samd21j18a.h>
#include <port.h>

#include "user_board.h"

#include "xprintf.h"

#include "MadgwickAHRS.h"

extern Madgwick filter;

/*=========================================================================
    I2C ADDRESS/BITS AND SETTINGS
    -----------------------------------------------------------------------*/
    /** 7-bit I2C address for this sensor */
    #define FXOS8700_ADDRESS           (0x1F)     // 0011111
    /** Device ID for this sensor (used as sanity check during init) */
    #define FXOS8700_ID                (0xC7)     // 1100 0111

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    /*!
        Raw register addresses used to communicate with the sensor.
    */
    typedef enum
    {
      FXOS8700_REGISTER_STATUS          = 0x00, /**< 0x00 */
      FXOS8700_REGISTER_OUT_X_MSB       = 0x01, /**< 0x01 */
      FXOS8700_REGISTER_OUT_X_LSB       = 0x02, /**< 0x02 */
      FXOS8700_REGISTER_OUT_Y_MSB       = 0x03, /**< 0x03 */
      FXOS8700_REGISTER_OUT_Y_LSB       = 0x04, /**< 0x04 */
      FXOS8700_REGISTER_OUT_Z_MSB       = 0x05, /**< 0x05 */
      FXOS8700_REGISTER_OUT_Z_LSB       = 0x06, /**< 0x06 */
      FXOS8700_REGISTER_WHO_AM_I        = 0x0D, /**< 0x0D (default value = 0b11000111, read only) */
      FXOS8700_REGISTER_XYZ_DATA_CFG    = 0x0E, /**< 0x0E */
      FXOS8700_REGISTER_INT_SOURCE      = 0x0C, /**< 0x0C */
      FXOS8700_REGISTER_CTRL_REG1       = 0x2A, /**< 0x2A (default value = 0b00000000, read/write) */
      FXOS8700_REGISTER_CTRL_REG2       = 0x2B, /**< 0x2B (default value = 0b00000000, read/write) */
      FXOS8700_REGISTER_CTRL_REG3       = 0x2C, /**< 0x2C (default value = 0b00000000, read/write) */
      FXOS8700_REGISTER_CTRL_REG4       = 0x2D, /**< 0x2D (default value = 0b00000000, read/write) */
      FXOS8700_REGISTER_CTRL_REG5       = 0x2E, /**< 0x2E (default value = 0b00000000, read/write) */
      FXOS8700_REGISTER_MSTATUS         = 0x32, /**< 0x32 */
      FXOS8700_REGISTER_MOUT_X_MSB      = 0x33, /**< 0x33 */
      FXOS8700_REGISTER_MOUT_X_LSB      = 0x34, /**< 0x34 */
      FXOS8700_REGISTER_MOUT_Y_MSB      = 0x35, /**< 0x35 */
      FXOS8700_REGISTER_MOUT_Y_LSB      = 0x36, /**< 0x36 */
      FXOS8700_REGISTER_MOUT_Z_MSB      = 0x37, /**< 0x37 */
      FXOS8700_REGISTER_MOUT_Z_LSB      = 0x38, /**< 0x38 */
      FXOS8700_REGISTER_MCTRL_REG1      = 0x5B, /**< 0x5B (default value = 0b00000000, read/write) */
      FXOS8700_REGISTER_MCTRL_REG2      = 0x5C, /**< 0x5C (default value = 0b00000000, read/write) */
      FXOS8700_REGISTER_MCTRL_REG3      = 0x5D, /**< 0x5D (default value = 0b00000000, read/write) */
      FXOS8700_REGISTER_MINT_SRC        = 0x5E, /**< 0x5E (default value = 0b00000000, read/write) */
    } fxos8700Registers_t;
/*=========================================================================*/

/*=========================================================================
    OPTIONAL SPEED SETTINGS
    -----------------------------------------------------------------------*/
    /*!
        Range settings for the accelerometer sensor.
    */
    typedef enum
    {
      ACCEL_RANGE_2G                    = 0x00, /**< +/- 2g range */
      ACCEL_RANGE_4G                    = 0x01, /**< +/- 4g range */
      ACCEL_RANGE_8G                    = 0x02  /**< +/- 8g range */
    } fxos8700AccelRange_t;
/*=========================================================================*/

    /*=========================================================================
        I2C ADDRESS/BITS AND SETTINGS
        -----------------------------------------------------------------------*/
        /** 7-bit address for this sensor */
        #define FXAS21002C_ADDRESS       (0x21)       // 0100001
        /** Device ID for this sensor (used as a sanity check during init) */
        #define FXAS21002C_ID            (0xD7)       // 1101 0111
        /** Gyroscope sensitivity at 250dps */
        #define GYRO_SENSITIVITY_250DPS  (0.0078125F) // Table 35 of datasheet
        /** Gyroscope sensitivity at 500dps */
        #define GYRO_SENSITIVITY_500DPS  (0.015625F)
        /** Gyroscope sensitivity at 1000dps */
        #define GYRO_SENSITIVITY_1000DPS (0.03125F)
        /** Gyroscope sensitivity at 2000dps */
        #define GYRO_SENSITIVITY_2000DPS (0.0625F)
    /*=========================================================================*/

    /*=========================================================================
        REGISTERS
        -----------------------------------------------------------------------*/
        /*!
            Raw register addresses used to communicate with the sensor.
        */
        typedef enum
        {
          GYRO_REGISTER_STATUS              = 0x00, /**< 0x00 */
          GYRO_REGISTER_OUT_X_MSB           = 0x01, /**< 0x01 */
          GYRO_REGISTER_OUT_X_LSB           = 0x02, /**< 0x02 */
          GYRO_REGISTER_OUT_Y_MSB           = 0x03, /**< 0x03 */
          GYRO_REGISTER_OUT_Y_LSB           = 0x04, /**< 0x04 */
          GYRO_REGISTER_OUT_Z_MSB           = 0x05, /**< 0x05 */
          GYRO_REGISTER_OUT_Z_LSB           = 0x06, /**< 0x06 */
          GYRO_REGISTER_INT_SRC_FLAG        = 0x0B, /**< 0x0B */
          GYRO_REGISTER_WHO_AM_I            = 0x0C, /**< 0x0C (default value = 0b11010111, read only) */
          GYRO_REGISTER_CTRL_REG0           = 0x0D, /**< 0x0D (default value = 0b00000000, read/write) */
          GYRO_REGISTER_CTRL_REG1           = 0x13, /**< 0x13 (default value = 0b00000000, read/write) */
          GYRO_REGISTER_CTRL_REG2           = 0x14, /**< 0x14 (default value = 0b00000000, read/write) */
        } gyroRegisters_t;
    /*=========================================================================*/

    /*=========================================================================
        OPTIONAL SPEED SETTINGS
        -----------------------------------------------------------------------*/
        /*!
            Enum to define valid gyroscope range values
        */
        typedef enum
        {
          GYRO_RANGE_250DPS  = 250,     /**< 250dps */
          GYRO_RANGE_500DPS  = 500,     /**< 500dps */
          GYRO_RANGE_1000DPS = 1000,    /**< 1000dps */
          GYRO_RANGE_2000DPS = 2000     /**< 2000dps */
        } gyroRange_t;
    /*=========================================================================*/


#define TASK_STACK 1024

extern int16_t imuStatus;
volatile uint16_t extCount;

struct
{
	short compass[3];
	short gyro[3];
	short accel[3];
	long quat[4];
	long temp;
} imuData;

extern "C" void writeFile(char , int , const void *);

/*
 * IMU thread.
 */

//struct
//{
//	short compass[3];
//	short gyro[3];
//	short accel[3];
//	long quat[4];
//	long temp;
//} imuData;

static struct i2c_master_packet packet = { .address = 0, .data_length = 0, .data = NULL, .ten_bit_address = false, .high_speed = false, .hs_master_code = false };

static struct i2c_master_module i2c_master_instance;

extern "C"
{
static int i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data);
static int i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data);

static int i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data)
{
	int status;

	packet.address = slave_addr;
	packet.data_length = 1;
	packet.data = &reg_addr;

	//system_interrupt_enter_critical_section();
	//chSysLock();
	//xprintf("i2cWrite: addr 0x%02x, reg 0x%02x, len %d, data 0x%02x\n", slave_addr, reg_addr, length, *data);

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

static int i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data)
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

static void configure_i2c_master(void);

static void configure_i2c_master(void)
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

void extint_fx_detection_callback(void)
{
	CH_IRQ_PROLOGUE();

	uint8_t d;

	port_pin_set_output_level(LED_0_PIN, 1);

	chSysLockFromISR();
	chEvtSignalI(&nil.threads[0], 1);
	chSysUnlockFromISR();

	CH_IRQ_EPILOGUE();
}

extern unsigned short _aSense;
extern float _gSense, _mSense;

static int8_t configure_imu(void)
{
	uint8_t ida, idg;
	uint8_t data;

	/* Make sure we have the correct chip ID since this checks for correct address and that the IC is properly connected */
	i2c_read(FXOS8700_ADDRESS, FXOS8700_REGISTER_WHO_AM_I, 1, &ida);

	xprintf("FXOS id 0x%02x\n", ida);

	/* Make sure we have the correct chip ID since this checks for correct address and that the IC is properly connected */
	i2c_read(FXAS21002C_ADDRESS, GYRO_REGISTER_WHO_AM_I, 1, &idg);

	xprintf("FXAS id 0x%02x\n", idg);

	if (ida != FXOS8700_ID)
	{
		xprintf("FXOS ID bad\n");
		return -1;
	}
	if (idg != FXAS21002C_ID)
	{
		xprintf("FXAS ID bad\n");
		return -1;
	}

	/* Set to standby mode (required to make changes to this register) */

	// write 0000 0000 = 0x00 to accelerometer control register 1 to place FXOS8700CQ into
	// standby
	// [7-1] = 0000 000
	// [0]: active=0

	data = 0;
	i2c_write(FXOS8700_ADDRESS, FXOS8700_REGISTER_CTRL_REG1, 1, &data);

	/* Configure the accelerometer */
	data = 0x01; // +/- 4g
	i2c_write(FXOS8700_ADDRESS, FXOS8700_REGISTER_XYZ_DATA_CFG, 1, &data);

	/* High resolution */
	data = 0x02;
	i2c_write(FXOS8700_ADDRESS, FXOS8700_REGISTER_CTRL_REG2, 1, &data);

	//	data = 0x00;
	//	i2c_write(FXOS8700_ADDRESS, FXOS8700_REGISTER_CTRL_REG3, 1, &data); // Active Low, push-pull
	data = 0x01;
	i2c_write(FXOS8700_ADDRESS, FXOS8700_REGISTER_CTRL_REG5, 1, &data); // data rdy interrupt output = INT1
	data = 0x01;
	i2c_write(FXOS8700_ADDRESS, FXOS8700_REGISTER_CTRL_REG4, 1, &data); // enable data rdy interrupt output

	/* Active, Normal Mode, Low Noise, 50Hz in Hybrid Mode */
	// write 0000 1101 = 0x0D to accelerometer control register 1
	// [7-6]: aslp_rate=00
	// [5-3]: dr=001 for 50Hz data rate (when in hybrid mode)
	// [2]: lnoise=1 for low noise mode
	// [1]: f_read=0 for normal 16 bit reads
	// [0]: active=1 to take the part out of standby and enable sampling
	data = 0x1D;
	i2c_write(FXOS8700_ADDRESS, FXOS8700_REGISTER_CTRL_REG1, 1, &data);

	/* Configure the magnetometer */
	/* Hybrid Mode, Over Sampling Rate = 16 */
	// write 0001 1111 = 0x1F to magnetometer control register 1 // [7]: m_acal=0: auto calibration disabled
	// [6]: m_rst=0: no one-shot magnetic reset
	// [5]: m_ost=0: no one-shot magnetic measurement
	// [4-2]: m_os=101=5: 8x oversampling (for 100Hz) to reduce magnetometer noise
	// [1-0]: m_hms=11=3: select hybrid mode with accel and 	magnetometer active
	data = 0x17;
	i2c_write(FXOS8700_ADDRESS, FXOS8700_REGISTER_MCTRL_REG1, 1, &data);

	data = 0x01; // data rdy
	i2c_write(FXOS8700_ADDRESS, FXOS8700_REGISTER_MINT_SRC, 1, &data);

	/* Jump to reg 0x33 after reading 0x06 */
	// write 0010 0000 = 0x20 to magnetometer control register 2 // [7]: reserved
	// [6]: reserved
	// [5]: hyb_autoinc_mode=1 to map the magnetometer registers to follow the accelerometer registers
	// [4]: m_maxmin_dis=0 to retain default min/max latching even though not used
	// [3]: m_maxmin_dis_ths=0
	// [2]: m_maxmin_rst=0
	// [1-0]: m_rst_cnt=00 to enable magnetic reset each cycle
	data = 0x20;
	i2c_write(FXOS8700_ADDRESS, FXOS8700_REGISTER_MCTRL_REG2, 1, &data);

	  /* Set CTRL_REG1 (0x13)
	   ====================================================================
	   BIT  Symbol    Description                                   Default
	   ---  ------    --------------------------------------------- -------
	     6  RESET     Reset device on 1                                   0
	     5  ST        Self test enabled on 1                              0
	   4:2  DR        Output data rate                                  000
	                  000 = 800 Hz
	                  001 = 400 Hz
	                  010 = 200 Hz
	                  011 = 100 Hz
	                  100 = 50 Hz
	                  101 = 25 Hz
	                  110 = 12.5 Hz
	                  111 = 12.5 Hz
	     1  ACTIVE    Standby(0)/Active(1)                                0
	     0  READY     Standby(0)/Ready(1)                                 0
	  */

	  /* Set CTRL_REG0 (0x0D)  Default value 0x00
	  =====================================================================
	  BIT  Symbol     Description                                   Default
	  7:6  BW         cut-off frequency of low-pass filter               00
	    5  SPIW       SPI interface mode selection                        0
	  4:3  SEL        High-pass filter cutoff frequency selection        00
	    2  HPF_EN     High-pass filter enable                             0
	  1:0  FS         Full-scale range selection
	                  00 = +-2000 dps
	                  01 = +-1000 dps
	                  10 = +-500 dps
	                  11 = +-250 dps
	  The bit fields in CTRL_REG0 should be changed only in Standby or Ready modes.
	  */

	/* Reset then switch to active mode with 50Hz output */
	data = 0;
	i2c_write(FXAS21002C_ADDRESS, GYRO_REGISTER_CTRL_REG1, 1, &data);     // Standby
	data = (1<<6);
	i2c_write(FXAS21002C_ADDRESS, GYRO_REGISTER_CTRL_REG1, 1, &data);   // Reset
	data = 0x02; // 500 dps, 16 Hz low pass cut off
	i2c_write(FXAS21002C_ADDRESS, GYRO_REGISTER_CTRL_REG0, 1, &data); // Set sensitivity
	data = 0x0c; // INT_CFG_DRDY, INT_EN_DRDY, PP, Active Low
	i2c_write(FXAS21002C_ADDRESS, GYRO_REGISTER_CTRL_REG2, 1, &data);
	data = 0x12; // 50 Hz, active
	i2c_write(FXAS21002C_ADDRESS, GYRO_REGISTER_CTRL_REG1, 1, &data);     // Active

	_gSense = GYRO_SENSITIVITY_500DPS;
	_mSense = 0.1;
	_aSense = 2048; // 14 bits +/- 5g -> 16384/4 = 2048

	struct extint_chan_conf config_extint_chan;
	extint_chan_get_config_defaults(&config_extint_chan);

	config_extint_chan.gpio_pin           = PIN_PB08;
	config_extint_chan.gpio_pin_mux       = MUX_PB08A_EIC_EXTINT8;
	config_extint_chan.gpio_pin_pull      = EXTINT_PULL_UP;
	config_extint_chan.detection_criteria = EXTINT_DETECT_FALLING;

	extint_chan_set_config(8, &config_extint_chan);

	extint_register_callback(extint_fx_detection_callback, 8, EXTINT_CALLBACK_TYPE_DETECT);
	extint_chan_enable_callback(8, EXTINT_CALLBACK_TYPE_DETECT);

	config_extint_chan.gpio_pin           = PIN_PB09;
	config_extint_chan.gpio_pin_mux       = MUX_PB09A_EIC_EXTINT9;
	config_extint_chan.gpio_pin_pull      = EXTINT_PULL_UP;
	config_extint_chan.detection_criteria = EXTINT_DETECT_FALLING;

	extint_chan_set_config(9, &config_extint_chan);

	extint_register_callback(extint_fx_detection_callback, 9, EXTINT_CALLBACK_TYPE_DETECT);
	extint_chan_enable_callback(9, EXTINT_CALLBACK_TYPE_DETECT);

	return 0;
}

extern unsigned int imuCount ;

extern float ax, ay, az;
extern float gx, gy, gz;
extern float mx, my, mz;

THD_FUNCTION(ThreadFX, arg)
{
	(void) arg;
	uint8_t data[13];
	uint8_t a_status, m_status;

	configure_i2c_master();

	chThdSleepMilliseconds(450);
	xprintf("fxThread\n");
	chThdSleepMilliseconds(500);

	configure_imu();
	imuCount = 10;

	while (true)
	{
		chEvtWaitAnyTimeout(ALL_EVENTS, TIME_INFINITE);

		port_pin_set_output_level(LED_0_PIN, 0);

		// check for ACCEL/MAG interrupt
		if (port_pin_get_input_level(PIN_PA08) == 0)
		{
			i2c_read(FXOS8700_ADDRESS, FXOS8700_REGISTER_INT_SOURCE, 1, (unsigned char *)&data);
			if (imuCount > 0)
			{
				xprintf("INT_SOURCE 0x%02x\n", data[0]);
			}
			i2c_read(FXOS8700_ADDRESS, FXOS8700_REGISTER_STATUS, 13, (unsigned char *)&data);
			a_status = data[0];

	 	    /* Shift values to create properly formed integers */
			/* Note, accel data is 14-bit and left-aligned, so we shift two bit right */

			imuData.accel[0] = (int16_t)((data[1] << 8) | data[2]) >> 2;
			imuData.accel[1] = (int16_t)((data[3] << 8) | data[4]) >> 2;
			imuData.accel[2] = (int16_t)((data[5] << 8) | data[6]) >> 2;

			imuData.compass[0] = (int16_t)((data[7] << 8) | data[8]);
			imuData.compass[1] = (int16_t)((data[9] << 8) | data[10]);
			imuData.compass[2] = (int16_t)((data[11] << 8) | data[12]);

			ax = (float)imuData.accel[0] / (float)_aSense;
			ay = (float)imuData.accel[1] / (float)_aSense;
			az = (float)imuData.accel[2] / (float)_aSense;
			mx = (float)imuData.compass[0] * _mSense;
			my = (float)imuData.compass[1] * _mSense;
			mz = (float)imuData.compass[2] * _mSense;

			if (imuCount > 0)
			{
				xprintf("FXOS status 0x%02x\n", a_status);
				xprintf(" compass : %5d %5d %5d", imuData.compass[0], imuData.compass[1], imuData.compass[2]);
				xprintf(" accel : %5d %5d %5d\n", imuData.accel[0], imuData.accel[1], imuData.accel[2]);

				xprintf(" ax ay az %d %d %d\n", (int)(ax * 100), (int)(ay * 100), (int)(az * 100));

				imuCount--;
			}
		}

		// did we get a GYRO interrupt
		if (port_pin_get_input_level(PIN_PA09) == 0)
		{

			i2c_read(FXAS21002C_ADDRESS, GYRO_REGISTER_STATUS | 0x80, 7, (unsigned char *)&data);
			m_status = data[0];
			imuData.gyro[0] = (int16_t)((data[1] << 8) | data[2]);
			imuData.gyro[1] = (int16_t)((data[3] << 8) | data[4]);
			imuData.gyro[2] = (int16_t)((data[5] << 8) | data[6]);

			gx = (float)imuData.gyro[0] * _gSense;
			gy = (float)imuData.gyro[1] * _gSense;
			gz = (float)imuData.gyro[2] * _gSense;

			filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);

			float q1, q2, q3, q4;
			filter.getQuat(&q1, &q2, &q3, &q4);
			imuData.quat[0] = q1 * 65536;
			imuData.quat[1] = q2 * 65536;
			imuData.quat[2] = q3 * 65536;
			imuData.quat[3] = q4 * 65536;

			if (imuCount > 0)
			{
				xprintf("FXA status 0x%02x\n", m_status);
				xprintf(" gryo : %5d %5d %5d", imuData.gyro[0], imuData.gyro[1], imuData.gyro[2]);
				xprintf(" quant : %11d %11d %11d %11d\n", imuData.quat[0], imuData.quat[1], imuData.quat[2], imuData.quat[3]);

				float h, p, r;

				h = filter.getYaw() * 100.0;
				p = filter.getPitch() * 100.0;
				r = filter.getRoll() * 100.0;

				xprintf(" HPR : %5d %5d %5d\n", h, p, r);

				imuCount--;
			}

			// write IMU data to SD card
			writeFile(0x03, sizeof(imuData), &imuData);
		}

		chThdSleepMilliseconds(1);
	}
}

