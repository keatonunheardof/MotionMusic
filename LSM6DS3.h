#include <util/twi.h>

//*****************************************************************************
// Function Prototypes 
//*****************************************************************************
void init_LSM6DS3();
void write_LSM6DS3(uint8_t address, uint8_t value);
void read_LSM6DS3(uint8_t address, uint8_t *rd_buf);
void init_accel();
void init_gyro();
uint8_t accel_data_avail();
uint8_t gyro_data_avail();
void set_bits(uint8_t address, uint8_t bits_to_set);
void clear_bits(uint8_t address, uint8_t bits_to_clear);
uint8_t check_bit(uint8_t address, uint8_t bit_to_check);

//*****************************************************************************
// Device Register Map
//*****************************************************************************

#define LSM6DS3_ADDRESS 0xD4                    //LSM6DS3-0, address pin floating
#define LSM6DS3_WRITE (LSM6DS3_ADDRESS | TW_WRITE) //LSB is a zero to write
#define LSM6DS3_READ  (LSM6DS3_ADDRESS | TW_READ)  //LSB is a one to read

#define FUNC_CFG_ACCESS			0x01
#define SENSOR_SYNC_TIME_FRAME	0x04
#define SENSOR_SYNC_EN			0x05
#define FIFO_CTRL1				0x06
#define FIFO_CTRL2				0x07
#define FIFO_CTRL3				0x08
#define FIFO_CTRL4				0x09
#define FIFO_CTRL5				0x0A
#define ORIENT_CFG_G			0x0B
#define REFERENCE_G				0x0C
#define INT1_CTRL				0x0D
#define INT2_CTRL				0x0E
#define WHO_AM_I				0x0F
#define CTRL1_XL				0x10
#define CTRL2_G					0x11
#define CTRL3_C					0x12
#define CTRL4_C					0x13
#define CTRL5_C					0x14
#define CTRL6_C					0x15
#define CTRL7_G					0x16
#define CTRL8_XL				0x17
#define CTRL9_XL				0x18
#define CTRL10_C				0x19
#define MASTER_CONFIG			0x1A
#define WAKE_UP_SRC				0x1B
#define TAP_SRC					0x1C
#define D6D_SRC					0x1D
#define STATUS_REG				0x1E
#define OUT_TEMP_L				0x20
#define OUT_TEMP_H				0x21
#define OUTX_L_G				0x22
#define OUTX_H_G				0x23
#define OUTY_L_G				0x24
#define OUTY_H_G				0x25
#define OUTZ_L_G				0x26
#define OUTZ_H_G				0x27
#define OUTX_L_XL				0x28
#define OUTX_H_XL				0x29
#define OUTY_L_XL				0x2A
#define OUTY_H_XL				0x2B
#define OUTZ_L_XL				0x2C
#define OUTZ_H_XL				0x2D
#define SENSORHUB1_REG			0x2E
#define SENSORHUB2_REG			0x2F
#define SENSORHUB3_REG			0x30
#define SENSORHUB4_REG			0x31
#define SENSORHUB5_REG			0x32
#define SENSORHUB6_REG			0x33
#define SENSORHUB7_REG			0x34
#define SENSORHUB8_REG			0x35
#define SENSORHUB9_REG			0x36
#define SENSORHUB10_REG			0x37
#define SENSORHUB11_REG			0x38
#define SENSORHUB12_REG			0x39
#define FIFO_STATUS1			0x3A
#define FIFO_STATUS2			0x3B
#define FIFO_STATUS3			0x3C
#define FIFO_STATUS4			0x3D
#define FIFO_DATA_OUT_L			0x3E
#define FIFO_DATA_OUT_H			0x3F
#define TIMESTAMP0_REG			0x40
#define TIMESTAMP1_REG			0x41
#define TIMESTAMP2_REG			0x42
#define STEP_TIMESTAMP_L		0x49
#define STEP_TIMESTAMP_H		0x4A
#define STEP_COUNTER_L			0x4B
#define STEP_COUNTER_H			0x4C
#define SENSORHUB13_REG			0x4D
#define SENSORHUB14_REG			0x4E
#define SENSORHUB15_REG			0x4F
#define SENSORHUB16_REG			0x50
#define SENSORHUB17_REG			0x51
#define SENSORHUB18_REG			0x52
#define FUNC_SRC				0x53
#define TAP_CFG					0x58
#define TAP_THS_6D				0x59
#define INT_DUR2				0x5A
#define WAKE_UP_THS				0x5B
#define WAKE_UP_DUR				0x5C
#define FREE_FALL				0x5D
#define MD1_CFG					0x5E
#define MD2_CFG					0x5F
#define OUT_MAG_RAW_X_L			0x66
#define OUT_MAG_RAW_X_H			0x67
#define OUT_MAG_RAW_Y_L			0x68
#define OUT_MAG_RAW_Y_H			0x69
#define OUT_MAG_RAW_Z_L			0x6A
#define OUT_MAG_RAW_Z_H			0x6B


//*****************************************************************************
// Embedded Function Register Map
//*****************************************************************************

#define SLV0_ADD						0x02
#define SLV0_SUBADD						0x03
#define SLAVE0_CONFIG					0x04
#define SLV1_ADD						0x05
#define SLV1_SUBADD						0x06
#define SLAVE1_CONFIG					0x07
#define SLV2_ADD						0x08
#define SLV2_SUBADD						0x09
#define SLAVE2_CONFIG					0x0A
#define SLV3_ADD						0x0B
#define SLV3_SUBADD						0x0C
#define SLAVE3_CONFIG					0x0D
#define DATAWRITE_SRC_MODE_SUB_SLV0		0x0E
#define CONFIG_PEDO_THS_MIN				0x0F
#define CONFIG_TILT_IIR					0x10
#define CONFIG_TILT_ACOS				0x11
#define CONFIG_TILT_WTIME				0x12
#define SM_STEP_THS						0x13
#define MAG_SI_XX						0x24
#define MAG_SI_XY						0x25
#define MAG_SI_XZ						0x26
#define MAG_SI_YX						0x27
#define MAG_SI_YY						0x28
#define MAG_SI_YZ						0x29
#define MAG_SI_ZX						0x2A
#define MAG_SI_ZY						0x2B
#define MAG_SI_ZZ						0x2C
#define MAG_OFFX_L						0x2D
#define MAG_OFFX_H						0x2E
#define MAG_OFFY_L						0x2F
#define MAG_OFFY_H						0x30
#define MAG_OFFZ_L						0x31
#define MAG_OFFZ_H						0x32


//*****************************************************************************
// Device Register Bit Locations
//*****************************************************************************

#define FUNC_CFG_EN	(1<<7)

#define TPH_7	(1<<7)
#define TPH_6	(1<<6)
#define TPH_5	(1<<5)
#define TPH_4	(1<<4)
#define TPH_3	(1<<3)
#define TPH_2	(1<<2)
#define TPH_1	(1<<1)
#define TPH_0	(1<<0)

#define FTH_7	(1<<7)
#define FTH_6	(1<<6)
#define FTH_5	(1<<5)	
#define FTH_4	(1<<4)
#define FTH_3	(1<<3)
#define FTH_2	(1<<2)
#define FTH_1	(1<<1)
#define FTH_0	(1<<0)

#define TIMER_PEDO_FIFO_EN		(1<<7)
#define TIMER_PEDO_FIFO_DRDY	(1<<6)
#define FTH_11			(1<<3)
#define FTH_10			(1<<2)
#define FTH_9			(1<<1)
#define FTH_8			(1<<0)

#define DEC_FIFO_GYRO2	(1<<5)
#define DEC_FIFO_GYRO1	(1<<4)
#define DEC_FIFO_GYRO0	(1<<3)
#define DEC_FIFO_XL2	(1<<2)
#define DEC_FIFO_XL1	(1<<1)
#define DEC_FIFO_XL0	(1<<0)

#define ONLY_HIGH_DATA	(1<<6)
#define DEC_DS4_FIFO2	(1<<5)
#define DEC_DS4_FIFO1	(1<<4)
#define DEC_DS4_FIFO0	(1<<3)
#define DEC_DS3_FIFO2	(1<<2)
#define DEC_DS3_FIFO1	(1<<1)
#define DEC_DS3_FIFO0	(1<<0)

#define ODR_FIFO_3		(1<<6)
#define ODR_FIFO_2		(1<<5)
#define ODR_FIFO_1		(1<<4)
#define ODR_FIFO_0		(1<<3)
#define FIFO_MODE_2		(1<<2)
#define FIFO_MODE_1		(1<<1)
#define FIFO_MODE_0		(1<<0)

#define SignX_G		(1<<5)
#define SignY_G		(1<<4)
#define SignZ_G		(1<<3)
#define Orient_2	(1<<2)
#define Orient_1	(1<<1)	
#define Orient_0	(1<<0)

#define INT1_STEP_DETECTOR	(1<<7)	
#define INT1_SIG_MOT		(1<<6)
#define INT1_FULL_FLAG		(1<<5)
#define INT1_FIFO_OVR		(1<<4)
#define INT1_FTH			(1<<3)
#define INT1_BOOT			(1<<2)
#define INT1_DRDY_G			(1<<1)
#define INT1_DRDY_XL		(1<<0)

#define INT2_STEP_DELTA		(1<<7)
#define INT2_STEP_COUNT_OV	(1<<6)
#define INT2_FULL_FLAG		(1<<5)
#define INT2_FIFO_OVR		(1<<4)
#define INT2_FTH			(1<<3)
#define INT2_DRDY_TEMP		(1<<2)
#define INT2_DRDY_G			(1<<1)
#define INT2_DRDY_XL		(1<<0)

#define ODR_XL3		(1<<7)
#define ODR_XL2		(1<<6)	
#define ODR_XL1		(1<<5)
#define ODR_XL0		(1<<4)
#define FS_XL1		(1<<3)
#define FS_XL0		(1<<2)
#define BW_XL1		(1<<1)
#define BW_XL0		(1<<0)

#define ODR_G3		(1<<7)
#define ODR_G2		(1<<6)
#define ODR_G1		(1<<5)
#define ODR_G0		(1<<4)
#define FS_G1		(1<<3)
#define FS_G0		(1<<2)
#define FS_125		(1<<1)

#define BOOT		(1<<7)
#define BDU			(1<<6)
#define H_LACTIVE	(1<<5)
#define PP_OD		(1<<4)
#define SIM			(1<<3)
#define IF_INC		(1<<2)
#define BLE			(1<<1)
#define SW_RESET	(1<<0)

#define XL_BW_SCAL_ODR	(1<<7)
#define SLEEP_G			(1<<6)
#define INT2_on_INT1	(1<<5)
#define FIFO_TEMP_EN	(1<<4)
#define DRDY_MASK		(1<<3)
#define I2C_disable		(1<<2)
#define MODE3_EN		(1<<1)
#define STOP_ON_FTH		(1<<0)

#define ROUNDING2	(1<<7)
#define ROUNDING1	(1<<6)
#define ROUNDING0	(1<<5)
#define ST1_G		(1<<3)
#define ST0_G		(1<<2)
#define ST1_XL		(1<<1)
#define ST0_XL		(1<<0)

#define TRIG_EN		(1<<7)
#define LVLen		(1<<6)
#define LVL2_EN		(1<<5)
#define XL_HM_MODE	(1<<4)

#define G_HM_MODE		(1<<7)
#define HP_G_EN			(1<<6)
#define HPCF_G1			(1<<5)
#define HPCF_G0			(1<<4)
#define HP_G_RST		(1<<3)
#define ROUNDING_STATUS	(1<<2)

#define LPF2_XL_EN		(1<<7)
#define HPCF_XL1		(1<<6)
#define HPCF_XL0		(1<<5)
#define HP_SLOPE_XL_EN	(1<<2)
#define LOW_PASS_ON_6D	(1<<0)

#define Zen_XL		(1<<5)	
#define Yen_XL		(1<<4)
#define Xen_XL		(1<<3)
#define SOFT_EN		(1<<2)

#define Zen_G			(1<<5)
#define Yen_G			(1<<4)
#define Xen_G			(1<<3)
#define FUNC_EN			(1<<2)
#define PEDO_RST_STEP	(1<<1)
#define SIGN_MOTION_EN	(1<<0)

#define DRDY_ON_INT1		(1<<7)
#define DATA_VALID_SEL_FIFO	(1<<6)
#define START_CONFIG		(1<<4)
#define PULL_UP_EN			(1<<3)
#define PASS_THROUGH_MODE	(1<<2)
#define IRON_EN				(1<<1)
#define MASTER_ON			(1<<0)

#define FF_IA			(1<<5)
#define SLEEP_STATE_IA	(1<<4)
#define WU_IA			(1<<3)
#define X_WU			(1<<2)
#define Y_WU			(1<<1)
#define Z_WU			(1<<0)

#define TAP_IA		(1<<6)
#define SINGLE_TAP	(1<<5)
#define DOUBLE_TAP	(1<<4)
#define TAP_SIGN	(1<<3)
#define X_TAP		(1<<2)
#define Y_TAP		(1<<1)
#define Z_TAP		(1<<0)
	
#define D6D_IA	    (1<<6)
#define ZH_EVENT	(1<<5)
#define ZL_EVENT    (1<<4)
#define YH_EVENT    (1<<3)
#define YL_EVENT    (1<<2)
#define XH_EVENT    (1<<1)
#define XL_EVENT    (1<<0)

#define EV_BOOT	(1<<3)
#define TDA		(1<<2)
#define GDA		(1<<1)
#define XLDA	(1<<0)

#define Temp15	(1<<7)
#define Temp14	(1<<6)
#define Temp13	(1<<5)
#define Temp12	(1<<4)
#define Temp11	(1<<3)
#define Temp10	(1<<2)
#define Temp9	(1<<1)
#define Temp8	(1<<0)
#define Temp7	(1<<7)
#define Temp6	(1<<6)
#define Temp5	(1<<5)
#define Temp4	(1<<4)
#define Temp3	(1<<3)
#define Temp2	(1<<2)
#define Temp1	(1<<1)
#define Temp0	(1<<0)

#define D15		(1<<7)
#define D14		(1<<6)
#define D13		(1<<5)
#define D12		(1<<4)
#define D11		(1<<3)
#define D10		(1<<2)
#define D9		(1<<1)
#define D8		(1<<0)
#define D7		(1<<7)
#define D6		(1<<6)
#define D5		(1<<5)	
#define D4		(1<<4)
#define D3		(1<<3)
#define D2		(1<<2)
#define D1		(1<<1)
#define D0		(1<<0)

#define SHub1_7		(1<<7)
#define SHub1_6		(1<<6)
#define SHub1_5		(1<<5)
#define SHub1_4		(1<<4)
#define SHub1_3		(1<<3)
#define SHub1_2		(1<<2)
#define SHub1_1		(1<<1)
#define SHub1_0		(1<<0)

#define SHub2_7		(1<<7)
#define SHub2_6		(1<<6)
#define SHub2_5		(1<<5)
#define SHub2_4		(1<<4)
#define SHub2_3		(1<<3)
#define SHub2_2		(1<<2)
#define SHub2_1		(1<<1)
#define SHub2_0		(1<<0)

#define SHub3_7		(1<<7)
#define SHub3_6		(1<<6)
#define SHub3_5		(1<<5)
#define SHub3_4		(1<<4)
#define SHub3_3		(1<<3)
#define SHub3_2		(1<<2)
#define SHub3_1		(1<<1)
#define SHub3_0		(1<<0)

#define SHub4_7		(1<<7)
#define SHub4_6		(1<<6)
#define SHub4_5		(1<<5)
#define SHub4_4		(1<<4)
#define SHub4_3		(1<<3)
#define SHub4_2		(1<<2)
#define SHub4_1		(1<<1)
#define SHub4_0		(1<<0)

#define SHub5_7		(1<<7)
#define SHub5_6		(1<<6)
#define SHub5_5		(1<<5)
#define SHub5_4		(1<<4)
#define SHub5_3		(1<<3)
#define SHub5_2		(1<<2)
#define SHub5_1		(1<<1)
#define SHub5_0		(1<<0)

#define SHub6_7		(1<<7)
#define SHub6_6		(1<<6)
#define SHub6_5		(1<<5)
#define SHub6_4		(1<<4)
#define SHub6_3		(1<<3)
#define SHub6_2		(1<<2)
#define SHub6_1		(1<<1)
#define SHub6_0		(1<<0)

#define SHub7_7		(1<<7)
#define SHub7_6		(1<<6)
#define SHub7_5		(1<<5)
#define SHub7_4		(1<<4)
#define SHub7_3		(1<<3)
#define SHub7_2		(1<<2)
#define SHub7_1		(1<<1)
#define SHub7_0		(1<<0)

#define SHub8_7		(1<<7)
#define SHub8_6		(1<<6)
#define SHub8_5		(1<<5)
#define SHub8_4		(1<<4)
#define SHub8_3		(1<<3)
#define SHub8_2		(1<<2)
#define SHub8_1		(1<<1)
#define SHub8_0		(1<<0)

#define SHub9_7		(1<<7)
#define SHub9_6		(1<<6)
#define SHub9_5		(1<<5)
#define SHub9_4		(1<<4)
#define SHub9_3		(1<<3)
#define SHub9_2		(1<<2)
#define SHub9_1		(1<<1)
#define SHub9_0		(1<<0)

#define SHub10_7	(1<<7)
#define SHub10_6	(1<<6)
#define SHub10_5	(1<<5)
#define SHub10_4	(1<<4)
#define SHub10_3	(1<<3)
#define SHub10_2	(1<<2)
#define SHub10_1	(1<<1)
#define SHub10_0	(1<<0)

#define SHub11_7	(1<<7)
#define SHub11_6	(1<<6)
#define SHub11_5	(1<<5)
#define SHub11_4	(1<<4)
#define SHub11_3	(1<<3)
#define SHub11_2	(1<<2)
#define SHub11_1	(1<<1)
#define SHub11_0	(1<<0)

#define SHub12_7	(1<<7)
#define SHub12_6	(1<<6)
#define SHub12_5	(1<<5)
#define SHub12_4	(1<<4)
#define SHub12_3	(1<<3)
#define SHub12_2	(1<<2)
#define SHub12_1	(1<<1)
#define SHub12_0	(1<<0)

#define DIFF_FIFO_7		(1<<7)
#define DIFF_FIFO_6		(1<<6)
#define DIFF_FIFO_5		(1<<5)
#define DIFF_FIFO_4		(1<<4)
#define DIFF_FIFO_3		(1<<3)
#define DIFF_FIFO_2		(1<<2)
#define DIFF_FIFO_1		(1<<1)
#define DIFF_FIFO_0		(1<<0)

#define FTH				(1<<7)
#define FIFO_OVER_RUN	(1<<6)
#define FIFO_FULL		(1<<5)
#define FIFO_EMPTY		(1<<4)
#define DIFF_FIFO_11	(1<<3)
#define DIFF_FIFO_10	(1<<2)
#define DIFF_FIFO_9		(1<<1)
#define DIFF_FIFO_8		(1<<0)

#define FIFO_PATTERN_7	(1<<7)
#define FIFO_PATTERN_6	(1<<6)
#define FIFO_PATTERN_5	(1<<5)
#define FIFO_PATTERN_4	(1<<4)
#define FIFO_PATTERN_3	(1<<3)
#define FIFO_PATTERN_2	(1<<2)	
#define FIFO_PATTERN_1	(1<<1)
#define FIFO_PATTERN_0	(1<<0)

#define FIFO_PATTERN_9	(1<<1)
#define FIFO_PATTERN_8	(1<<0)

#define DATA_OUT_FIFO_L_7	(1<<7)	
#define DATA_OUT_FIFO_L_6	(1<<6)
#define DATA_OUT_FIFO_L_5	(1<<5)
#define DATA_OUT_FIFO_L_4	(1<<4)
#define DATA_OUT_FIFO_L_3	(1<<3)
#define DATA_OUT_FIFO_L_2	(1<<2)
#define DATA_OUT_FIFO_L_1	(1<<1)
#define DATA_OUT_FIFO_L_0	(1<<0)

#define DATA_OUT_FIFO_H_7	(1<<7)
#define DATA_OUT_FIFO_H_6	(1<<6)
#define DATA_OUT_FIFO_H_5	(1<<5)
#define DATA_OUT_FIFO_H_4	(1<<4)
#define DATA_OUT_FIFO_H_3	(1<<3)	
#define DATA_OUT_FIFO_H_2	(1<<2)
#define DATA_OUT_FIFO_H_1	(1<<1)
#define DATA_OUT_FIFO_H_0	(1<<0)

#define TIMESTAMP0_7	(1<<7)
#define TIMESTAMP0_6	(1<<6)
#define TIMESTAMP0_5	(1<<5)
#define TIMESTAMP0_4	(1<<4)
#define TIMESTAMP0_3	(1<<3)
#define TIMESTAMP0_2	(1<<2)
#define TIMESTAMP0_1	(1<<1)
#define TIMESTAMP0_0	(1<<0)

#define TIMESTAMP1_7	(1<<7)
#define TIMESTAMP1_6	(1<<6)
#define TIMESTAMP1_5	(1<<5)	
#define TIMESTAMP1_4	(1<<4)
#define TIMESTAMP1_3	(1<<3)
#define TIMESTAMP1_2	(1<<2)
#define TIMESTAMP1_1	(1<<1)
#define TIMESTAMP1_0	(1<<0)

#define TIMESTAMP2_7	(1<<7)
#define TIMESTAMP2_6	(1<<6)
#define TIMESTAMP2_5	(1<<5)
#define TIMESTAMP2_4	(1<<4)
#define TIMESTAMP2_3	(1<<3)
#define TIMESTAMP2_2	(1<<2)
#define TIMESTAMP2_1	(1<<1)
#define TIMESTAMP2_0	(1<<0)

#define STEP_TIMESTAMP_L_7	(1<<7)
#define STEP_TIMESTAMP_L_6	(1<<6)
#define STEP_TIMESTAMP_L_5	(1<<5)
#define STEP_TIMESTAMP_L_4	(1<<4)
#define STEP_TIMESTAMP_L_3	(1<<3)
#define STEP_TIMESTAMP_L_2	(1<<2)
#define STEP_TIMESTAMP_L_1	(1<<1)
#define STEP_TIMESTAMP_L_0	(1<<0)

#define STEP_TIMESTAMP_L_7	(1<<7)
#define STEP_TIMESTAMP_L_6	(1<<6)
#define STEP_TIMESTAMP_L_5	(1<<5)
#define STEP_TIMESTAMP_L_4	(1<<4)
#define STEP_TIMESTAMP_L_3	(1<<3)
#define STEP_TIMESTAMP_L_2	(1<<2)
#define STEP_TIMESTAMP_L_1	(1<<1)
#define STEP_TIMESTAMP_L_0	(1<<0)

#define STEP_COUNTER_L_7	(1<<7)
#define STEP_COUNTER_L_6	(1<<6)
#define STEP_COUNTER_L_5	(1<<5)
#define STEP_COUNTER_L_4	(1<<4)
#define STEP_COUNTER_L_3	(1<<3)
#define STEP_COUNTER_L_2	(1<<2)
#define STEP_COUNTER_L_1	(1<<1)
#define STEP_COUNTER_L_0	(1<<0)

#define STEP_COUNTER_H_7	(1<<7)
#define STEP_COUNTER_H_6	(1<<6)
#define STEP_COUNTER_H_5	(1<<5)
#define STEP_COUNTER_H_4	(1<<4)
#define STEP_COUNTER_H_3	(1<<3)
#define STEP_COUNTER_H_2	(1<<2)
#define STEP_COUNTER_H_1	(1<<1)
#define STEP_COUNTER_H_0	(1<<0)

#define SHub13_7	(1<<7)
#define SHub13_6	(1<<6)
#define SHub13_5	(1<<5)
#define SHub13_4	(1<<4)
#define SHub13_3	(1<<3)
#define SHub13_2	(1<<2)
#define SHub13_1	(1<<1)
#define SHub13_0	(1<<0)

#define SHub14_7	(1<<7)
#define SHub14_6	(1<<6)
#define SHub14_5	(1<<5)
#define SHub14_4	(1<<4)
#define SHub14_3	(1<<3)
#define SHub14_2	(1<<2)
#define SHub14_1	(1<<1)
#define SHub14_0	(1<<0)

#define SHub15_7	(1<<7)
#define SHub15_6	(1<<6)
#define SHub15_5	(1<<5)
#define SHub15_4	(1<<4)
#define SHub15_3	(1<<3)
#define SHub15_2	(1<<2)
#define SHub15_1	(1<<1)
#define SHub15_0	(1<<0)

#define SHub16_7	(1<<7)
#define SHub16_6	(1<<6)
#define SHub16_5	(1<<5)
#define SHub16_4	(1<<4)
#define SHub16_3	(1<<3)
#define SHub16_2	(1<<2)
#define SHub16_1	(1<<1)
#define SHub16_0	(1<<0)

#define SHub17_7	(1<<7)
#define SHub17_6	(1<<6)
#define SHub17_5	(1<<5)
#define SHub17_4	(1<<4)
#define SHub17_3	(1<<3)	
#define SHub17_2	(1<<2)
#define SHub17_1	(1<<1)
#define SHub17_0	(1<<0)

#define SHub18_7	(1<<7)
#define SHub18_6	(1<<6)
#define SHub18_5	(1<<5)
#define SHub18_4	(1<<4)
#define SHub18_3	(1<<3)
#define SHub18_2	(1<<2)
#define SHub18_1	(1<<1)
#define SHub18_0	(1<<0)

#define STEP_COUNT_DELTA_IA	(1<<7)
#define SIGN_MOTION_IA		(1<<6)
#define TILT_IA				(1<<5)
#define STEP_DETECTED		(1<<4)
#define STEP_OVERFLOW		(1<<3)
#define SI_END_OP			(1<<1)
#define SENSORHUB_END_OP	(1<<0)

#define TIMER_EN	(1<<7)
#define PEDO_EN		(1<<6)
#define TILT_EN		(1<<5)
#define SLOPE_FDS	(1<<4)
#define TAP_X_EN	(1<<3)
#define TAP_Y_EN	(1<<2)
#define TAP_Z_EN	(1<<1)
#define LIR			(1<<0)

#define D4D_EN		(1<<7)
#define SIXD_THS1	(1<<6)
#define SIXD_THS0	(1<<5)
#define TAP_THS4	(1<<4)
#define TAP_THS3	(1<<3)
#define TAP_THS2	(1<<2)
#define TAP_THS1	(1<<1)
#define TAP_THS0	(1<<0)

#define DUR3		(1<<7)
#define DUR2		(1<<6)
#define DUR1		(1<<5)
#define DUR0		(1<<4)
#define QUIET1		(1<<3)
#define QUIET0		(1<<2)
#define SHOCK1		(1<<1)
#define SHOCK0		(1<<0)

#define SINGLE_DOUBLE_TAP	(1<<7)
#define INACTIVITY	(1<<6)
#define WK_THS5		(1<<5)
#define WK_THS4		(1<<4)
#define WK_THS3		(1<<3)
#define WK_THS2		(1<<2)
#define WK_THS1		(1<<1)
#define WK_THS0		(1<<0)

#define FF_DUR5		(1<<7)
#define WAKE_DUR1	(1<<6)
#define WAKE_DUR0	(1<<5)
#define TIMER_HR	(1<<4)
#define SLEEP_DUR3	(1<<3)
#define SLEEP_DUR2	(1<<2)
#define SLEEP_DUR1	(1<<1)
#define SLEEP_DUR0	(1<<0)

#define FF_DUR4		(1<<7)
#define FF_DUR3		(1<<6)
#define FF_DUR2		(1<<5)
#define FF_DUR1		(1<<4)
#define FF_DUR0		(1<<3)
#define FF_THS2		(1<<2)
#define FF_THS1		(1<<1)
#define FF_THS0		(1<<0)

#define INT1_INACT_STATE	(1<<7)
#define INT1_SINGLE_TAP		(1<<6)
#define INT1_WU				(1<<5)
#define INT1_FF				(1<<4)
#define INT1_DOUBLE_TAP		(1<<3)
#define INT1_6D				(1<<2)
#define INT1_TILT			(1<<1)
#define INT1_TIMER			(1<<0)

#define INT2_INACT_STATE	(1<<7)
#define INT2_SINGLE_TAP		(1<<6)
#define INT2_WU				(1<<5)
#define INT2_FF				(1<<4)
#define INT2_DOUBLE_TAP		(1<<3)
#define INT2_6D				(1<<2)
#define INT2_TILT			(1<<1)
#define INT2_IRON			(1<<0)

//*****************************************************************************
// Embedded Function Register Bit Locations
//*****************************************************************************

#define Slave0_add6		(1<<7)
#define Slave0_add5		(1<<6)
#define Slave0_add4		(1<<5)
#define Slave0_add3		(1<<4)
#define Slave0_add2		(1<<3)
#define Slave0_add1		(1<<2)
#define Slave0_add0		(1<<1)
#define rw_0			(1<<0)

#define Slave0_reg7		(1<<7)
#define Slave0_reg6		(1<<6)
#define Slave0_reg5		(1<<5)
#define Slave0_reg4		(1<<4)
#define Slave0_reg3		(1<<3)
#define Slave0_reg2		(1<<2)
#define Slave0_reg1		(1<<1)
#define Slave0_reg0		(1<<0)

#define Slave0_rate1	(1<<7)
#define Slave0_rate0	(1<<6)
#define Aux_sens_on1	(1<<5)
#define Aux_sens_on0	(1<<4)
#define Src_mode		(1<<3)
#define Slave0_numop2	(1<<2)
#define Slave0_numop1	(1<<1)
#define Slave0_numop0	(1<<0)

#define Slave1_add6		(1<<7)
#define Slave1_add5		(1<<6)
#define Slave1_add4		(1<<5)
#define Slave1_add3		(1<<4)
#define Slave1_add2		(1<<3)
#define Slave1_add1		(1<<2)
#define Slave1_add0		(1<<1)
#define r_1				(1<<0)

#define Slave1_reg7		(1<<7)
#define Slave1_reg6		(1<<6)
#define Slave1_reg5		(1<<5)
#define Slave1_reg4		(1<<4)
#define Slave1_reg3		(1<<3)
#define Slave1_reg2		(1<<2)
#define Slave1_reg1		(1<<1)
#define Slave1_reg0		(1<<0)

#define Slave1_rate1	(1<<7)
#define Slave1_rate0	(1<<6)
#define Slave1_numop2	(1<<2)
#define Slave1_numop1	(1<<1)
#define Slave1_numop0	(1<<0)

#define Slave2_add6		(1<<7)
#define Slave2_add5		(1<<6)
#define Slave2_add4		(1<<5)
#define Slave2_add3		(1<<4)
#define Slave2_add2		(1<<3)
#define Slave2_add1		(1<<2)
#define Slave2_add0		(1<<1)
#define r_2				(1<<0)

#define Slave2_reg7		(1<<7)
#define Slave2_reg6		(1<<6)
#define Slave2_reg5		(1<<5)
#define Slave2_reg4		(1<<4)
#define Slave2_reg3		(1<<3)
#define Slave2_reg2		(1<<2)
#define Slave2_reg1		(1<<1)
#define Slave2_reg0		(1<<0)

#define Slave2_rate1	(1<<7)
#define Slave2_rate0	(1<<6)
#define Slave2_numop2	(1<<2)
#define Slave2_numop1	(1<<1)
#define Slave2_numop0	(1<<0)

#define Slave3_add6		(1<<7)
#define Slave3_add5		(1<<6)
#define Slave3_add4		(1<<5)
#define Slave3_add3		(1<<4)
#define Slave3_add2		(1<<3)
#define Slave3_add1		(1<<2)
#define Slave3_add0		(1<<1)
#define r_3				(1<<0)

#define Slave3_reg7		(1<<7)
#define Slave3_reg6		(1<<6)
#define Slave3_reg5		(1<<5)
#define Slave3_reg4		(1<<4)
#define Slave3_reg3		(1<<3)
#define Slave3_reg2		(1<<2)
#define Slave3_reg1		(1<<1)
#define Slave3_reg0		(1<<0)

#define Slave3_rate1	(1<<7)
#define Slave3_rate0	(1<<6)
#define Slave3_numop2	(1<<2)
#define Slave3_numop1	(1<<1)
#define Slave3_numop0	(1<<0)

#define Slave_dataw_7	(1<<7)
#define Slave_dataw_6	(1<<6)
#define Slave_dataw_5	(1<<5)
#define Slave_dataw_4	(1<<4)
#define Slave_dataw_3	(1<<3)
#define Slave_dataw_2	(1<<2)
#define Slave_dataw_1	(1<<1)
#define Slave_dataw_0	(1<<0)

#define SM_THS_7		(1<<7)
#define SM_THS_6		(1<<6)
#define SM_THS_5		(1<<5)
#define SM_THS_4		(1<<4)
#define SM_THS_3		(1<<3)
#define SM_THS_2		(1<<2)
#define SM_THS_1		(1<<1)
#define SM_THS_0		(1<<0)

#define SC_DELTA_7		(1<<7)
#define SC_DELTA_6		(1<<6)
#define SC_DELTA_5		(1<<5)
#define SC_DELTA_4		(1<<4)
#define SC_DELTA_3		(1<<3)
#define SC_DELTA_2		(1<<2)
#define SC_DELTA_1		(1<<1)
#define SC_DELTA_0		(1<<0)

#define MAG_SI_XX_7		(1<<7)
#define MAG_SI_XX_6		(1<<6)
#define MAG_SI_XX_5		(1<<5)
#define MAG_SI_XX_4		(1<<4)
#define MAG_SI_XX_3		(1<<3)
#define MAG_SI_XX_2		(1<<2)
#define MAG_SI_XX_1		(1<<1)
#define MAG_SI_XX_0		(1<<0)

#define MAG_SI_XY_7		(1<<7)
#define MAG_SI_XY_6		(1<<6)
#define MAG_SI_XY_5		(1<<5)
#define MAG_SI_XY_4		(1<<4)
#define MAG_SI_XY_3		(1<<3)
#define MAG_SI_XY_2		(1<<2)
#define MAG_SI_XY_1		(1<<1)
#define MAG_SI_XY_0		(1<<0)

#define MAG_SI_XZ_7		(1<<7)
#define MAG_SI_XZ_6		(1<<6)
#define MAG_SI_XZ_5		(1<<5)
#define MAG_SI_XZ_4		(1<<4)
#define MAG_SI_XZ_3		(1<<3)
#define MAG_SI_XZ_2		(1<<2)
#define MAG_SI_XZ_1		(1<<1)
#define MAG_SI_XZ_0		(1<<0)

#define MAG_SI_YX_7		(1<<7)
#define MAG_SI_YX_6		(1<<6)
#define MAG_SI_YX_5		(1<<5)
#define MAG_SI_YX_4		(1<<4)
#define MAG_SI_YX_3		(1<<3)
#define MAG_SI_YX_2		(1<<2)
#define MAG_SI_YX_1		(1<<1)
#define MAG_SI_YX_0		(1<<0)

#define MAG_SI_YZ_7		(1<<7)
#define MAG_SI_YZ_6		(1<<6)
#define MAG_SI_YZ_5		(1<<5)
#define MAG_SI_YZ_4		(1<<4)
#define MAG_SI_YZ_3		(1<<3)
#define MAG_SI_YZ_2		(1<<2)
#define MAG_SI_YZ_1		(1<<1)
#define MAG_SI_YZ_0		(1<<0)

#define MAG_SI_YX_7		(1<<7)
#define MAG_SI_YX_6		(1<<6)
#define MAG_SI_YX_5		(1<<5)
#define MAG_SI_YX_4		(1<<4)
#define MAG_SI_YX_3		(1<<3)
#define MAG_SI_YX_2		(1<<2)
#define MAG_SI_YX_1		(1<<1)
#define MAG_SI_YX_0		(1<<0)

#define MAG_SI_ZX_7		(1<<7)
#define MAG_SI_ZX_6		(1<<6)
#define MAG_SI_ZX_5		(1<<5)
#define MAG_SI_ZX_4		(1<<4)
#define MAG_SI_ZX_3		(1<<3)
#define MAG_SI_ZX_2		(1<<2)
#define MAG_SI_ZX_1		(1<<1)
#define MAG_SI_ZX_0		(1<<0)

#define MAG_SI_ZY_7		(1<<7)
#define MAG_SI_ZY_6		(1<<6)
#define MAG_SI_ZY_5		(1<<5)
#define MAG_SI_ZY_4		(1<<4)
#define MAG_SI_ZY_3		(1<<3)
#define MAG_SI_ZY_2		(1<<2)
#define MAG_SI_ZY_1		(1<<1)
#define MAG_SI_ZY_0		(1<<0)

#define MAG_SI_ZZ_7		(1<<7)
#define MAG_SI_ZZ_6		(1<<6)
#define MAG_SI_ZZ_5		(1<<5)
#define MAG_SI_ZZ_4		(1<<4)
#define MAG_SI_ZZ_3		(1<<3)
#define MAG_SI_ZZ_2		(1<<2)
#define MAG_SI_ZZ_1		(1<<1)
#define MAG_SI_ZZ_0		(1<<0)

#define MAG_OFFX_L_7	(1<<7)
#define MAG_OFFX_L_6	(1<<6)
#define MAG_OFFX_L_5	(1<<5)
#define MAG_OFFX_L_4	(1<<4)
#define MAG_OFFX_L_3	(1<<3)
#define MAG_OFFX_L_2	(1<<2)
#define MAG_OFFX_L_1	(1<<1)
#define MAG_OFFX_L_0	(1<<0)

#define MAG_OFFX_H_7	(1<<7)
#define MAG_OFFX_H_6	(1<<6)
#define MAG_OFFX_H_5	(1<<5)
#define MAG_OFFX_H_4	(1<<4)
#define MAG_OFFX_H_3	(1<<3)
#define MAG_OFFX_H_2	(1<<2)
#define MAG_OFFX_H_1	(1<<1)
#define MAG_OFFX_H_0	(1<<0)

#define MAG_OFFY_L_7	(1<<7)
#define MAG_OFFY_L_6	(1<<6)
#define MAG_OFFY_L_5	(1<<5)
#define MAG_OFFY_L_4	(1<<4)
#define MAG_OFFY_L_3	(1<<3)
#define MAG_OFFY_L_2	(1<<2)
#define MAG_OFFY_L_1	(1<<1)
#define MAG_OFFY_L_0	(1<<0)

#define MAG_OFFY_H_7	(1<<7)
#define MAG_OFFY_H_6	(1<<6)
#define MAG_OFFY_H_5	(1<<5)
#define MAG_OFFY_H_4	(1<<4)
#define MAG_OFFY_H_3	(1<<3)
#define MAG_OFFY_H_2	(1<<2)
#define MAG_OFFY_H_1	(1<<1)
#define MAG_OFFY_H_0	(1<<0)

#define MAG_OFFZ_L_7	(1<<7)
#define MAG_OFFZ_L_6	(1<<6)
#define MAG_OFFZ_L_5	(1<<5)
#define MAG_OFFZ_L_4	(1<<4)
#define MAG_OFFZ_L_3	(1<<3)
#define MAG_OFFZ_L_2	(1<<2)
#define MAG_OFFZ_L_1	(1<<1)
#define MAG_OFFZ_L_0	(1<<0)

#define MAG_OFFZ_H_7	(1<<7)
#define MAG_OFFZ_H_6	(1<<6)
#define MAG_OFFZ_H_5	(1<<5)
#define MAG_OFFZ_H_4	(1<<4)
#define MAG_OFFZ_H_3	(1<<3)
#define MAG_OFFZ_H_2	(1<<2)
#define MAG_OFFZ_H_1	(1<<1)
#define MAG_OFFZ_H_0	(1<<0)

