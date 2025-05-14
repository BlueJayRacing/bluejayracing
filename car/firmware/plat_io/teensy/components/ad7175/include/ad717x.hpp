/******************************************************************************
 *   @file    AD717X.hpp
 *   @brief   AD717X header file.
 *   	      (Sort of) Compatible devices: AD7172-2, AD7172-4, AD7173-8,
 *            AD7175-2, AD7175-8, AD7176-2, AD7177-2, AD4111, AD4112, AD4114,
 *            AD4115, AD4116 (may need some library changes)
 *T           Tested devices: AD7175-8
 *   @author  tchen (travis.yu.chen@gmail.com)
 *			  Credit to the following for the base no_os AD717X library:
 *            acozma (andrei.cozma@analog.com)
 *            dnechita (dan.nechita@analog.com)
 *******************************************************************************/
#pragma once
#ifndef _AD717X_HPP_
#define _AD717X_HPP_

#include <Arduino.h>
#include <SPI.h>

#include <array>
#include <stdbool.h>
#include <stdint.h>

/*
 *@enum	ad717x_mode
 *@details ADC Modes of Operation
 **/
typedef enum ad717x_mode
{
    CONTINUOUS,            /* Continuous Mode- Default */
    SINGLE,                /* Single Mode */
    STANDBY,               /* Stand-by Mode */
    POWER_DOWN,            /* Power Down Mode */
    INTERNAL_OFFSET_CALIB, /* Internal Offset Calibration*/
    INTERNAL_GAIN_CALIB,   /* Internal Gain Calibration */
    SYS_OFFSET_CALIB,      /* System Offset Calibraion */
    SYS_GAIN_CALIB         /* System Gain Calibration */
} ad717x_mode_t;

/*
 *@enum	ad717x_reference_source
 *@details Type of ADC Reference
 **/
typedef enum ad717x_ref_source
{
    EXTERNAL_REF = 0x0, /* External Reference REF+/-*/
    EXTERNAL_REF2 = 0x1, /* External Reference REF2+/-*/
    INTERNAL_REF = 0x2, /* Internal 2.5V Reference */
    AVDD_AVSS    = 0x3  /* AVDD - AVSS */
} ad717x_ref_source_t;

/*
 *@enum	ad717x_analog_input_pairs
 *@details Analog Input Pairs to channels for the AD411X Family
 **/
typedef enum ad717x_analog_input_pairs
{
    VIN0_VIN1          = 0x1,
    VIN0_VINCOM        = 0x10,
    VIN1_VIN0          = 0x20,
    VIN1_VINCOM        = 0x30,
    VIN2_VIN3          = 0x43,
    VIN2_VINCOM        = 0x50,
    VIN3_VIN2          = 0x62,
    VIN3_VINCOM        = 0x70,
    VIN4_VIN5          = 0x85,
    VIN4_VINCOM        = 0x90,
    VIN5_VIN4          = 0xA4,
    VIN5_VINCOM        = 0xB0,
    VIN6_VIN7          = 0xC7,
    VIN6_VINCOM        = 0xD0,
    VIN7_VIN6          = 0xE6,
    VIN7_VINCOM        = 0xF0,
    IIN3P_IIN3M        = 0x18B,
    IIN2P_IIN2M        = 0x1AA,
    IIN1P_IIN1M        = 0x1C9,
    IIN0P_IIN0M        = 0x1E8,
    TEMPERATURE_SENSOR = 0x232,
    REFERENCE          = 0x2B6
} ad717x_analog_input_pairs_t;

/*
 *@enum	ad717x_analog_input
 *@details Positive/Negative Analog Input to channels for the AD717x Family
 **/
typedef enum ad717x_analog_input
{
    AIN0          = 0x0,
    AIN1          = 0x1,
    AIN2          = 0x2,
    AIN3          = 0x3,
    AIN4          = 0x4,
    AIN5          = 0x5,
    AIN6          = 0x6,
    AIN7          = 0x7,
    AIN8          = 0x8,
    AIN9          = 0x9,
    AIN10         = 0x0A,
    AIN11         = 0x0B,
    AIN12         = 0x0C,
    AIN13         = 0x0D,
    AIN14         = 0x0E,
    AIN15         = 0x0F,
    AIN16         = 0x10,
    TEMP_SENSOR_P = 0x11,
    TEMP_SENSOR_M = 0x12,
    AVDD_AVSS_P   = 0x13,
    AVDD_AVSS_M   = 0x14,
    REF_P         = 0x15,
    REF_M         = 0x16,
} ad717x_analog_input_t;

/*
 *@union ad717x_analog_inputs
 *@details Types of Analog Inputs
 **/
typedef union ad717x_analog_inputs {
    ad717x_analog_input_pairs_t input_pairs;
    struct {
        ad717x_analog_input_t pos_input;
        ad717x_analog_input_t neg_input;
    } ainp;
} ad717x_analog_inputs_t;

/*
 *@enum	ad717x_device_type
 *@details AD717x-AD411x Device definitions
 **/
typedef enum ad717x_device_type
{
    ID_AD4111,
    ID_AD4112,
    ID_AD4114,
    ID_AD4115,
    ID_AD4116,
    ID_AD7172_2,
    ID_AD7172_4,
    ID_AD7173_8,
    ID_AD7175_2,
    ID_AD7175_8,
    ID_AD7176_2,
    ID_AD7177_2
} ad717x_device_type_t;

/*
 *@enum ad717x_enhfilt
 *@details Post filter for enhanced rejection
 **/
typedef enum ad717x_enhfilt
{
    SPS27_DB47_MS36P7 = 0x2,
    SPS25_DB62_MS40   = 0x3,
    SPS20_DB86_MS50   = 0x5,
    SPS16P6_db82_MS60 = 0x6
} ad717x_enhfilt_t;

/*
 *@enum ad717x_order
 *@details Order of digital filter
 **/
typedef enum ad717x_order
{
    SINC5_SINC1 = 0x0,
    SINC3       = 0x3
} ad717x_order_t;

/*
 *@enum ad717x_odr
 *@details Output data rate
 * Note: only some data rates below are supported for each version of AD717X.
 * Refer to the datasheet to know the exact output data rate mappings.
 **/
typedef enum ad717x_odr
{
    SPS_250000  = 0x0,
    SPS_125000  = 0x1,
    SPS_62500   = 0x2,
    SPS_50000   = 0x3,
    SPS_31250   = 0x4,
    SPS_25000   = 0x5,
    SPS_15625   = 0x6,
    SPS_10000   = 0x7,
    SPS_5000    = 0x8,
    SPS_2500    = 0x9,
    SPS_1000    = 0xA,
    SPS_500     = 0xB,
    SPS_400     = 0xC,
    SPS_200     = 0xD,
    SPS_100     = 0xE,
    SPS_60      = 0xF,
    SPS_50      = 0x10,
    SPS_20      = 0x11,
    SPS_16      = 0x12,
    SPS_10      = 0x13,
    SPS_5       = 0x14,
    SPS_2P5     = 0x15,
    SPS_1P25    = 0x16
} ad717x_odr_t;

typedef enum ad717x_crc_mode
{
    AD717X_DISABLE,
    AD717X_USE_CRC,
    AD717X_USE_XOR,
} ad717x_crc_mode_t;

/*! AD717X register info */
typedef struct ad717x_st_reg {
    int32_t addr;
    uint32_t value;
    int32_t size;
} ad717x_st_reg_t;

/*
 *@struct ad717x_channel_map
 *@details Channel mapping (channel register pg 20, also Figure 45A: channel configuration)
 **/
typedef struct ad717x_channel_map {
    bool channel_enable; // channel can be either on or off, independent of input
    uint8_t setup_sel; // choosing setup index
    ad717x_analog_inputs_t inputs;
} ad717x_channel_map_t;

/*
 *@struct ad717x_channel_setup
 *@details Channel setup // pg 20 actually setup configuration
 **/
typedef struct ad717x_channel_config {
    bool bi_polar; // bipolar (neg diff, offset binary) vs unipolar (pos diff, straight binary) mode 
    bool ref_buff; // enable
    bool input_buff; // enable
    ad717x_ref_source_t ref_source; // the min & max value that the input can be, based on reference
} ad717x_channel_config_t;

/*
 *@struct ad717x_filtcon
 *@details Filter configuration (pg 20 Figure 45B: Setup configuration?)
 **/
typedef struct ad717x_filter_config {
    bool sinc3_map;
    bool enhfilten;
    ad717x_enhfilt_t enhfilt;
    ad717x_order_t oder; // Digital filter type
    ad717x_odr_t odr;
} ad717x_filter_config_t;

// pg 21
typedef struct ad717x_setup {
    // no offset register, why
    double gain;
    ad717x_channel_config_t setup;
    ad717x_filter_config_t filter_config; // Not configured
} ad717x_setup_t;

/*
 * The structure describes the device and is used with the ad717x driver.
 * @slave_select_id: The ID of the Slave Select to be passed to the SPI calls.
 * @regs: A reference to the register list of the device that the user must
 *       provide when calling the Setup() function.
 * @num_regs: The length of the register list.
 * @userCRC: Error check type to use on SPI transfers.
 */
typedef struct ad717x_dev {
    bool ref_en;
    ad717x_mode_t mode;
    ad717x_crc_mode_t useCRC;
    ad717x_device_type_t active_device;
    std::vector<ad717x_st_reg_t> regs;
} ad717x_dev_t;

typedef struct {
    bool ref_en;
    bool stat_on_read_en;
    ad717x_mode_t mode;
    ad717x_device_type_t active_device;
    std::vector<ad717x_setup_t> setups;
    std::vector<ad717x_channel_map_t> chan_map;
} ad717x_init_param_t;

typedef struct ad717x_dev_status {
    bool data_ready;
    bool adc_error;
    bool crc_error;
    bool reg_error;
    uint8_t active_channel;
} ad717x_dev_status_t;

typedef struct ad717x_data {
    ad717x_dev_status_t status;
    uint32_t value;
} ad717x_data_t;

/* AD717X Register Map */
#define AD717X_COMM_REG      0x00
#define AD717X_STATUS_REG    0x00
#define AD717X_ADCMODE_REG   0x01
#define AD717X_IFMODE_REG    0x02
#define AD717X_REGCHECK_REG  0x03
#define AD717X_DATA_REG      0x04
#define AD717X_GPIOCON_REG   0x06
#define AD717X_ID_REG        0x07
#define AD717X_CHMAP0_REG    0x10
#define AD717X_CHMAP1_REG    0x11
#define AD717X_CHMAP2_REG    0x12
#define AD717X_CHMAP3_REG    0x13
#define AD717X_CHMAP4_REG    0x14
#define AD717X_CHMAP5_REG    0x15
#define AD717X_CHMAP6_REG    0x16
#define AD717X_CHMAP7_REG    0x17
#define AD717X_CHMAP8_REG    0x18
#define AD717X_CHMAP9_REG    0x19
#define AD717X_CHMAP10_REG   0x1A
#define AD717X_CHMAP11_REG   0x1B
#define AD717X_CHMAP12_REG   0x1C
#define AD717X_CHMAP13_REG   0x1D
#define AD717X_CHMAP14_REG   0x1E
#define AD717X_CHMAP15_REG   0x1F
#define AD717X_SETUPCON0_REG 0x20
#define AD717X_SETUPCON1_REG 0x21
#define AD717X_SETUPCON2_REG 0x22
#define AD717X_SETUPCON3_REG 0x23
#define AD717X_SETUPCON4_REG 0x24
#define AD717X_SETUPCON5_REG 0x25
#define AD717X_SETUPCON6_REG 0x26
#define AD717X_SETUPCON7_REG 0x27
#define AD717X_FILTCON0_REG  0x28
#define AD717X_FILTCON1_REG  0x29
#define AD717X_FILTCON2_REG  0x2A
#define AD717X_FILTCON3_REG  0x2B
#define AD717X_FILTCON4_REG  0x2C
#define AD717X_FILTCON5_REG  0x2D
#define AD717X_FILTCON6_REG  0x2E
#define AD717X_FILTCON7_REG  0x2F
#define AD717X_OFFSET0_REG   0x30
#define AD717X_OFFSET1_REG   0x31
#define AD717X_OFFSET2_REG   0x32
#define AD717X_OFFSET3_REG   0x33
#define AD717X_OFFSET4_REG   0x34
#define AD717X_OFFSET5_REG   0x35
#define AD717X_OFFSET6_REG   0x36
#define AD717X_OFFSET7_REG   0x37
#define AD717X_GAIN0_REG     0x38
#define AD717X_GAIN1_REG     0x39
#define AD717X_GAIN2_REG     0x3A
#define AD717X_GAIN3_REG     0x3B
#define AD717X_GAIN4_REG     0x3C
#define AD717X_GAIN5_REG     0x3D
#define AD717X_GAIN6_REG     0x3E
#define AD717X_GAIN7_REG     0x3F

/* Communication Register bits */
#define AD717X_COMM_REG_WEN   (0 << 7)
#define AD717X_COMM_REG_WR    (0 << 6)
#define AD717X_COMM_REG_RD    (1 << 6)
#define AD717X_COMM_REG_RA(x) ((x) & 0x3F)

/* Status Register bits */
#define AD717X_STATUS_REG_RDY     (1 << 7)
#define AD717X_STATUS_REG_ADC_ERR (1 << 6)
#define AD717X_STATUS_REG_CRC_ERR (1 << 5)
#define AD717X_STATUS_REG_REG_ERR (1 << 4)
#define AD717X_STATUS_REG_CH(x)   ((x) & 0x0F)

/* ADC Mode Register bits */
#define AD717X_ADCMODE_REG_REF_EN    (1 << 15)
#define AD717X_ADCMODE_SING_CYC      (1 << 13)
#define AD717X_ADCMODE_REG_DELAY(x)  (((x) & 0x7) << 8)
#define AD717X_ADCMODE_REG_MODE(x)   (((x) & 0x7) << 4)
#define AD717X_ADCMODE_REG_CLKSEL(x) (((x) & 0x3) << 2)

/* ADC Mode Register additional bits for AD7172-2, AD7172-4, AD4111 and AD4112
 */
#define AD717X_ADCMODE_REG_HIDE_DELAY (1 << 14)

/* Interface Mode Register bits */
#define AD717X_IFMODE_REG_ALT_SYNC    (1 << 12)
#define AD717X_IFMODE_REG_IOSTRENGTH  (1 << 11)
#define AD717X_IFMODE_REG_DOUT_RESET  (1 << 8)
#define AD717X_IFMODE_REG_CONT_READ   (1 << 7)
#define AD717X_IFMODE_REG_DATA_STAT   (1 << 6)
#define AD717X_IFMODE_REG_REG_CHECK   (1 << 5)
#define AD717X_IFMODE_REG_XOR_EN      (0x01 << 2)
#define AD717X_IFMODE_REG_CRC_EN      (0x02 << 2)
#define AD717X_IFMODE_REG_XOR_STAT(x) (((x) & AD717X_IFMODE_REG_XOR_EN) == AD717X_IFMODE_REG_XOR_EN)
#define AD717X_IFMODE_REG_CRC_STAT(x) (((x) & AD717X_IFMODE_REG_CRC_EN) == AD717X_IFMODE_REG_CRC_EN)
#define AD717X_IFMODE_REG_DATA_WL16   (1 << 0)

/* Interface Mode Register additional bits for AD717x family, not for AD411x */
#define AD717X_IFMODE_REG_HIDE_DELAY (1 << 10)

/* GPIO Configuration Register bits */
#define AD717X_GPIOCON_REG_MUX_IO    (1 << 12)
#define AD717X_GPIOCON_REG_SYNC_EN   (1 << 11)
#define AD717X_GPIOCON_REG_ERR_EN(x) (((x) & 0x3) << 9)
#define AD717X_GPIOCON_REG_ERR_DAT   (1 << 8)
#define AD717X_GPIOCON_REG_IP_EN1    (1 << 5)
#define AD717X_GPIOCON_REG_IP_EN0    (1 << 4)
#define AD717X_GPIOCON_REG_OP_EN1    (1 << 3)
#define AD717X_GPIOCON_REG_OP_EN0    (1 << 2)
#define AD717X_GPIOCON_REG_DATA1     (1 << 1)
#define AD717X_GPIOCON_REG_DATA0     (1 << 0)

/* GPIO Configuration Register additional bits for AD7172-4, AD7173-8 */
#define AD717X_GPIOCON_REG_GP_DATA3 (1 << 7)
#define AD717X_GPIOCON_REG_GP_DATA2 (1 << 6)
#define AD717X_GPIOCON_REG_GP_DATA1 (1 << 1)
#define AD717X_GPIOCON_REG_GP_DATA0 (1 << 0)

/* GPIO Configuration Register additional bits for AD7173-8 */
#define AD717X_GPIOCON_REG_PDSW     (1 << 14)
#define AD717X_GPIOCON_REG_OP_EN2_3 (1 << 13)

/* GPIO Configuration Register additional bits for AD4111, AD4112, AD4114,
 * AD4115 */
#define AD4111_GPIOCON_REG_OP_EN0_1 (1 << 13)
#define AD4111_GPIOCON_REG_DATA1    (1 << 7)
#define AD4111_GPIOCON_REG_DATA0    (1 << 6)

/* GPIO Configuration Register additional bits for AD4116 */
#define AD4116_GPIOCON_REG_OP_EN2_3 (1 << 13)
#define AD4116_GPIOCON_REG_DATA3    (1 << 7)
#define AD4116_GPIOCON_REG_DATA2    (1 << 6)

/* GPIO Configuration Register additional bits for AD4111 */
#define AD4111_GPIOCON_REG_OW_EN (1 << 12)

/* Channel Map Register 0-3 bits */
#define AD717X_CHMAP_REG_CH_EN        (1 << 15)
#define AD717X_CHMAP_REG_SETUP_SEL(x) (((x) & 0x7) << 12)
#define AD717X_CHMAP_REG_AINPOS(x)    (((x) & 0x1F) << 5)
#define AD717X_CHMAP_REG_AINNEG(x)    (((x) & 0x1F) << 0)

/* Channel Map Register additional bits for AD4111, AD4112, AD4114, AD4115,
 * AD4116 */
#define AD4111_CHMAP_REG_INPUT(x) (((x) & 0x3FF) << 0)

/* Setup Configuration Register 0-3 bits */
#define AD717X_SETUP_CONF_REG_BI_UNIPOLAR (1 << 12)
#define AD717X_SETUP_CONF_REG_REF_SEL(x)  (((x) & 0x3) << 4)

/* Setup Configuration Register additional bits for AD7173-8 */
#define AD717X_SETUP_CONF_REG_REF_BUF(x) (((x) & 0x3) << 10)
#define AD717X_SETUP_CONF_REG_AIN_BUF(x) (((x) & 0x3) << 8)
#define AD717X_SETUP_CONF_REG_BURNOUT_EN (1 << 7)
#define AD717X_SETUP_CONF_REG_BUFCHOPMAX (1 << 6)

/* Setup Configuration Register additional bits for AD7172-2, AD7172-4, AD7175-2
 */
#define AD717X_SETUP_CONF_REG_REFBUF_P (1 << 11)
#define AD717X_SETUP_CONF_REG_REFBUF_N (1 << 10)
#define AD717X_SETUP_CONF_REG_AINBUF_P (1 << 9)
#define AD717X_SETUP_CONF_REG_AINBUF_N (1 << 8)

/* Setup Configuration Register additional bits for AD4111, AD4112, AD4114,
 * AD4115, AD4116 */
#define AD4111_SETUP_CONF_REG_REFPOS_BUF (1 << 11)
#define AD4111_SETUP_CONF_REG_REFNEG_BUF (1 << 10)
#define AD4111_SETUP_CONF_REG_AIN_BUF(x) (((x) & 0x3) << 8)
#define AD4111_SETUP_CONF_REG_BUFCHOPMAX (1 << 6)

/* Filter Configuration Register 0-3 bits */
#define AD717X_FILT_CONF_REG_SINC3_MAP  (1 << 15)
#define AD717X_FILT_CONF_REG_ENHFILTEN  (1 << 11)
#define AD717X_FILT_CONF_REG_ENHFILT(x) (((x) & 0x7) << 8)
#define AD717X_FILT_CONF_REG_ORDER(x)   (((x) & 0x3) << 5)
#define AD717X_FILT_CONF_REG_ODR(x)     (((x) & 0x1F) << 0)

/* ID register mask for relevant bits */
#define AD717X_ID_REG_MASK 0xFFF0
/* AD7172-2 ID */
#define AD7172_2_ID_REG_VALUE 0x00D0
/* AD7172-4 ID */
#define AD7172_4_ID_REG_VALUE 0x2050
/* AD7173-8 ID */
#define AD7173_8_ID_REG_VALUE 0x30D0
/* AD7175-2 ID */
#define AD7175_2_ID_REG_VALUE 0x0CD0
/* AD7175-8 ID */
#define AD7175_8_ID_REG_VALUE 0x3CD0
/* AD7176-2 ID */
#define AD7176_2_ID_REG_VALUE 0x0C90
/* AD7177-2 ID */
#define AD7177_2_ID_REG_VALUE 0x4FD0
/* AD4111, AD4112 IDs */
#define AD411X_ID_REG_VALUE 0x30D0
/* AD4114, AD4115 IDs */
#define AD4114_5_ID_REG_VALUE 0x31D0
/* AD4116 ID */
#define AD4116_ID_REG_VALUE 0x34D0

/*****************************************************************************/
/******************* AD717X Constants ****************************************/
/*****************************************************************************/
#define AD717X_CRC8_POLYNOMIAL_REPRESENTATION 0x07 /* x8 + x2 + x + 1 */
/* Timeout for ADC Conversion */
#define AD717X_CONV_TIMEOUT 10000

#define EINVAL 22

#define NO_OS_BITS_PER_LONG 32

#define NO_OS_GENMASK(h, l)                                                                                            \
    ({                                                                                                                 \
        uint32_t t = (uint32_t)(~0UL);                                                                                 \
        t          = t << (NO_OS_BITS_PER_LONG - (h - l + 1));                                                         \
        t          = t >> (NO_OS_BITS_PER_LONG - (h + 1));                                                             \
        t;                                                                                                             \
    })

#define AD717x_CHANNEL_INPUT_MASK         NO_OS_GENMASK(9, 0)
#define AD717X_CHMAP_REG_SETUP_SEL_MSK    NO_OS_GENMASK(14, 12)
#define AD717X_CHMAP_REG_AINPOS_MSK       NO_OS_GENMASK(9, 5)
#define AD717X_CHMAP_REG_AINNEG_MSK       NO_OS_GENMASK(4, 0)
#define AD717X_ADCMODE_REG_MODE_MSK       NO_OS_GENMASK(6, 4)
#define AD717X_SETUP_CONF_REG_REF_SEL_MSK NO_OS_GENMASK(5, 4)
#define AD717x_ODR_MSK                    NO_OS_GENMASK(4, 0)

class AD717X {
  public:
    AD717X() : spi_host_(&SPI), settings_(8000000, MSBFIRST, SPI_MODE3) {};
    ~AD717X();

    int32_t init(ad717x_init_param_t& t_init_param, SPIClass* t_spi_host, int8_t t_cs_pin);
    int32_t init(ad717x_init_param_t& t_init_param, SPIClass* t_spi_host, int8_t t_cs_pin, SPISettings t_settings);
    int32_t readRegister(uint8_t t_addr);
    int32_t writeRegister(uint8_t t_addr);
    int32_t reset(void);
    int32_t waitForReady(uint32_t t_timeout);
    int32_t contConvReadData(ad717x_data_t* t_p_data);
    int32_t setChannelStatus(uint8_t t_channel_id, bool t_channel_status);
    int32_t setADCMode(ad717x_mode_t t_mode);
    int32_t connectAnalogInput(uint8_t t_channel_id, ad717x_analog_inputs_t t_analog_input);
    int32_t assignSetup(uint8_t t_channel_id, uint8_t t_setup_id);
    int32_t setPolarity(bool t_bipolar, uint8_t t_setup_id);
    int32_t setReferenceSource(ad717x_ref_source_t t_ref_source, uint8_t t_setup_id);
    int32_t enableBuffers(bool t_inbuf_en, bool t_refbuf_en, uint8_t t_setup_id);
    int32_t singleReadData(uint8_t t_channel_id, ad717x_data_t* t_p_data);
    int32_t configureDeviceODR(uint8_t t_filtcon_id, uint8_t t_odr_sel);
    int32_t setGain(double gain, uint8_t t_setup_id);
    int32_t configureFilter(ad717x_filter_config_t t_filt_config, uint8_t t_setup_id);
    int32_t readStatusRegOnData(bool enable);
    void parseStatusReg(ad717x_dev_status_t* dev_status);

    ad717x_st_reg* getReg(uint8_t t_reg_address);
  private:
    
    int32_t initRegs(ad717x_device_type_t t_dev_type);
    int32_t updateCRCSetting(void);
    int32_t computeDataregSize(void);
    static uint8_t computeCRC8(uint8_t* t_p_buf, uint8_t t_buf_size);
    static uint8_t computeXOR8(uint8_t* t_p_buf, uint8_t t_buf_size);

  private:
    ad717x_dev_t device_;
    SPIClass* spi_host_;
    SPISettings settings_;
    int8_t cs_pin_;
};

#endif /* _AD717X_H_ */