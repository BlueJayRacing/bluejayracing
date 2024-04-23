#ifndef LIS3MDL_HPP
#define LIS3MDL_HPP


#define LIS3MDL_ADDRESS 0x1C
#define LIS3MDL_ADDRESS2 0x1E
#define LIS3MDL_DATA_REGISTER 0x28
#define LIS3MDL_WHO_AM_I 0x0F

// IDS
#define LIS3MDL_ID 0b00111101


// Register Addresses
#define LIS3MDL_REG_ONE 0x20
#define LIS3MDL_REG_TWO 0x21
#define LIS3MDL_REG_THREE 0x22
#define LIS3MDL_REG_FOUR 0x23
#define LIS3MDL_REG_FIVE 0x24

//#define LIS3MDL_READ_LENGTH 0x06 // hello?????

// REG_1 
#define ODR_FAST 0b00000010 // output data rate, fast, low performance + odr_fast enabled  = 1000 Hz
//#define OM_XY 0b00100000 // medium performance + odr_fast enabled = 560 Hz (hopefully)
#define SELF_TEST 0b00000001 

// REG_2 operations
#define FS 0b01100000 // full-scale selection (+/-) 16 in gauss

// REG_3
#define OP_MODE_ON 0b00000000 // operating mode (continuous conversion)
#define OP_MODE_OFF 0b00000010 // operating mode (power-down mode)
#define LP_MODE 0b00100000 // low power mode

// REG_4
#define OMZ_LOW 0b00000000 // low power mode (default)
#define OMZ_HIGH 0b00001000 // high power mode

// REG_5
#define FAST_READ_ON 0b10000000 // fast read on
#define FAST_READ_OFF 0b00000000 // fast read of
#define BDU 0b00000000 // continuous data update

#endif

/* Questions
    Operating modes (X&Y, Z)? (REG_2, REG_4)
    BLE (Big/little endian data selection) 
*/