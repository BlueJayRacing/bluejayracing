
/**
 * @file /example1/main.cpp
 * @author Philippe Lucidarme
 * @date December 2019
 * @brief File containing example of serial port communication
 *
 * This example send the ASCII table through the serial device
 *
 * @see https://lucidar.me
 */


// Serial library
#include "serialib/serialib.h"
#include <unistd.h>
#include <stdio.h>
#include <cassert>
using namespace std;

#if defined (_WIN32) || defined(_WIN64)
    //for serial ports above "COM9", we must use this extended syntax of "\\.\COMx".
    //also works for COM0 to COM9.
    //https://docs.microsoft.com/en-us/windows/win32/api/fileapi/nf-fileapi-createfilea?redirectedfrom=MSDN#communications-resources
    #define SERIAL_PORT "\\\\.\\COM1"
#endif
#if defined (__linux__) || defined(__APPLE__)
    #define SERIAL_PORT "/dev/ttyACM0"
#endif



/*!
 * \brief main  Simple example that send ASCII characters to the serial device
 * \return      0 : success
 *              <0 : an error occured
 */
int main( /*int argc, char *argv[]*/)
{
    // Serial object
    serialib serial;


    // Connection to serial port
    char errorOpening = serial.openDevice(SERIAL_PORT, 115200);


    // If connection fails, return the error code otherwise, display a success message
    if (errorOpening!=1) {
	printf("Error Opening");	
	return errorOpening;
    }
    printf ("Successful connection to %s\n",SERIAL_PORT);


    int counter = 0;

    // Scan for RTCM messages and display them
    uint8_t last = 0x00;

    while (true) {
        uint8_t cur;
        serial.readBytes((void*)&cur, 1);
        
        if (last != 0xd3 || (cur >> 2) != 0x00) {
                last = cur;
                continue;
        }

	uint8_t length_buffer;
	serial.readBytes((void*)&length_buffer, 1);
	
	// calculate length from the 10 bits following d3
	uint16_t length = ((uint16_t)(cur&0x03)<<8) + length_buffer;
	
        // read data and parity
	length += 3;
	uint8_t* data = (uint8_t*)malloc((length+3)* sizeof(uint8_t)); // add another 3 for the first three bytes
	assert(serial.readBytes((void*)(data+3), length));
	
	// print the whole message
	printf("message #%d, length %d:\n", ++counter, length + 3);
	data[0] = last;
	data[1] = cur;
	data[2] = length_buffer;
	for(uint8_t* ptr = data; ptr < data + length; ptr++) printf("%02x ", *ptr);
	printf("\n");

	last = 0x00;
	free(data);
    }
    
    // Close the serial device
    serial.closeDevice();

    return 0 ;
}
