//
// Created by Jonathan He on 4/3/24.
// how to configure serial IO:
// https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
//

#include <iostream>
#include <mqueue.h>

#include <errno.h>   // Error integer and strerror() function
#include <fcntl.h>   // Contains file controls like O_RDWR
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()

#include "baja_live_comm.pb.h"
#include "ipc_config.h"

#define SERIAL_PORT "/dev/ttyACM0"

using namespace std;

int main()
{
    cout << "starting uart emit for rtcm corrections..." << endl;

    // Open queue
    const mqd_t data_queue = BajaIPC::open_queue(CarIPC::BROKER_TO_SD_WRITER_QUEUE, true);
    if (data_queue == -1) {
        cout << "Failed to get data queue. Errno " << errno << endl;
        return EXIT_FAILURE;
    }

    // Open UART Port
    int serial_port = open(SERIAL_PORT, O_RDWR);

    // Check for errors
    if (serial_port < 0) {
        printf("Error %i from open: %s\n", errno, strerror(errno));
    }

    // Main loop
    while (true) {
        string msg = BajaIPC::get_message(data_queue); // Blocking
        if (msg == "") {
            continue;
        }

        Observation observation;
        observation.ParseFromString(msg);

        if (!observation.has_rtk_correction())
            continue;

        RTKCorrection rtk = observation.rtk_correction();
        int ts            = observation.timestamp().ts();

        string rtk_correction  = rtk.rtk_correction();
        unsigned char* rtk_msg = (unsigned char*)(rtk_correction.c_str());

        write(serial_port, rtk_msg, sizeof(rtk_msg) - sizeof(unsigned char)); // minus 1 get rid of '\0'

        for (auto e : rtk_correction)
            cout << hex << (int)e << " ";
        cout << endl;
    }
    return EXIT_SUCCESS;
}