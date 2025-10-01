#include <Arduino.h>
#include <magnetometer.hpp>

#include "util/ring_buffer.hpp"
#include "util/circular_buffer.hpp"
#include "config/config.hpp"
#include "util/sample_data.hpp"
#include "config/config.hpp"
#include "util/debug_util.hpp"
#include "mag/mag_id_config.hpp"


namespace baja {
namespace mag {

#define AH_BUFFERBAD     1 /* No error */
#define AH_OK            0 /* No error */
#define AH_INVALID_VAL  -1 /* Invalid argument */

void store_float_as_uint32(float * f, uint32_t * u32);

void store_uint32_as_float(uint32_t * u32, float * f);

class MagHandler {
public:
    MagHandler(buffer::RingBuffer<data::ChannelSample, baja::config::SAMPLE_RING_BUFFER_SIZE>& ringBuffer, buffer::CircularBuffer<data::ChannelSample, config::FAST_BUFFER_SIZE>& fastBuffer, int mag_id, TwoWire& wire);

    ~MagHandler();

    bool begin();

    bool pollAndProcess();

    void readMagAsUint32();

    uint64_t getSampleCount() const;

    void resetSampleCount();

private:
    buffer::RingBuffer<data::ChannelSample, baja::config::SAMPLE_RING_BUFFER_SIZE>& ringBuffer_;
    buffer::CircularBuffer<data::ChannelSample, config::FAST_BUFFER_SIZE>& fastBuffer_;
    float data_x, data_y, data_z;
    uint32_t conv_data_x, conv_data_y, conv_data_z;
    Magnetometer mag;
    uint64_t samplingCount;
    int mag_id_;
    TwoWire& wire_;

    uint64_t start_micro;

    uint32_t start_samples;
};


}
}