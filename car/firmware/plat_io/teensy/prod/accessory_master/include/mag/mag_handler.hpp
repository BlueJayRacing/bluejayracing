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

class MagHandler {
public:
    MagHandler(buffer::RingBuffer<data::ChannelSample, baja::config::SAMPLE_RING_BUFFER_SIZE>& ringBuffer, buffer::CircularBuffer<data::ChannelSample, config::FAST_BUFFER_SIZE>& fastBuffer, int mag_id, TwoWire& wire);

    ~MagHandler();

    bool begin();

    bool pollAndProcess();

    uint64_t getSampleCount() const;

    void resetSampleCount();

private:
    buffer::RingBuffer<data::ChannelSample, baja::config::SAMPLE_RING_BUFFER_SIZE>& ringBuffer_;
    buffer::CircularBuffer<data::ChannelSample, config::FAST_BUFFER_SIZE>& fastBuffer_;
    int16_t data_x, data_y, data_z;
    Magnetometer mag;
    uint64_t samplingCount;
    int mag_id_;
};


}
}