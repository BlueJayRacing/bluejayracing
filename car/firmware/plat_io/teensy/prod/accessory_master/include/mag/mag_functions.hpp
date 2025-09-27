#include "mag/mag_handler.hpp"
#include "util/ring_buffer.hpp"
#include "util/circular_buffer.hpp"
#include "util/sample_data.hpp"
#include "config/config.hpp"

namespace baja {
namespace mag {

namespace functions {
    bool initialize(
        buffer::RingBuffer<data::ChannelSample, config::SAMPLE_RING_BUFFER_SIZE>& mainBuffer,
        buffer::CircularBuffer<data::ChannelSample, config::FAST_BUFFER_SIZE>& fastBuffer
    );
    
    bool start();

    bool stop();

    bool isRunning();

    bool processSample();

} // namespace functions

} 
} // namespace baja