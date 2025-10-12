#include "mag/mag_functions.hpp"
#include "util/debug_util.hpp"
#include <vector>

namespace baja {
namespace mag {
namespace functions {

static MagHandler* mag_vec[MAG_COUNT];

bool initialize(buffer::RingBuffer<data::ChannelSample, config::SAMPLE_RING_BUFFER_SIZE>& mainBuffer, 
    buffer::CircularBuffer<data::ChannelSample, config::FAST_BUFFER_SIZE>& fastBuffer) {
    util::Debug::info("MAG: Magnetometer initializing");
    Wire.begin();

    for (int i = 0; i < MAG_COUNT; i++) {
        MagHandler * mag = new MagHandler(mainBuffer, fastBuffer, i, Wire);

        if (!mag->begin()) {
            util::Debug::error("MAG " + String(i) + ": Failed to initialize magnetometer");
            continue;
        }

        mag_vec[i] = mag;
    }

    util::Debug::info("MAG: Initialized Mag");
    return true;
}

bool processSample() {
    int success_ct = 0;
    for(int i = 0; i < MAG_COUNT; i++) {
        if(!mag_vec[i]) {
            // util::Debug::error("MAG" + String(i) + ": mag handler is not initialized, id: ");
            continue;
        }

        if(!mag_vec[i]->pollAndProcess()) {
            util::Debug::info("Mag " + String(i) + ": failed to process sample");
        } else {
            success_ct++;
        }
    }

    return success_ct != 0;
}

}
}
}