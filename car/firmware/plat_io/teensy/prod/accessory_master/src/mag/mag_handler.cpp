#include "mag/mag_handler.hpp"
#include "util/time_since_epoch.hpp"

namespace baja {
namespace mag {
    const uint32_t DATA_OFFSET = 1<<15;

    MagHandler::MagHandler(buffer::RingBuffer<data::ChannelSample, baja::config::SAMPLE_RING_BUFFER_SIZE>& ringBuffer, buffer::CircularBuffer<data::ChannelSample, config::FAST_BUFFER_SIZE>& fastBuffer, int mag_id, TwoWire& wire) : 
        ringBuffer_(ringBuffer),
        fastBuffer_(fastBuffer),
        mag(wire, MAG_ADDR_MAP.at(mag_id)),
        samplingCount(0),
        mag_id_(mag_id) {}

    bool MagHandler::begin() {
        if (!mag.begin()) {
            util::Debug::error("Magnetometer initialization failed");
            return false;
        }

        return true;
    }

    bool MagHandler::pollAndProcess() {
        uint64_t time = getMicrosecondsSinceEpoch();
        uint32_t milli = millis();

        if (!mag.readRawMag(data_x, data_y, data_z)) {
            util::Debug::error("Magnetometer read failed");
            return false;
        }

        util::Debug::info("Magdata: " + String(data_x) + ", " + String(data_y) + ", " + String(data_z));

        data::ChannelSample channelSampleX(
            time,
            30 + 3 * mag_id_,
            data_x + DATA_OFFSET,
            milli
        );

        data::ChannelSample channelSampleY(
            time,
            30 + 3 * mag_id_ + 1,
            data_y + DATA_OFFSET,
            milli
        );

        data::ChannelSample channelSampleZ(
            time,
            30 + 3 * mag_id_ + 2,
            data_z + DATA_OFFSET,
            milli
        );

        if (!ringBuffer_.write(channelSampleX)) {
            return AH_BUFFERBAD;
        }

        if (!ringBuffer_.write(channelSampleY)) {
            return AH_BUFFERBAD;
        }

        if (!ringBuffer_.write(channelSampleZ)) {
            return AH_BUFFERBAD;
        }

        if (samplingCount % config::FAST_BUFFER_DOWNSAMPLE_RATIO == 0) {
            if (!fastBuffer_.write(channelSampleX)) {
                
            }

            if (!fastBuffer_.write(channelSampleY)) {

            }

            if (!fastBuffer_.write(channelSampleZ)) {

            }
        }

        samplingCount++;


        return true;
    }

    uint64_t MagHandler::getSampleCount() const {
        return samplingCount;
    }

    void MagHandler::resetSampleCount() {
        samplingCount = 0;
    }
}
}