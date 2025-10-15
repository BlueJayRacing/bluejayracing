#include "mag/mag_handler.hpp"
#include "util/time_since_epoch.hpp"

namespace baja {
namespace mag {
    void store_float_as_uint32(float * f, uint32_t * u32) {
        memcpy(u32, f, sizeof(uint32_t));
    }

    void store_uint32_as_float(uint32_t * u32, float * f) {
        memcpy(f, u32, sizeof(uint32_t));
    }

    MagHandler::MagHandler(buffer::RingBuffer<data::ChannelSample, baja::config::SAMPLE_RING_BUFFER_SIZE>& ringBuffer, buffer::CircularBuffer<data::ChannelSample, config::FAST_BUFFER_SIZE>& fastBuffer, int mag_id, TwoWire& wire) : 
        ringBuffer_(ringBuffer),
        fastBuffer_(fastBuffer),
        samplingCount(0),
        mag_id_(mag_id),
        wire_(wire),
        start_micro(micros()),
        mag(Wire, MAG_ADDR_MAP.at(mag_id)) {}

    bool MagHandler::begin() {
        // mag.begin(wire_, MAG_ADDR_MAP.at(mag_id_));
        Wire.begin();
        return mag.begin();

        // return true;
    }

    void MagHandler::readMagAsUint32() {
        mag.readRawMag(data_x, data_y, data_z);
        conv_data_x = data_x<<16;
        conv_data_y = data_y<<16;
        conv_data_z = data_z<<16;

        // store_float_as_uint32(&data_x, &conv_data_x);
        // store_float_as_uint32(&data_y, &conv_data_y);
        // store_float_as_uint32(&data_z, &conv_data_z);
    }

    bool MagHandler::pollAndProcess() {
        uint64_t time = getMicrosecondsSinceEpoch();

        // testing sampling rate
        uint64_t diff = time - start_micro;
        if (samplingCount-start_samples == 1000 && mag_id_ == 0) {
            float rate = (float) (samplingCount - start_samples) * 1000000. / diff;
            util::Debug::info("Sampling rate: " + String(rate));
            start_micro = time;
            start_samples = samplingCount;
        }

        uint32_t milli = millis();

        uint64_t mictest = micros();
        readMagAsUint32();
        uint64_t testdiff = micros() - mictest;

        if(mag_id_== 0 && samplingCount % 150 == 0)
            util::Debug::info(String(testdiff));
        //     util::Debug::info("Magdata " + String(mag_id_) + ": " + String(data_x) + "-" + String(conv_data_x) + ", " + String(data_y) + "-" + String(conv_data_y) + ", " + String(data_z) + "-" + String(conv_data_z));

        data::ChannelSample channelSampleX(
            time,
            30 + 3 * mag_id_,
            conv_data_x,
            milli
        );

        data::ChannelSample channelSampleY(
            time,
            30 + 3 * mag_id_ + 1,
            conv_data_y,
            milli
        );

        data::ChannelSample channelSampleZ(
            time,
            30 + 3 * mag_id_ + 2,
            conv_data_z,
            milli
        );

        if (!ringBuffer_.write(channelSampleX)) {
            // return AH_BUFFERBAD;
        }

        if (!ringBuffer_.write(channelSampleY)) {
            // return AH_BUFFERBAD;
        }

        if (!ringBuffer_.write(channelSampleZ)) {
            // return AH_BUFFERBAD;
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