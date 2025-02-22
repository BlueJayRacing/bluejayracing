#include <fileWriter.hpp>
#include <SPI.h>
#include <SD.h>

fileWriter::fileWriter(void) {
    time_offset_ = 0; 

    const int chipSelect = BUILTIN_SDCARD;

    if (!SD.begin(chipSelect)) {
        Serial.println("SD initialization failed");
        return;
    }
}

fileWriter::~fileWriter(void) {
    for (int i = 0; i < file_channels_.size(); i++) {
        extmem_free(file_channels_[i].buf_ptr);
    }
}

int32_t fileWriter::addChannel(const access_channel_config_t& channel_config)
{
    int32_t new_buf_size = channel_config.buf_size_kB * (1 << 10);

    // Verify that the channel buffer size is valid
    if (channel_config.buf_size_kB == 0 || getSpaceFree() < new_buf_size) {
        return -1;
    }

    // Create new channel and add it to channel vector
    void* new_data_ptr = extmem_malloc(new_buf_size);
    if (new_data_ptr == NULL) {
        return -2;
    }

    std::string file_name = channel_config.channel_name;

    file_channel_state_t new_chan_state = {file_name, new_data_ptr, new_buf_size, 0, time_offset_};

    file_channels_.push_back(new_chan_state);

    return file_channels_.size() - 1;
}

int32_t fileWriter::logChannel(uint8_t chan_index, uint32_t data_val) {
    if (chan_index >= file_channels_.size()) {
        return -1;
    }

    void* curr_ptr = file_channels_[chan_index].buf_ptr + file_channels_[chan_index].buf_index;

    uint32_t data_point[2] = {micros(), data_val};
    memcpy(curr_ptr, (void*) data_point, sizeof(data_point));

    file_channels_[chan_index].buf_index += sizeof(data_point);

    if (getChannelDataSize(chan_index) == getChannelSize(chan_index)) {
        writeChannel(chan_index);
    }

    return 0;
}

int32_t fileWriter::getChannelSize(uint8_t chan_index) {
    if (chan_index >= file_channels_.size()) {
        return -1;
    } 

    return file_channels_[chan_index].buf_size;
}


int32_t fileWriter::getChannelDataSize(uint8_t chan_index) {
    if (chan_index >= file_channels_.size()) {
        return -1;
    } 

    return file_channels_[chan_index].buf_index;
}

int32_t fileWriter::getSpaceFree(void) {
    uint32_t free_buf_size = PSRAM_MAX_SIZE;

    for (int i = 0; i < file_channels_.size(); i++) {
        free_buf_size -= file_channels_[i].buf_size;
    }

    return free_buf_size;
}

void fileWriter::writeChannel(uint8_t chan_index)
{
    File file = SD.open(file_channels_[chan_index].file_name.data());

    file.write(file_channels_[chan_index].buf_ptr, file_channels_[chan_index].buf_size);

    file.close();

    file_channels_[chan_index].buf_index = 0;
    file_channels_[chan_index].time_offset_at_start = time_offset_;
}