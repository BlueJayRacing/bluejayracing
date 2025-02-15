#ifndef _FILE_WRITER_HPP_
#define _FILE_WRITER_HPP_

#include <config.hpp>

typedef struct file_channel_state {
    const std::string file_name;
    void* const buf_ptr;
    const uint32_t buf_size;
    uint32_t buf_index;
    uint32_t time_offset_at_start;
} file_channel_state_t;

class fileWriter {
  public:
    fileWriter(void);
    ~fileWriter(void);
    int32_t addChannel(const access_channel_config_t& chan_config);
    int32_t logChannel(uint8_t chan_index, uint32_t data_val);
    int32_t getChannelSize(uint8_t chan_index);
    int32_t getChannelDataSize(uint8_t chan_index);
    int32_t updateTime(uint64_t new_time);
    int32_t getSpaceFree(void);

  private:
    void writeChannel(uint8_t chan_index);
    std::vector<file_channel_state_t> file_channels_;
    uint64_t time_offset_;

  private:
    const uint32_t PSRAM_MAX_SIZE = 16000000;
};

#endif