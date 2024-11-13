#include <memoryBlock.hpp>

#define MIN(a, b) ((a) < (b) ? (a) : (b))

/*******************************************************************************
 * @brief Write a vector of data onto the memory block.
 *
 * @return Returns the number of bytes written.
 *******************************************************************************/
int32_t memoryBlock::write(const std::vector<uint8_t>& t_new_data)
{
    int num_bytes = MIN(t_new_data.size(), bytesLeft());

    std::copy(t_new_data.begin(), t_new_data.begin() + num_bytes, data_.begin() + index_);
    index_ += num_bytes;

    return num_bytes;
}

/*******************************************************************************
 * @brief Push a memory block that's been used back to the queue.
 *
 * @return Returns 0 for success or negative error code.
 *******************************************************************************/
uint8_t* memoryBlock::data(void) { return data_.data(); }

/*******************************************************************************
 * @brief Push a memory block that's been used back to the queue.
 *
 * @return Returns 0 for success or negative error code.
 *******************************************************************************/
int32_t memoryBlock::size(void) { return data_.size(); }

/*******************************************************************************
 * @brief Push a memory block that's been used back to the queue.
 *
 * @return Returns 0 for success or negative error code.
 *******************************************************************************/
int32_t memoryBlock::bytesLeft(void) { return data_.size() - index_; }

/*******************************************************************************
 * @brief Push a memory block that's been used back to the queue.
 *
 * @return Returns 0 for success or negative error code.
 *******************************************************************************/
void memoryBlock::clear(void) { index_ = 0; }