# DataChunk.msg - supports both fixed (chunk_type=0) and verbose (chunk_type=1) variants

# Indicator: 0 = fixed, 1 = verbose
int8 chunk_type

# FixedDataChunk fields (used when chunk_type==0)
uint64 base_timestamp
uint32 fixed_sample_count
baja_msgs/Sample[] fixed_samples

# VerboseDataChunk fields (used when chunk_type==1)
uint32 verbose_sample_count
# Use fixed-size arrays to hold up to 100 elements (as per nanopb settings)
uint64[50] verbose_timestamps
uint8[50] verbose_channel_ids
uint32[50] verbose_values
