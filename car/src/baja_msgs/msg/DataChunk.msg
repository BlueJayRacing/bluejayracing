uint64 timestamp               # Timestamp when the chunk was created (microseconds)
string source_id               # Unique identifier of the data source (e.g., "teensy1")
UnifiedSample[] samples        # Array of unified samples
uint32 sequence_num            # Sequence number for detecting missed packets