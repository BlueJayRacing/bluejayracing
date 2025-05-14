uint8 device_id                # Unique device identifier assigned by configuration
string device_name             # Semantic name for the device
string mac_address             # Device MAC address
uint8 internal_channel_id      # Teensy internal channel id (0–29)
uint8 ros_channel_id           # Unified ROS channel id (mapped via config)
string internal_channel_name   # Raw channel name (e.g., "AIN0", "DIN3")
string semantic_channel_name   # Semantic name assigned on the Raspberry Pi
uint64 recorded_time           # Timestamp recorded by the device (in microseconds)
uint64 received_time           # Timestamp when received by the Raspberry Pi
float32 data_value              # Sensor data value