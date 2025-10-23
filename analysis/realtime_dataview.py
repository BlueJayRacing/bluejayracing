#!/usr/bin/env python3
import socket
import struct
import time
import threading
import argparse
from collections import defaultdict, deque
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import binascii
from datetime import datetime

# Configuration constants
DEFAULT_UDP_PORT = 8888
MAX_BUFFER_SIZE = 1500  # bytes
PLOT_WINDOW = 1000      # Number of points to display in the plot window
UPDATE_INTERVAL = 100   # milliseconds between plot updates
VELOCITY_WINDOW = 10    # Number of samples for trailing average velocity

# Voltage scaling configuration
VOLTAGE_MAX = 5.0       # Maximum voltage (5V)
UINT24_MAX = 0xFFFFFF   # Maximum value for a 24-bit unsigned integer (16,777,215)

# Channel name mappings
CHANNEL_NAMES = {
    0: "AIN0 - Ground reference",
    1: "AIN1 - 5V reference",
    2: "AIN2 - 2.5V reference",
    3: "AIN3 - 2.5V reference (buffered)",
    4: "AIN4 - Strain gauge 2",
    5: "AIN5 - Strain gauge 1",
    6: "AIN6 - Channel 1",
    7: "AIN7 - Channel 6",
    8: "AIN8 - Channel 2",
    9: "AIN9 - Channel 7",
    10: "AIN10 - Channel 3",
    11: "AIN11 - Channel 8",
    12: "AIN12 - Channel 4",
    13: "AIN13 - Channel 9",
    14: "AIN14 - Channel 5",
    15: "AIN15 - Channel 10",
    16: "DIN0 - Digital Input 0",
    17: "DIN1 - Digital Input 1",
    18: "DIN2 - Digital Input 2",
    19: "DIN3 - Digital Input 3",
    20: "DIN4 - Digital Input 4",
    21: "DIN5 - Digital Input 5",
    22: "MISC0 - System temperature",
    23: "MISC1 - Power supply",
    24: "MISC2 - CPU load",
    25: "MISC3 - Memory usage",
    26: "MISC4",
    27: "MISC5",
    28: "MISC6",
    29: "MISC7"
}

# Define the channels we want to plot
ANALOG_CHANNELS = [6, 8, 10, 12, 14, 7, 9, 11, 13, 15]  # AIN6-AIN15 (Channel 1-10)
DIGITAL_CHANNELS = [16, 17, 18, 19, 20, 21]             # DIN0-DIN5 (Digital Input 0-5)

class DataChunkParser:
    """
    Parse binary data according to nanopb DataChunk format.
    Implements multiple parsing strategies for the data.
    """
    def __init__(self, verbose=False):
        self.verbose = verbose
        self.packet_count = 0
        self.successful_parses = 0
        self.max_samples = 50  # From proto definition
    
    def parse(self, data, sender_info=""):
        """
        Parse binary data using multiple strategies
        """
        self.packet_count += 1
        
        # Create result structure
        result = {
            'timestamps': [],
            'internal_channel_ids': [],
            'values': [],
            'sample_count': 0,
            'parsed_successfully': False
        }
        
        # Try different parsing strategies
        if len(data) > 0:
            # Log raw data for debugging
            if self.verbose and self.packet_count % 20 == 0:
                print(f"\nRaw packet #{self.packet_count} from {sender_info} ({len(data)} bytes):")
                print(f"Header: {binascii.hexlify(data[:20]).decode()}")
            
            # Try parsing method 1: Look for potential protocol buffer format
            parsed = self.try_protobuf_parse(data)
            if parsed and parsed['sample_count'] > 0:
                result = parsed
                self.successful_parses += 1
                return result
            
            # Try parsing method 2: Simple binary structure
            parsed = self.try_simple_binary_parse(data)
            if parsed and parsed['sample_count'] > 0:
                result = parsed
                self.successful_parses += 1
                return result
            
            # Try parsing method 3: Scan for patterns
            parsed = self.scan_for_patterns(data)
            if parsed and parsed['sample_count'] > 0:
                result = parsed
                self.successful_parses += 1
                return result
            
            if self.verbose and self.packet_count % 100 == 0:
                success_rate = (self.successful_parses / self.packet_count) * 100
                print(f"Parser stats: {self.successful_parses}/{self.packet_count} packets parsed successfully ({success_rate:.1f}%)")
        
        return result
    
    def try_protobuf_parse(self, data):
        """
        Try to parse the data as a protocol buffer message.
        Based on the observed packet patterns.
        """
        try:
            # Based on the hex dump, it looks like field tag 0x0A followed by a length
            result = {
                'timestamps': [],
                'internal_channel_ids': [],
                'values': [],
                'sample_count': 0,
                'parsed_successfully': False
            }
            
            offset = 0
            
            # Try to interpret as protobuf
            if offset < len(data) and data[offset] >> 3 == 1:  # Field 1 (timestamps)
                field_type = data[offset] & 0x07
                offset += 1
                
                # Look for length if it's a length-delimited field
                if field_type == 2:  # Length-delimited field
                    length, bytes_read = self.decode_varint(data[offset:])
                    offset += bytes_read
                    field_end = offset + length
                    
                    # Process data within this field
                    while offset < field_end:
                        # Assume 8-byte timestamps (uint64)
                        if offset + 8 <= field_end:
                            timestamp = struct.unpack('<Q', data[offset:offset+8])[0]
                            result['timestamps'].append(timestamp)
                            offset += 8
                        else:
                            break
            
            # Field 2 (channel ids)
            if offset < len(data) and data[offset] >> 3 == 2:
                field_type = data[offset] & 0x07
                offset += 1
                
                if field_type == 2:  # Length-delimited
                    length, bytes_read = self.decode_varint(data[offset:])
                    offset += bytes_read
                    field_end = offset + length
                    
                    while offset < field_end:
                        channel_id = data[offset]
                        result['internal_channel_ids'].append(channel_id)
                        offset += 1
            
            # Field 3 (values)
            if offset < len(data) and data[offset] >> 3 == 3:
                field_type = data[offset] & 0x07
                offset += 1
                
                if field_type == 2:  # Length-delimited
                    length, bytes_read = self.decode_varint(data[offset:])
                    offset += bytes_read
                    field_end = offset + length
                    
                    while offset < field_end:
                        value, bytes_read = self.decode_varint(data[offset:])
                        result['values'].append(value)
                        offset += bytes_read
            
            # Check for sample count field
            if offset < len(data) and data[offset] >> 3 == 4:
                field_type = data[offset] & 0x07
                offset += 1
                
                if field_type == 0:  # Varint
                    sample_count, bytes_read = self.decode_varint(data[offset:])
                    result['sample_count'] = sample_count
            else:
                # If no explicit sample count, use the length of the shortest array
                result['sample_count'] = min(len(result['timestamps']), 
                                           len(result['internal_channel_ids']), 
                                           len(result['values']))
            
            # Validate the parsed data
            if (result['sample_count'] > 0 and 
                len(result['timestamps']) >= result['sample_count'] and
                len(result['internal_channel_ids']) >= result['sample_count'] and
                len(result['values']) >= result['sample_count']):
                
                # Trim arrays to sample_count
                result['timestamps'] = result['timestamps'][:result['sample_count']]
                result['internal_channel_ids'] = result['internal_channel_ids'][:result['sample_count']]
                result['values'] = result['values'][:result['sample_count']]
                result['parsed_successfully'] = True
                
                return result
        
        except Exception as e:
            if self.verbose:
                print(f"Protobuf parse failed: {str(e)}")
        
        return None
    
    def try_simple_binary_parse(self, data):
        """
        Try parsing the data as a simple binary structure.
        """
        try:
            result = {
                'timestamps': [],
                'internal_channel_ids': [],
                'values': [],
                'sample_count': 0,
                'parsed_successfully': False
            }
            
            # Skip potential header (first 16 bytes)
            offset = 16
            
            # Try to extract data assuming fixed-width fields
            while offset + 12 <= len(data):  # Assuming 8 bytes timestamp, 1 byte channel, 4 bytes value
                try:
                    timestamp = struct.unpack('<Q', data[offset:offset+8])[0]
                    offset += 8
                    
                    channel_id = data[offset]
                    offset += 1
                    
                    # Try different value formats
                    if offset + 4 <= len(data):
                        value = struct.unpack('<I', data[offset:offset+4])[0]
                        offset += 4
                    elif offset + 2 <= len(data):
                        value = struct.unpack('<H', data[offset:offset+2])[0]
                        offset += 2
                    else:
                        value = data[offset]
                        offset += 1
                    
                    # Add to result if channel_id is in our range (0-29)
                    if 0 <= channel_id <= 29:
                        result['timestamps'].append(timestamp)
                        result['internal_channel_ids'].append(channel_id)
                        result['values'].append(value)
                    
                except Exception:
                    # If we hit an error, skip ahead by one byte and try again
                    offset += 1
            
            result['sample_count'] = len(result['timestamps'])
            
            if result['sample_count'] > 0:
                result['parsed_successfully'] = True
                return result
        
        except Exception as e:
            if self.verbose:
                print(f"Simple binary parse failed: {str(e)}")
        
        return None
    
    def scan_for_patterns(self, data):
        """
        Scan through the data looking for patterns that might indicate
        channel data and timestamps.
        """
        try:
            result = {
                'timestamps': [],
                'internal_channel_ids': [],
                'values': [],
                'sample_count': 0,
                'parsed_successfully': False
            }
            
            # Extract channel data by scanning for channel IDs
            found_channels = []
            
            # Scan for channel IDs (0-29) throughout the packet
            for i in range(len(data) - 4):
                if data[i] <= 29:  # Potential channel ID
                    # Check surrounding bytes for potential data
                    if i > 8 and i + 4 < len(data):
                        potential_channel = data[i]
                        
                        # Try to extract timestamp and value
                        try:
                            # Try timestamp as 8 bytes before
                            timestamp = struct.unpack('<Q', data[i-8:i])[0]
                            
                            # Try value as 4 bytes after (uint32)
                            value = struct.unpack('<I', data[i+1:i+5])[0]
                            
                            # If values seem reasonable
                            current_time_us = int(time.time() * 1000000)
                            if timestamp > 0 and timestamp < current_time_us * 2:
                                found_channels.append((timestamp, potential_channel, value, i))
                        except:
                            pass
            
            # If found potential channels, sort by timestamp
            if found_channels:
                found_channels.sort()  # Sort by timestamp
                
                # Extract the data
                for timestamp, channel, value, _ in found_channels:
                    result['timestamps'].append(timestamp)
                    result['internal_channel_ids'].append(channel)
                    result['values'].append(value)
                
                result['sample_count'] = len(result['timestamps'])
                result['parsed_successfully'] = True
                
                return result
        
        except Exception as e:
            if self.verbose:
                print(f"Pattern scanning failed: {str(e)}")
        
        return None
    
    def decode_varint(self, data):
        """Decode a protobuf varint from the data."""
        value = 0
        shift = 0
        for i, byte in enumerate(data):
            value |= ((byte & 0x7F) << shift)
            if not (byte & 0x80):
                return value, i + 1
            shift += 7
            if shift > 64:
                raise ValueError("Varint too long")
        raise ValueError("Incomplete varint")


class UDPDataSpeedPlotter:
    def __init__(self, port=DEFAULT_UDP_PORT, buffer_size=MAX_BUFFER_SIZE, 
                 plot_window=PLOT_WINDOW, show_stats=True, verbose=False):
        """
        Initialize the UDP data plotter for counts, data speed, and digital line velocity.
        """
        self.port = port
        self.buffer_size = buffer_size
        self.plot_window = plot_window
        self.show_stats = show_stats
        self.verbose = verbose
        
        # Create data parser
        self.parser = DataChunkParser(verbose=verbose)
        
        # Data storage for channel counts
        self.data_lock = threading.Lock()
        self.channel_data = defaultdict(lambda: {
            'timestamps': deque(maxlen=plot_window),
            'values': deque(maxlen=plot_window),  # These are the 32-bit uint counts
            'cumulative_values': deque(maxlen=plot_window),  # Running total for digital channels
            'count': 0,
            'last_update': 0,
            'delta_values': deque(maxlen=plot_window),  # For tracking changes in counts
            'velocity': deque(maxlen=plot_window),      # For tracking 10-sample trailing average velocity
            'last_value': 0,
            'total_count': 0,  # Cumulative counter for total events
            'samples_per_second': deque(maxlen=plot_window)  # Speed tracking
        })
        
        # Data rate tracking
        self.speed_data = {
            'timestamps': deque(maxlen=plot_window),
            'packets_per_second': deque(maxlen=plot_window),
            'samples_per_second': deque(maxlen=plot_window),
            'bytes_per_second': deque(maxlen=plot_window)
        }
        
        # Performance tracking
        self.total_samples = 0
        self.total_packets = 0
        self.total_bytes = 0
        self.start_time = time.time()
        self.active_channels = set()
        
        # Rate calculation sliding window
        self.packet_times = deque(maxlen=100)  # Times of last 100 packets
        self.byte_counts = deque(maxlen=100)   # Sizes of last 100 packets
        self.sample_counts = deque(maxlen=100) # Samples in last 100 packets
        
        # Initialize UDP socket
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('0.0.0.0', self.port))
        self.socket.settimeout(0.1)  # Small timeout for non-blocking operation
        
        # Setup plots for analog and digital channels
        self.setup_plots()
        
        # Flag to control the UDP receiving thread
        self.running = True
        
        print(f"UDP Data & Velocity Plotter started on port {self.port}")
        print(f"Waiting for UDP packets...")

    def scan_for_digital_inputs(self, data):
        """
        Special scan function to look specifically for digital input channels (16-21)
        """
        try:
            # Look for digital input bytes in the data stream
            for i in range(len(data) - 1):
                # Check for potential digital input channel IDs (16-21)
                if 16 <= data[i] <= 21:
                    # If the next byte could be a reasonable value (0 or 1 for digital)
                    if i + 1 < len(data) and data[i+1] <= 1:
                        # We found a potential digital input pattern
                        channel_id = data[i]
                        value = data[i+1]
                        
                        if self.verbose:
                            print(f"Potential digital input found: Channel {channel_id}, Value {value}")
                        
                        # Try to construct a timestamp (use current time)
                        timestamp = int(time.time() * 1000000)
                        
                        return {
                            'timestamp': timestamp,
                            'channel_id': channel_id,
                            'value': value
                        }
        except Exception as e:
            if self.verbose:
                print(f"Error scanning for digital inputs: {e}")
        
        return None

    def setup_plots(self):
        """Set up the matplotlib figure with three plots: analog channels, digital channels, and data rate."""
        # Create a figure with 3 rows
        self.fig = plt.figure(figsize=(12, 12))
        gs = self.fig.add_gridspec(3, 1, height_ratios=[2, 2, 1])
        
        # Top subplot: Analog channels (Channel 1-10)
        self.ax_analog = self.fig.add_subplot(gs[0])
        self.ax_analog.set_title("Analog Channels (1-10) Voltage Over Time")
        self.ax_analog.set_xlabel("Time (s)")
        self.ax_analog.set_ylabel("Voltage (V)")
        self.ax_analog.grid(True)
        
        # Lines for each analog channel
        self.analog_lines = {}
        
        # Middle subplot: Digital channels (Digital Input 0-5)
        self.ax_digital = self.fig.add_subplot(gs[1])
        self.ax_digital.set_title("Digital Inputs (0-5) Total Counts")
        self.ax_digital.set_xlabel("Time (s)")
        self.ax_digital.set_ylabel("Total Counts")
        self.ax_digital.grid(True)
        
        # Lines for each digital channel
        self.digital_lines = {}
        
        # Bottom subplot: Data speed
        self.ax_speed = self.fig.add_subplot(gs[2])
        self.ax_speed.set_title("Data Speed")
        self.ax_speed.set_xlabel("Time (s)")
        self.ax_speed.set_ylabel("Rate")
        self.ax_speed.grid(True)
        
        # Lines for data speed metrics
        self.line_packets, = self.ax_speed.plot([], [], lw=2, color='blue', label='Packets/sec')
        self.line_samples, = self.ax_speed.plot([], [], lw=2, color='green', label='Samples/sec')
        self.line_bytes, = self.ax_speed.plot([], [], lw=2, color='red', label='KB/sec')
        
        self.ax_speed.legend(loc='upper right')
        
        # Adjust layout
        plt.tight_layout()
        
        # Create animation
        self.ani = FuncAnimation(
            self.fig, self.update_plot, interval=UPDATE_INTERVAL,
            blit=False, cache_frame_data=False
        )

    def receive_udp_thread(self):
        """Thread function to continuously receive UDP packets."""
        print(f"Starting UDP receiver on 0.0.0.0:{self.port}")
        while self.running:
            try:
                data, addr = self.socket.recvfrom(self.buffer_size)
                
                # Track packet for data rate calculation
                recv_time = time.time()
                self.packet_times.append(recv_time)
                self.byte_counts.append(len(data))
                
                if self.verbose:
                    print(f"Received {len(data)} bytes from {addr[0]}:{addr[1]}")
                    print(f"First 20 bytes: {binascii.hexlify(data[:20]).decode()}")
                
                # Process the datagram normally
                self.process_datagram(data, addr, recv_time)
                
                # Special scan for digital inputs
                digital_input = self.scan_for_digital_inputs(data)
                if digital_input:
                    channel_id = digital_input['channel_id']
                    value = digital_input['value']
                    timestamp = digital_input['timestamp']
                    
                    # Create a synthetic datagram with just this digital input
                    synthetic_chunk = {
                        'timestamps': [timestamp],
                        'internal_channel_ids': [channel_id],
                        'values': [value],
                        'sample_count': 1,
                        'parsed_successfully': True
                    }
                    
                    # Process this synthetic chunk
                    with self.data_lock:
                        relative_time = recv_time - self.start_time
                        
                        # Calculate delta if we have a previous value
                        if self.channel_data[channel_id]['count'] > 0:
                            last_value = self.channel_data[channel_id]['last_value']
                            delta = value - last_value
                            self.channel_data[channel_id]['delta_values'].append(delta)
                            
                            # Only increment total count on rising edge (0->1 transition)
                            if delta > 0:
                                self.channel_data[channel_id]['total_count'] += 1
                            
                            # Calculate trailing average velocity
                            if len(self.channel_data[channel_id]['delta_values']) >= VELOCITY_WINDOW:
                                trailing_avg = sum(list(self.channel_data[channel_id]['delta_values'])[-VELOCITY_WINDOW:]) / VELOCITY_WINDOW
                            else:
                                trailing_avg = sum(self.channel_data[channel_id]['delta_values']) / len(self.channel_data[channel_id]['delta_values'])
                            
                            self.channel_data[channel_id]['velocity'].append(trailing_avg)
                        else:
                            # First value for this channel
                            self.channel_data[channel_id]['delta_values'].append(0)
                            self.channel_data[channel_id]['velocity'].append(0)
                            self.channel_data[channel_id]['total_count'] = 0
                        
                        # Store the data
                        self.channel_data[channel_id]['timestamps'].append(relative_time)
                        self.channel_data[channel_id]['values'].append(value)
                        self.channel_data[channel_id]['cumulative_values'].append(self.channel_data[channel_id]['total_count'])
                        self.channel_data[channel_id]['count'] += 1
                        self.channel_data[channel_id]['last_update'] = relative_time
                        self.channel_data[channel_id]['last_value'] = value
                        
                        # Add to active channels
                        self.active_channels.add(channel_id)
                        
                    if self.verbose:
                        print(f"Processed synthetic digital input: Channel {channel_id}, Value {value}")
                
            except socket.timeout:
                # This is normal due to our socket timeout setting
                pass
            except Exception as e:
                print(f"Error receiving UDP data: {e}")
                import traceback
                traceback.print_exc()

    def convert_to_voltage(self, value):
        """Convert a uint24 value to voltage (0-5V scale)"""
        # Make sure value is treated as float before division
        return (float(value) / UINT24_MAX) * VOLTAGE_MAX
        
    def process_datagram(self, data, addr, recv_time):
        """
        Process a received UDP datagram and update data rate statistics.
        """
        try:
            sender = f"{addr[0]}:{addr[1]}"
            
            # Parse the datagram
            chunk = self.parser.parse(data, sender)
            
            # Check if we got any valid samples
            if not chunk['parsed_successfully'] or chunk['sample_count'] == 0:
                if self.verbose:
                    print(f"Failed to parse DataChunk from {sender}")
                return
            
            # Track samples for data rate calculation
            self.sample_counts.append(chunk['sample_count'])
            
            # Update packet counter
            self.total_packets += 1
            self.total_bytes += len(data)
            
            # Calculate current data rates
            self.calculate_data_rates(recv_time)
            
            with self.data_lock:
                # Process each sample
                for i in range(chunk['sample_count']):
                    channel_id = chunk['internal_channel_ids'][i]
                    timestamp = chunk['timestamps'][i] / 1000000.0  # Convert to seconds
                    value = chunk['values'][i]  # This is the 32-bit uint count
                    
                    # Check for digital channels
                    is_digital_channel = 16 <= channel_id <= 21
                    
                    # Debugging print for digital channels
                    if is_digital_channel and self.verbose:
                        print(f"Digital Channel {channel_id}: value={value}")
                    
                    # For better plotting, we'll use a relative timestamp (time since start)
                    if len(chunk['timestamps']) > 0:
                        relative_time = timestamp - chunk['timestamps'][0]/1000000.0 + (recv_time - self.start_time)
                    else:
                        relative_time = recv_time - self.start_time
                    
                    # Calculate delta (change in count) if we have a previous value
                    if self.channel_data[channel_id]['count'] > 0:
                        last_value = self.channel_data[channel_id]['last_value']
                        
                        # Handle rollover for 32-bit counter if it goes backwards
                        if value < last_value:
                            delta = (0xFFFFFFFF - last_value) + value + 1  # Assuming uint32 rollover
                        else:
                            delta = value - last_value
                        
                        self.channel_data[channel_id]['delta_values'].append(delta)
                        
                                                    # Update total count for this channel
                        if is_digital_channel:
                          self.channel_data[channel_id]['total_count'] = value
                        
                        # Calculate trailing average velocity (10-sample window)
                        if len(self.channel_data[channel_id]['delta_values']) >= VELOCITY_WINDOW:
                            # Use the last 10 samples for the trailing average
                            trailing_avg = sum(list(self.channel_data[channel_id]['delta_values'])[-VELOCITY_WINDOW:]) / VELOCITY_WINDOW
                        else:
                            # If we don't have 10 samples yet, use what we have
                            trailing_avg = sum(self.channel_data[channel_id]['delta_values']) / len(self.channel_data[channel_id]['delta_values'])
                        
                        self.channel_data[channel_id]['velocity'].append(trailing_avg)
                    else:
                        # First value, no delta or velocity
                        self.channel_data[channel_id]['delta_values'].append(0)
                        self.channel_data[channel_id]['velocity'].append(0)
                        self.channel_data[channel_id]['total_count'] = 0  # Initialize total count
                    
                    # Store the appropriate value for display
                    display_value = value
                    
                    # For analog channels, convert to voltage
                    if channel_id in ANALOG_CHANNELS:
                        display_value = self.convert_to_voltage(value)
                    
                    # Store the data
                    self.channel_data[channel_id]['timestamps'].append(relative_time)
                    self.channel_data[channel_id]['values'].append(display_value) 
                    self.channel_data[channel_id]['cumulative_values'].append(self.channel_data[channel_id]['total_count'])
                    self.channel_data[channel_id]['count'] += 1
                    self.channel_data[channel_id]['last_update'] = relative_time
                    self.channel_data[channel_id]['last_value'] = value  # Store raw value for delta calculations
                    
                    # Add to channel set
                    self.active_channels.add(channel_id)
                    self.total_samples += 1
                
            if self.verbose and self.total_packets % 100 == 0:
                print(f"Processed {self.total_packets} packets, {self.total_samples} samples total")
                
        except Exception as e:
            print(f"Error processing datagram: {e}")
            if self.verbose:
                import traceback
                traceback.print_exc()

    def calculate_data_rates(self, current_time):
        """Calculate data rates based on recent packets."""
        with self.data_lock:
            # Only calculate if we have multiple packets
            if len(self.packet_times) < 2:
                return
            
            # Calculate time window for rate calculation
            time_window = current_time - self.packet_times[0]
            if time_window <= 0:
                return
                
            # Calculate rates
            packets_per_second = len(self.packet_times) / time_window
            total_samples = sum(self.sample_counts)
            samples_per_second = total_samples / time_window
            total_bytes = sum(self.byte_counts)
            bytes_per_second = total_bytes / time_window
            
            # Store for plotting
            rel_time = current_time - self.start_time
            self.speed_data['timestamps'].append(rel_time)
            self.speed_data['packets_per_second'].append(packets_per_second)
            self.speed_data['samples_per_second'].append(samples_per_second)
            self.speed_data['bytes_per_second'].append(bytes_per_second / 1024)  # KB/s
            
            # Update channel-specific sample rates
            for channel_id in self.active_channels:
                channel = self.channel_data[channel_id]
                # Only calculate if we have data for this channel
                if channel['count'] > 0:
                    # Calculate samples per second for this channel in recent window
                    channel_samples = sum(1 for t in channel['timestamps'] 
                                         if t > (rel_time - time_window))
                    channel_rate = channel_samples / time_window
                    channel['samples_per_second'].append(channel_rate)

    def update_plot(self, frame):
        """Update function for matplotlib animation."""
        with self.data_lock:
            # Get all possible colors for channels
            colors = plt.cm.tab10.colors + plt.cm.Dark2.colors
            
            # Get all active analog channels (Channel 1-10)
            active_analog_channels = [ch for ch in self.active_channels if ch in ANALOG_CHANNELS]
            
            # Get all active digital channels (Digital Input 0-5)
            active_digital_channels = [ch for ch in self.active_channels if ch in DIGITAL_CHANNELS]
            
                            # Update the analog channels plot with voltage values
            for idx, channel_id in enumerate(sorted(active_analog_channels)):
                channel = self.channel_data[channel_id]
                color = colors[idx % len(colors)]
                
                # If this is a new channel, create a line for it
                if channel_id not in self.analog_lines:
                    name = CHANNEL_NAMES.get(channel_id, f"Channel {channel_id}")
                    line, = self.ax_analog.plot([], [], lw=1.5, color=color, label=name)
                    self.analog_lines[channel_id] = line
                
                # Update the line data
                if len(channel['timestamps']) > 0:
                    timestamps = list(channel['timestamps'])
                    values = list(channel['values'])
                    
                    self.analog_lines[channel_id].set_data(timestamps, values)
                    
                    # Update legend with current voltage
                    name = CHANNEL_NAMES.get(channel_id, f"Channel {channel_id}")
                    current_value = values[-1] if values else 0
                    self.analog_lines[channel_id].set_label(f"{name.split(' - ')[0]} ({current_value:.2f}V)")
            
                            # Update the digital channels plot with cumulative counts
            for idx, channel_id in enumerate(sorted(active_digital_channels)):
                channel = self.channel_data[channel_id]
                color = colors[idx % len(colors)]
                
                # If this is a new channel, create a line for it
                if channel_id not in self.digital_lines:
                    name = CHANNEL_NAMES.get(channel_id, f"Channel {channel_id}")
                    line, = self.ax_digital.plot([], [], lw=1.5, color=color, label=name)
                    self.digital_lines[channel_id] = line
                
                # Update the line data with cumulative values
                if len(channel['timestamps']) > 0:
                    timestamps = list(channel['timestamps'])
                    cumulative_values = list(channel['cumulative_values'])
                    
                    self.digital_lines[channel_id].set_data(timestamps, cumulative_values)
                    
                    # Update legend with total count and velocity
                    name = CHANNEL_NAMES.get(channel_id, f"Channel {channel_id}")
                    total_count = channel['total_count']
                    velocity = channel['velocity'][-1] if channel['velocity'] else 0
                    self.digital_lines[channel_id].set_label(f"{name.split(' - ')[0]} (Total: {total_count}, Vel: {velocity:.1f})")
            
            # Get combined time range for all data
            all_times = []
            for channel_data in [self.analog_lines, self.digital_lines]:
                for channel_id in channel_data:
                    channel = self.channel_data[channel_id]
                    if len(channel['timestamps']) > 0:
                        all_times.extend(channel['timestamps'])
            
            if all_times:
                time_min = max(0, min(all_times))
                time_max = max(all_times)
                time_range = time_max - time_min
                
                # Add 5% padding to right edge
                time_max += time_range * 0.05
                
                # Update time range for all plots
                self.ax_analog.set_xlim(time_min, time_max)
                self.ax_digital.set_xlim(time_min, time_max)
                
                # Find appropriate y limits for analog plot
                all_analog_values = []
                for channel_id in self.analog_lines:
                    channel = self.channel_data[channel_id]
                    if len(channel['values']) > 0:
                        # Only include values in the visible time window
                        values = [v for t, v in zip(channel['timestamps'], channel['values']) 
                                 if t >= time_min and t <= time_max]
                        if values:
                            all_analog_values.extend(values)
                
                                    # Set appropriate y limits for analog plot (voltage range)
                if all_analog_values:
                    # For voltage, set dynamic range based on actual values with padding
                    if len(all_analog_values) > 0:
                        value_min = min(all_analog_values)
                        value_max = max(all_analog_values)
                        value_range = value_max - value_min
                        
                        # Add padding (at least 10% padding)
                        padding = max(0.1, value_range * 0.2)
                        self.ax_analog.set_ylim(max(0, value_min - padding), min(5.1, value_max + padding))
                
                # Find appropriate y limits for digital plot (using cumulative values)
                all_digital_values = []
                for channel_id in self.digital_lines:
                    channel = self.channel_data[channel_id]
                    if len(channel['cumulative_values']) > 0:
                        # Only include values in the visible time window
                        values = [v for t, v in zip(channel['timestamps'], channel['cumulative_values']) 
                                 if t >= time_min and t <= time_max]
                        if values:
                            all_digital_values.extend(values)
                
                if all_digital_values:
                    value_min = min(all_digital_values)
                    value_max = max(all_digital_values)
                    value_range = value_max - value_min
                    
                    # Add 5% padding
                    padding = max(1, value_range * 0.05)
                    self.ax_digital.set_ylim(max(0, value_min - padding), value_max + padding)
            
            # Update legends
            self.ax_analog.legend(loc='upper left')
            self.ax_digital.legend(loc='upper left')
            
            # Update speed plot
            if len(self.speed_data['timestamps']) > 0:
                # Update the packet rate line
                self.line_packets.set_data(
                    self.speed_data['timestamps'],
                    self.speed_data['packets_per_second']
                )
                
                # Update the sample rate line
                self.line_samples.set_data(
                    self.speed_data['timestamps'],
                    self.speed_data['samples_per_second']
                )
                
                # Update the byte rate line
                self.line_bytes.set_data(
                    self.speed_data['timestamps'],
                    self.speed_data['bytes_per_second']
                )
                
                # Update speed plot legend with current values
                if len(self.speed_data['packets_per_second']) > 0:
                    current_packets = self.speed_data['packets_per_second'][-1]
                    current_samples = self.speed_data['samples_per_second'][-1]
                    current_bytes = self.speed_data['bytes_per_second'][-1]
                    
                    self.line_packets.set_label(f"Packets/sec: {current_packets:.1f}")
                    self.line_samples.set_label(f"Samples/sec: {current_samples:.1f}")
                    self.line_bytes.set_label(f"KB/sec: {current_bytes:.1f}")
                    
                    self.ax_speed.legend(loc='upper right')
                
                # Set appropriate time range, matching the other plots
                if all_times:
                    self.ax_speed.set_xlim(time_min, time_max)
                
                # Set appropriate y limits for speed plot
                packets_values = list(self.speed_data['packets_per_second'])
                samples_values = list(self.speed_data['samples_per_second'])
                
                if packets_values and samples_values:
                    # Separate scales for packets and samples if they differ significantly
                    packets_max = max(packets_values) if packets_values else 0
                    samples_max = max(samples_values) if samples_values else 0
                    
                    # Find an appropriate scale
                    if samples_max > packets_max * 10:
                        # If samples/sec is much higher than packets/sec, use separate y-axes
                        if not hasattr(self, 'ax_samples'):
                            self.ax_samples = self.ax_speed.twinx()
                            self.ax_samples.set_ylabel('Samples/sec', color='green')
                            self.ax_samples.tick_params(axis='y', labelcolor='green')
                            
                        # Set limits for packets on left y-axis
                        self.ax_speed.set_ylim(0, packets_max * 1.2)
                        
                        # Set limits for samples on right y-axis
                        self.ax_samples.set_ylim(0, samples_max * 1.2)
                    else:
                        # Otherwise use a common scale
                        speed_max = max(packets_max, samples_max) * 1.2
                        self.ax_speed.set_ylim(0, speed_max)
                
                # Add a second y-axis for bytes/second if needed
                if not hasattr(self, 'ax_bytes'):
                    self.ax_bytes = self.ax_speed.twinx()
                    self.ax_bytes.set_ylabel('KB/sec', color='red')
                    self.ax_bytes.tick_params(axis='y', labelcolor='red')
                
                # Set appropriate y limits for bytes/second
                if self.speed_data['bytes_per_second']:
                    bytes_max = max(self.speed_data['bytes_per_second']) * 1.1
                    self.ax_bytes.set_ylim(0, bytes_max)

    def report_stats(self):
        """Periodically report channel statistics."""
        while self.running and self.show_stats:
            time.sleep(5)  # Report stats every 5 seconds
            
            with self.data_lock:
                if self.total_samples == 0:
                    continue
                
                # Calculate overall stats
                elapsed_time = time.time() - self.start_time
                avg_packets_per_sec = self.total_packets / elapsed_time
                avg_samples_per_sec = self.total_samples / elapsed_time
                avg_bytes_per_sec = self.total_bytes / elapsed_time
                
                # Get current rates
                current_packets = self.speed_data['packets_per_second'][-1] if self.speed_data['packets_per_second'] else 0
                current_samples = self.speed_data['samples_per_second'][-1] if self.speed_data['samples_per_second'] else 0
                current_bytes = self.speed_data['bytes_per_second'][-1] if self.speed_data['bytes_per_second'] else 0
                
                print(f"\n--- Data Performance Statistics ---")
                print(f"Total packets: {self.total_packets}, Total samples: {self.total_samples}, Total bytes: {self.total_bytes}")
                print(f"Elapsed time: {elapsed_time:.1f} seconds")
                print(f"Average rates: {avg_packets_per_sec:.1f} packets/sec, {avg_samples_per_sec:.1f} samples/sec, {avg_bytes_per_sec/1024:.1f} KB/sec")
                print(f"Current rates: {current_packets:.1f} packets/sec, {current_samples:.1f} samples/sec, {current_bytes:.1f} KB/sec")
                
                print("\n--- Channel Count Statistics ---")
                
                # Group by channel type
                analog_channels = [ch for ch in sorted(self.active_channels) if ch in ANALOG_CHANNELS]
                digital_channels = [ch for ch in sorted(self.active_channels) if ch in DIGITAL_CHANNELS]
                
                for channel_id in sorted(self.active_channels):
                    count = self.channel_data[channel_id]['count']
                    percentage = (count / self.total_samples) * 100.0
                    
                    name = CHANNEL_NAMES.get(channel_id, "Unknown")
                    print(f"  Channel {channel_id:2d} ({name}): {count} samples ({percentage:.2f}%)")
                
                # Print the current counts for all channels
                print("\nCurrent counts (24-bit values):")
                
                # First analog channels
                print("\nAnalog Channels:")
                for channel_id in analog_channels:
                    channel = self.channel_data[channel_id]
                    if len(channel['values']) > 0:
                        raw_value = channel['last_value']  # This is the raw uint24 value
                        voltage = channel['values'][-1]    # This is the scaled voltage
                        name = CHANNEL_NAMES.get(channel_id, "Unknown")
                        print(f"  Channel {channel_id:2d} ({name}): Raw = {raw_value}, Voltage = {voltage:.3f}V")
                
                # Then digital channels
                print("\nDigital Channels:")
                for channel_id in digital_channels:
                    channel = self.channel_data[channel_id]
                    if len(channel['values']) > 0:
                        latest_value = channel['values'][-1]
                        name = CHANNEL_NAMES.get(channel_id, "Unknown")
                        
                        # For digital channels, show total count and velocity
                        if len(channel['velocity']) > 0:
                            latest_velocity = channel['velocity'][-1]
                            total_count = channel['total_count']
                            print(f"  Channel {channel_id:2d} ({name}): Current = {latest_value}, Total = {total_count}, Velocity = {latest_velocity:.2f} counts/sample")
                        else:
                            total_count = channel['total_count']
                            print(f"  Channel {channel_id:2d} ({name}): Current = {latest_value}, Total = {total_count}")
                
                print("\n--- Sample Rate by Channel ---")
                # Calculate and display recent sample rates by channel
                for channel_id in sorted(self.active_channels):
                    channel = self.channel_data[channel_id]
                    if len(channel['samples_per_second']) > 0:
                        rate = channel['samples_per_second'][-1]
                        name = CHANNEL_NAMES.get(channel_id, "Unknown")
                        print(f"  Channel {channel_id:2d} ({name}): {rate:.2f} samples/sec")
                
                # Show timestamp of this report
                now = datetime.now().strftime("%H:%M:%S")
                print(f"\nReport time: {now}")
                print("------------------------\n")

    def run(self):
        """Main method to start the UDP receiver and plotting."""
        # Start the UDP receiver thread
        udp_thread = threading.Thread(target=self.receive_udp_thread)
        udp_thread.daemon = True
        udp_thread.start()
        
        # Start the stats reporting thread if enabled
        if self.show_stats:
            stats_thread = threading.Thread(target=self.report_stats)
            stats_thread.daemon = True
            stats_thread.start()
        
        try:
            # Show the plot (this blocks until window is closed)
            plt.show()
        except KeyboardInterrupt:
            print("Interrupted by user")
        finally:
            self.running = False
            self.socket.close()
            print("UDP Data Speed Plotter stopped")


def main():
    parser = argparse.ArgumentParser(description='UDP Data Speed Plotter')
    parser.add_argument('--port', type=int, default=DEFAULT_UDP_PORT,
                        help=f'UDP port to listen on (default: {DEFAULT_UDP_PORT})')
    parser.add_argument('--buffer-size', type=int, default=MAX_BUFFER_SIZE,
                        help=f'Maximum UDP packet size in bytes (default: {MAX_BUFFER_SIZE})')
    parser.add_argument('--window-size', type=int, default=PLOT_WINDOW,
                        help=f'Number of data points to display (default: {PLOT_WINDOW})')
    parser.add_argument('--velocity-window', type=int, default=VELOCITY_WINDOW,
                        help=f'Number of samples for trailing average velocity (default: {VELOCITY_WINDOW})')
    parser.add_argument('--no-stats', action='store_false', dest='show_stats',
                        help='Disable periodic channel statistics')
    parser.add_argument('--verbose', action='store_true', 
                        help='Enable verbose logging')
    parser.add_argument('--specific-ip', type=str, default='0.0.0.0',
                        help='Bind to specific IP address instead of all interfaces')
    parser.add_argument('--simulate-digital', action='store_true',
                        help='Simulate digital inputs for testing')
    parser.add_argument('--force-digital', action='store_true',
                        help='Force creation of empty digital channels at startup')
    
    args = parser.parse_args()
    
    # Create and run the plotter
    plotter = UDPDataSpeedPlotter(
        port=args.port,
        buffer_size=args.buffer_size,
        plot_window=args.window_size,
        show_stats=args.show_stats,
        verbose=args.verbose
    )
    
    # Force digital channels to be created even if no data yet
    if args.force_digital:
        print("Forcing digital channels to be created")
        now = time.time()
        rel_time = now - plotter.start_time
        
        with plotter.data_lock:
            for channel_id in DIGITAL_CHANNELS:
                # Initialize with zero value
                plotter.channel_data[channel_id]['timestamps'].append(rel_time)
                plotter.channel_data[channel_id]['values'].append(0)
                plotter.channel_data[channel_id]['cumulative_values'].append(0)
                plotter.channel_data[channel_id]['count'] = 1
                plotter.channel_data[channel_id]['last_update'] = rel_time
                plotter.channel_data[channel_id]['last_value'] = 0
                plotter.channel_data[channel_id]['total_count'] = 0
                plotter.channel_data[channel_id]['delta_values'].append(0)
                plotter.channel_data[channel_id]['velocity'].append(0)
                
                # Add to active channels
                plotter.active_channels.add(channel_id)
    
    # Start simulation thread if requested
    if args.simulate_digital:
        import random
        
        def simulate_digital_thread():
            """Generate synthetic digital input events for testing"""
            print("Starting digital input simulation thread")
            digital_channels = list(range(16, 22))  # DIN0-DIN5
            
            # Counter for total simulated pulses
            pulse_counts = {ch: 0 for ch in digital_channels}
            
            while plotter.running:
                time.sleep(0.5)  # Generate events every 0.5 seconds
                
                # Pick a random digital channel
                channel_id = random.choice(digital_channels)
                
                # Create a synthetic digital input event
                with plotter.data_lock:
                    relative_time = time.time() - plotter.start_time
                    
                    # Toggle value between 0 and 1
                    last_value = plotter.channel_data[channel_id]['last_value'] if plotter.channel_data[channel_id]['count'] > 0 else 0
                    value = 1 - last_value  # Toggle between 0 and 1
                    
                    # Calculate delta
                    delta = value - last_value
                    
                    # Update data
                    plotter.channel_data[channel_id]['delta_values'].append(delta)
                    
                    # Only increment total count on rising edges (0->1)
                    if delta > 0:
                        pulse_counts[channel_id] += 1
                        plotter.channel_data[channel_id]['total_count'] = pulse_counts[channel_id]
                    
                    # Calculate velocity (pulses per second)
                    if len(plotter.channel_data[channel_id]['delta_values']) >= VELOCITY_WINDOW:
                        trailing_avg = sum(list(plotter.channel_data[channel_id]['delta_values'])[-VELOCITY_WINDOW:]) / VELOCITY_WINDOW
                    else:
                        trailing_avg = sum(plotter.channel_data[channel_id]['delta_values']) / len(plotter.channel_data[channel_id]['delta_values'])
                    
                    plotter.channel_data[channel_id]['velocity'].append(trailing_avg)
                    
                    # Store data
                    plotter.channel_data[channel_id]['timestamps'].append(relative_time)
                    plotter.channel_data[channel_id]['values'].append(value)
                    plotter.channel_data[channel_id]['cumulative_values'].append(plotter.channel_data[channel_id]['total_count'])
                    plotter.channel_data[channel_id]['count'] += 1
                    plotter.channel_data[channel_id]['last_update'] = relative_time
                    plotter.channel_data[channel_id]['last_value'] = value
                    
                    # Add to active channels
                    plotter.active_channels.add(channel_id)
                
                print(f"Simulated digital input: Channel {channel_id}, Value {value}, Total {plotter.channel_data[channel_id]['total_count']}")
        
        # Start simulation thread
        sim_thread = threading.Thread(target=simulate_digital_thread)
        sim_thread.daemon = True
        sim_thread.start()
    
    # Run the plotter
    plotter.run()


if __name__ == "__main__":
    main()