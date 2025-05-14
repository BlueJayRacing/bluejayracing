#!/usr/bin/env python3
import socket
import struct
import time
import threading
import json
import argparse
from collections import defaultdict, deque
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Configuration constants
DEFAULT_UDP_PORT = 8888
MAX_BUFFER_SIZE = 1500  # bytes
PLOT_WINDOW = 1000  # Number of points to display in the plot window
UPDATE_INTERVAL = 100  # milliseconds between plot updates

# Channel name mappings (simplified from the C++ version)
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

class UDPDataPlotter:
    def __init__(self, port=DEFAULT_UDP_PORT, buffer_size=MAX_BUFFER_SIZE, 
                 plot_window=PLOT_WINDOW, show_stats=True, verbose=False):
        """
        Initialize the UDP data plotter.
        
        Args:
            port: UDP port to listen on
            buffer_size: Maximum UDP packet size
            plot_window: Number of data points to show in the visualization
            show_stats: Whether to periodically show channel statistics
            verbose: Whether to print verbose logging information
        """
        self.port = port
        self.buffer_size = buffer_size
        self.plot_window = plot_window
        self.show_stats = show_stats
        self.verbose = verbose
        
        # Data storage
        self.data_lock = threading.Lock()
        self.channel_data = defaultdict(lambda: {
            'timestamps': deque(maxlen=plot_window),
            'values': deque(maxlen=plot_window),
            'count': 0,
            'last_update': 0
        })
        
        self.total_samples = 0
        self.start_time = time.time()
        self.active_channels = set()
        
        # Initialize UDP socket
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('0.0.0.0', self.port))
        self.socket.settimeout(0.1)  # Small timeout for non-blocking operation
        
        # Setup plots
        self.setup_plots()
        
        # Flag to control the UDP receiving thread
        self.running = True
        
        print(f"UDP Data Plotter started on port {self.port}")
        print(f"Waiting for data...")

    def setup_plots(self):
        """Set up the matplotlib figure and subplots."""
        self.fig, self.axes = plt.subplots(4, 4, figsize=(15, 10), constrained_layout=True)
        self.fig.canvas.manager.set_window_title('UDP Data Visualization')
        self.axes = self.axes.flatten()
        
        # Initialize empty plots
        self.lines = {}
        for i, ax in enumerate(self.axes):
            channel_id = i
            # Empty line
            line, = ax.plot([], [], lw=1.5)
            ax.set_title(f"Channel {channel_id}")
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Value")
            ax.grid(True)
            self.lines[channel_id] = line

        # Create animation
        self.ani = FuncAnimation(
            self.fig, self.update_plot, interval=UPDATE_INTERVAL,
            blit=False, cache_frame_data=False
        )

    def receive_udp_thread(self):
        """Thread function to continuously receive UDP packets."""
        while self.running:
            try:
                data, addr = self.socket.recvfrom(self.buffer_size)
                self.process_datagram(data, addr)
            except socket.timeout:
                continue
            except Exception as e:
                print(f"Error receiving UDP data: {e}")

    def process_datagram(self, data, addr):
        """
        Process a received UDP datagram.
        
        This is a simplified version that assumes data format is:
        - Simple JSON with array of samples
        - Each sample has channel_id, timestamp, and value
        
        In a real application, you'd adapt this to your specific data format.
        """
        try:
            # Assume the UDP packet contains JSON data for simplicity
            # In a real implementation, you would use your specific format/protocol
            json_data = json.loads(data.decode('utf-8'))
            
            samples = json_data.get('samples', [])
            received_time = time.time()
            
            with self.data_lock:
                for sample in samples:
                    channel_id = sample.get('channel_id', 0)
                    timestamp = sample.get('timestamp', 0) / 1000000.0  # Convert to seconds
                    value = sample.get('value', 0.0)
                    
                    # For simplicity, we'll use a relative timestamp (time since start)
                    relative_time = received_time - self.start_time
                    
                    # Store the data
                    self.channel_data[channel_id]['timestamps'].append(relative_time)
                    self.channel_data[channel_id]['values'].append(value)
                    self.channel_data[channel_id]['count'] += 1
                    self.channel_data[channel_id]['last_update'] = relative_time
                    
                    self.active_channels.add(channel_id)
                    self.total_samples += 1
                    
            if self.verbose:
                sender = f"{addr[0]}:{addr[1]}"
                print(f"Received data from {sender}: {len(samples)} samples")
                
        except Exception as e:
            print(f"Error processing datagram: {e}")
            if self.verbose:
                print(f"Raw data: {data}")

    def update_plot(self, frame):
        """Update function for matplotlib animation."""
        with self.data_lock:
            for channel_id, line in self.lines.items():
                channel = self.channel_data[channel_id]
                
                # Skip if no data
                if len(channel['timestamps']) == 0:
                    continue
                
                # Prepare data for plotting
                timestamps = list(channel['timestamps'])
                values = list(channel['values'])
                
                # Update the line
                line.set_data(timestamps, values)
                
                # Update the axis limits if needed
                ax = line.axes
                ax.set_xlim(min(timestamps) if timestamps else 0, 
                           max(timestamps) if timestamps else 1)
                
                ymin = min(values) if values else 0
                ymax = max(values) if values else 1
                padding = (ymax - ymin) * 0.1 if ymax > ymin else 0.1
                ax.set_ylim(ymin - padding, ymax + padding)
                
                # Update title with the channel name if available
                name = CHANNEL_NAMES.get(channel_id, f"Channel {channel_id}")
                ax.set_title(f"{name}\n{len(values)} samples")
        
        return list(self.lines.values())

    def report_stats(self):
        """Periodically report channel statistics."""
        while self.running and self.show_stats:
            time.sleep(10)  # Report stats every 10 seconds
            
            with self.data_lock:
                if self.total_samples == 0:
                    continue
                
                print("\n--- Channel Statistics ---")
                print(f"Total samples received: {self.total_samples}")
                
                # Group by channel type
                adc_count, din_count, misc_count = 0, 0, 0
                
                for channel_id in sorted(self.active_channels):
                    count = self.channel_data[channel_id]['count']
                    percentage = (count / self.total_samples) * 100.0
                    
                    if channel_id < 16:
                        adc_count += count
                    elif channel_id < 22:
                        din_count += count
                    else:
                        misc_count += count
                    
                    name = CHANNEL_NAMES.get(channel_id, "Unknown")
                    print(f"  Channel {channel_id:2d} ({name}): {count} samples ({percentage:.2f}%)")
                
                print("\nSummary by type:")
                adc_percent = (adc_count / self.total_samples) * 100.0
                din_percent = (din_count / self.total_samples) * 100.0
                misc_percent = (misc_count / self.total_samples) * 100.0
                
                print(f"  ADC channels: {adc_count} samples ({adc_percent:.2f}%)")
                print(f"  Digital channels: {din_count} samples ({din_percent:.2f}%)")
                print(f"  Misc channels: {misc_count} samples ({misc_percent:.2f}%)")
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
            print("UDP Data Plotter stopped")

    def generate_test_data(self, interval=0.1):
        """
        Generate test data for demonstration/testing when no real UDP data is available.
        
        Args:
            interval: Time interval between generated samples (seconds)
        """
        print("Generating test data...")
        
        # Test data thread
        def test_data_thread():
            t = 0
            while self.running:
                # Generate data for 4 channels (0-3)
                samples = []
                for channel_id in range(4):
                    # Create different waveforms for different channels
                    if channel_id == 0:
                        # Sine wave
                        value = np.sin(t * 2)
                    elif channel_id == 1:
                        # Square wave
                        value = 1.0 if (t * 2) % (2 * np.pi) < np.pi else -1.0
                    elif channel_id == 2:
                        # Triangle wave
                        value = 2 * abs((t * 2) % (2 * np.pi) / np.pi - 1) - 1
                    else:
                        # Sawtooth wave
                        value = ((t * 2) % (2 * np.pi)) / np.pi - 1
                    
                    samples.append({
                        'channel_id': channel_id,
                        'timestamp': int(t * 1000000),  # Microseconds
                        'value': value
                    })
                
                # Create a fake datagram
                datagram = json.dumps({'samples': samples}).encode('utf-8')
                self.process_datagram(datagram, ('127.0.0.1', 12345))
                
                t += interval
                time.sleep(interval)
        
        # Start the test data thread
        test_thread = threading.Thread(target=test_data_thread)
        test_thread.daemon = True
        test_thread.start()


def main():
    parser = argparse.ArgumentParser(description='UDP Data Plotter')
    parser.add_argument('--port', type=int, default=DEFAULT_UDP_PORT,
                        help=f'UDP port to listen on (default: {DEFAULT_UDP_PORT})')
    parser.add_argument('--buffer-size', type=int, default=MAX_BUFFER_SIZE,
                        help=f'Maximum UDP packet size in bytes (default: {MAX_BUFFER_SIZE})')
    parser.add_argument('--window-size', type=int, default=PLOT_WINDOW,
                        help=f'Number of data points to display (default: {PLOT_WINDOW})')
    parser.add_argument('--no-stats', action='store_false', dest='show_stats',
                        help='Disable periodic channel statistics')
    parser.add_argument('--verbose', action='store_true',
                        help='Enable verbose logging')
    parser.add_argument('--test-mode', action='store_true',
                        help='Generate test data instead of waiting for UDP packets')
    
    args = parser.parse_args()
    
    # Create and run the plotter
    plotter = UDPDataPlotter(
        port=args.port,
        buffer_size=args.buffer_size,
        plot_window=args.window_size,
        show_stats=args.show_stats,
        verbose=args.verbose
    )
    
    # Generate test data if requested
    if args.test_mode:
        plotter.generate_test_data()
    
    # Run the plotter
    plotter.run()


if __name__ == "__main__":x
    main()