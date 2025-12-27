# main.py
from kivy.app import App
from kivy.uix.boxlayout import BoxLayout
from kivy.properties import StringProperty, BooleanProperty, NumericProperty, ListProperty
from kivy.clock import Clock
from kivy.core.clipboard import Clipboard
from threading import Thread
from time import sleep, time
import os
from datetime import datetime

# Try to import Android-specific modules
try:
    from usb4a import usb
    from usbserial4a import serial4a
    ANDROID = True
except ImportError:
    ANDROID = False


class SerialTerminal(BoxLayout):
    terminal_text = StringProperty("Serial Terminal Ready\n")
    is_logging = BooleanProperty(False)
    
    # Data properties
    altitude = NumericProperty(0.0)
    vertical_velocity = NumericProperty(0.0)
    uptime = NumericProperty(0.0)
    ground_distance = NumericProperty(0.0)
    range1 = NumericProperty(0.0)
    range2 = NumericProperty(0.0)
    range3 = NumericProperty(0.0)
    battery_voltage = NumericProperty(0.0)
    rssi = NumericProperty(0.0)
    snr = NumericProperty(0.0)
    latitude = NumericProperty(0.0)
    longitude = NumericProperty(0.0)
    
    # Connection and data age properties
    is_connected = BooleanProperty(False)
    data_age = NumericProperty(0.0)
    data_age_color = ListProperty([1, 0, 0, 1])  # Red by default
    data_age_text = StringProperty("DISCONNECTED")
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.serial_connection = None
        self.running = False
        self.read_thread = None
        self.log_file = None
        self.log_filename = None
        self.data_buffer = ""
        self.last_packet_time = None
        
        # Schedule data age update every 0.1 seconds
        Clock.schedule_interval(self.update_data_age, 0.1)
    
    def update_data_age(self, dt):
        """Update data age counter and color"""
        if not self.is_connected:
            self.data_age = 0.0
            self.data_age_color = [1, 0, 0, 1]  # Red
            self.data_age_text = "DISCONNECTED"
        elif self.last_packet_time is None:
            self.data_age = 0.0
            self.data_age_color = [1, 0, 0, 1]  # Red
            self.data_age_text = "NO DATA"
        else:
            self.data_age = time() - self.last_packet_time
            
            # If not logging, always red
            if not self.is_logging:
                self.data_age_color = [1, 0, 0, 1]  # Red
            # Update color based on age when logging
            elif self.data_age < 3.0:
                self.data_age_color = [0, 0.8, 0, 1]  # Green
            elif self.data_age < 5.0:
                self.data_age_color = [1, 1, 0, 1]  # Yellow
            else:
                self.data_age_color = [1, 0, 0, 1]  # Red
            
            self.data_age_text = f"{self.data_age:.1f}s"
        
    def connect_serial(self):
        """Connect to USB serial device"""
        if not ANDROID:
            self.add_terminal_text("Error: Not running on Android platform\n")
            return
            
        try:
            # Get list of USB devices
            device_list = usb.get_usb_device_list()
            
            if not device_list:
                self.add_terminal_text("No USB devices found. Please connect a device.\n")
                return
            
            # Use the first device found
            device = device_list[0]
            device_name = device.getDeviceName()
            
            self.add_terminal_text(f"Found device: {device_name}\n")
            
            # Check and request permission if needed
            if not usb.has_usb_permission(device):
                self.add_terminal_text("Requesting USB permission...\n")
                usb.request_usb_permission(device)
                # Wait a bit for permission dialog
                Clock.schedule_once(lambda dt: self.try_connect(device), 2)
                return
            
            self.try_connect(device)
            
        except Exception as e:
            self.add_terminal_text(f"Connection error: {str(e)}\n")
    
    def try_connect(self, device):
        """Attempt to connect to the device"""
        try:
            device_name = device.getDeviceName()
            
            # Check permission again
            if not usb.has_usb_permission(device):
                self.add_terminal_text("USB permission denied\n")
                return
            
            # Open serial connection
            self.serial_connection = serial4a.get_serial_port(
                device_name,
                115200,  # Baud rate
                8,     # Data bits
                'N',   # Parity (N=None, E=Even, O=Odd)
                1,     # Stop bits
                timeout=1
            )
            
            if self.serial_connection:
                self.add_terminal_text(f"Connected to {device_name}\n")
                self.add_terminal_text("Baud: 115200, 8N1\n")
                self.add_terminal_text("-" * 50 + "\n")
                
                # Set connected status
                self.is_connected = True
                self.last_packet_time = time()
                
                # Start reading in background thread
                self.running = True
                self.read_thread = Thread(target=self.read_serial_data)
                self.read_thread.daemon = True
                self.read_thread.start()
            else:
                self.add_terminal_text("Failed to open serial port\n")
                
        except Exception as e:
            self.add_terminal_text(f"Connection error: {str(e)}\n")
    
    def read_serial_data(self):
        """Background thread to read serial data"""
        while self.running and self.serial_connection:
            try:
                # Check if data is available
                if self.serial_connection.in_waiting > 0:
                    # Read available data
                    bytes_to_read = min(self.serial_connection.in_waiting, 1024)
                    data = self.serial_connection.read(bytes_to_read)
                    
                    if data:
                        # Decode bytes to string
                        try:
                            text = data.decode('utf-8', errors='replace')
                            
                            # Log ALL incoming text immediately (before parsing)
                            Clock.schedule_once(
                                lambda dt, t=text: self.add_terminal_text(t), 0
                            )
                            
                            # Add to buffer and process complete lines
                            self.data_buffer += text
                            
                            # Process complete lines (ending with newline)
                            while '\n' in self.data_buffer:
                                line, self.data_buffer = self.data_buffer.split('\n', 1)
                                line = line.strip()
                                
                                if line:
                                    # Try to parse as CSV data (but don't log again)
                                    if self.parse_data_line(line):
                                        # Update last packet time on successful parse
                                        self.last_packet_time = time()
                                    
                        except Exception as e:
                            Clock.schedule_once(
                                lambda dt: self.add_terminal_text(f"[Decode error]\n"), 0
                            )
                else:
                    # Small sleep to prevent busy waiting
                    sleep(0.01)
                    
            except Exception as e:
                Clock.schedule_once(
                    lambda dt: self.add_terminal_text(f"\nRead error: {str(e)}\n"), 0
                )
                break
    
    def parse_data_line(self, line):
        """Parse CSV line: uptime,altitude,vertical_velocity,ground_distance,range1,range2,range3,battery_voltage,rssi,snr,latitude,longitude"""
        try:
            # Split by comma
            parts = line.split(',')
            
            # Check if we have 12 values
            if len(parts) == 12:
                # Parse each value
                uptime = float(parts[0].strip())
                altitude = float(parts[1].strip())
                vertical_velocity = float(parts[2].strip())
                ground_distance = float(parts[3].strip())
                range1 = float(parts[4].strip())
                range2 = float(parts[5].strip())
                range3 = float(parts[6].strip())
                battery_voltage = float(parts[7].strip())
                rssi = float(parts[8].strip())
                snr = float(parts[9].strip())
                latitude = float(parts[10].strip())
                longitude = float(parts[11].strip())
                
                # Update properties on main thread
                Clock.schedule_once(
                    lambda dt: self.update_data_display(
                        uptime, altitude, vertical_velocity, 
                        ground_distance, range1, range2, range3, 
                        battery_voltage, rssi, snr, latitude, longitude
                    ), 0
                )
                return True
        except (ValueError, IndexError) as e:
            # Not valid CSV data, ignore
            pass
        return False
    
    def update_data_display(self, uptime, altitude, vertical_velocity, 
                           ground_distance, range1, range2, range3, 
                           battery_voltage, rssi, snr, latitude, longitude):
        """Update the data display properties"""
        self.uptime = uptime
        self.altitude = altitude
        self.vertical_velocity = vertical_velocity
        self.ground_distance = ground_distance
        self.range1 = range1
        self.range2 = range2
        self.range3 = range3
        self.battery_voltage = battery_voltage
        self.rssi = rssi
        self.snr = snr
        self.latitude = latitude
        self.longitude = longitude
    
    def disconnect_serial(self):
        """Disconnect from serial port"""
        self.running = False
        self.is_connected = False
        
        if self.read_thread:
            self.read_thread.join(timeout=2)
        
        if self.serial_connection:
            try:
                self.serial_connection.close()
                self.add_terminal_text("\n--- Disconnected ---\n")
            except Exception as e:
                self.add_terminal_text(f"Disconnect error: {str(e)}\n")
            finally:
                self.serial_connection = None
    
    def clear_terminal(self):
        """Clear the terminal display"""
        self.terminal_text = ""
    
    def add_terminal_text(self, text):
        """Add text to terminal (thread-safe)"""
        self.terminal_text += text
        
        # Write to log file if logging is enabled
        if self.is_logging and self.log_file:
            try:
                self.log_file.write(text)
                self.log_file.flush()  # Ensure data is written immediately
            except Exception as e:
                pass  # Silent fail to avoid interrupting serial read
    
    def start_logging(self):
        """Start logging serial data to file"""
        if self.is_logging:
            self.add_terminal_text("Already logging!\n")
            return
        
        try:
            # Get Downloads directory
            if ANDROID:
                # On Android, use the public Downloads directory
                from android.storage import primary_external_storage_path
                storage_path = primary_external_storage_path()
                logs_dir = os.path.join(storage_path, "Download")
            else:
                # Desktop fallback - use Downloads folder
                home = os.path.expanduser("~")
                logs_dir = os.path.join(home, "Downloads")
            
            # Ensure directory exists
            os.makedirs(logs_dir, exist_ok=True)
            
            # Create filename with timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.log_filename = os.path.join(logs_dir, f"serial_log_{timestamp}.txt")
            
            # Open file for writing
            self.log_file = open(self.log_filename, 'w', buffering=1)
            
            # Write CSV header
            self.log_file.write("uptime,altitude,vertical_velocity,ground_distance,range1,range2,range3,battery_voltage,rssi,snr,latitude,longitude\n")
            self.log_file.flush()
            
            self.is_logging = True
            
            self.add_terminal_text(f"\n--- Logging started ---\n")
            self.add_terminal_text(f"File: {self.log_filename}\n\n")
            
        except Exception as e:
            self.add_terminal_text(f"Failed to start logging: {str(e)}\n")
    
    def stop_logging(self):
        """Stop logging serial data"""
        if not self.is_logging:
            self.add_terminal_text("Not currently logging!\n")
            return
        
        try:
            if self.log_file:
                self.log_file.close()
                self.log_file = None
            
            self.is_logging = False
            self.add_terminal_text(f"\n--- Logging stopped ---\n")
            self.add_terminal_text(f"Saved to: {self.log_filename}\n\n")
            
        except Exception as e:
            self.add_terminal_text(f"Error stopping logging: {str(e)}\n")
    
    def copy_gps_to_clipboard(self):
        """Copy GPS coordinates to clipboard"""
        gps_string = f"{self.latitude:.6f}, {self.longitude:.6f}"
        Clipboard.copy(gps_string)
        self.add_terminal_text(f"Copied to clipboard: {gps_string}\n")
    
    def on_stop(self):
        """Called when app stops"""
        # Stop logging if active
        if self.is_logging:
            self.stop_logging()
        self.disconnect_serial()


class SerialTerminalApp(App):
    def build(self):
        self.terminal = SerialTerminal()
        return self.terminal
    
    def on_stop(self):
        """Clean up when app closes"""
        if self.terminal:
            self.terminal.on_stop()
        return True


if __name__ == '__main__':
    SerialTerminalApp().run()