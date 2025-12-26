# main.py
from kivy.app import App
from kivy.uix.boxlayout import BoxLayout
from kivy.properties import StringProperty, BooleanProperty
from kivy.clock import Clock
from threading import Thread
from time import sleep
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
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.serial_connection = None
        self.running = False
        self.read_thread = None
        self.log_file = None
        self.log_filename = None
        
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
                9600,  # Baud rate - adjust as needed
                8,     # Data bits
                'N',   # Parity (N=None, E=Even, O=Odd)
                1,     # Stop bits
                timeout=1
            )
            
            if self.serial_connection:
                self.add_terminal_text(f"Connected to {device_name}\n")
                self.add_terminal_text("Baud: 9600, 8N1\n")
                self.add_terminal_text("-" * 50 + "\n")
                
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
                            # Schedule UI update on main thread
                            Clock.schedule_once(
                                lambda dt, t=text: self.add_terminal_text(t), 0
                            )
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
    
    def disconnect_serial(self):
        """Disconnect from serial port"""
        self.running = False
        
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
        
        # Limit buffer size to prevent memory issues
        max_chars = 20000
        if len(self.terminal_text) > max_chars:
            self.terminal_text = self.terminal_text[-max_chars:]
    
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


class SerialTerminalApp(App):
    def build(self):
        self.terminal = SerialTerminal()
        return self.terminal
    
    def on_stop(self):
        """Clean up when app closes"""
        if self.terminal:
            self.terminal.disconnect_serial()
        return True


if __name__ == '__main__':
    SerialTerminalApp().run()