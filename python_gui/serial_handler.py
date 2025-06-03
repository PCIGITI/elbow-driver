import serial
import time
import threading
import config # For serial default settings


class SerialHandler:
    def __init__(self, data_callback=None, status_callback=None, error_callback=None):
        self.serial_port = None
        self.is_connected = False
        self.port_name = config.DEFAULT_SERIAL_PORT
        self.baudrate = config.SERIAL_BAUDRATE # etc.

        self.data_callback = data_callback # Function to call when data arrives
        self.status_callback = status_callback # Function to call with connection status updates
        self.error_callback = error_callback # Function to call on serial errors

        self.serial_thread = threading.Thread(target=self._monitor_serial, daemon=True)
        self.serial_thread_stop_event = threading.Event()
        # self.serial_thread.start() # Start in connect method
    
    def set_callbacks(self, data_callback, status_callback, error_callback):
        self.data_callback = data_callback
        self.status_callback = status_callback
        self.error_callback = error_callback

    def connect(self, port_name):
        if self.is_connected:
            return True # Already connected
        try:
            self.port_name = port_name
            self.serial_port = serial.Serial(
                port=self.port_name,
                baudrate=config.SERIAL_BAUDRATE,
                bytesize=config.SERIAL_BYTESIZE,
                parity=config.SERIAL_PARITY,
                stopbits=config.SERIAL_STOPBITS,
                timeout=config.SERIAL_TIMEOUT
            )
            time.sleep(config.SERIAL_CONNECT_DELAY) # Allow Arduino to reset
            self.is_connected = True
            if self.status_callback:
                self.status_callback(f"Connected to {self.port_name}", "green", True)

            if not self.serial_thread.is_alive():
                self.serial_thread_stop_event.clear()
                self.serial_thread = threading.Thread(target=self._monitor_serial, daemon=True)
                self.serial_thread.start()
            return True
        except serial.SerialException as e:
            if self.error_callback:
                self.error_callback(f"Failed to connect to {port_name}: {str(e)}")
            if self.status_callback:
                self.status_callback(f"Error connecting", "red", False)
            self.is_connected = False
            return False

    def disconnect(self):
        if not self.is_connected:
            return
        self.serial_thread_stop_event.set() # Signal thread to stop
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.close()
            except serial.SerialException as e:
                 if self.error_callback: self.error_callback(f"Error closing port: {e}")
        self.is_connected = False
        if self.serial_thread.is_alive():
            self.serial_thread.join(timeout=1) # Wait for thread to finish
        if self.status_callback:
            self.status_callback("Disconnected", "black", False)


    def send_command(self, command):
        if self.is_connected and self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(f"{command}\r\n".encode('ascii'))
                self.serial_port.flush()
                # Optionally log sent command via a callback if GUI needs to show it directly
                # if self.data_callback: self.data_callback(f"Sent: {command}", "sent")
                return True
            except serial.SerialException as e:
                if self.error_callback: self.error_callback(f"Error sending: {e}. Disconnecting.")
                self.disconnect() # Auto-disconnect on send error
                return False
            except Exception as e:
                if self.error_callback: self.error_callback(f"Unexpected error sending: {e}")
                self.disconnect()
                return False
        else:
            if self.error_callback: self.error_callback("Not connected. Cannot send command.")
            return False

    def _monitor_serial(self):
        while not self.serial_thread_stop_event.is_set():
            if self.is_connected and self.serial_port and self.serial_port.is_open:
                try:
                    if self.serial_port.in_waiting > 0:
                        response = self.serial_port.readline().decode('ascii', 'ignore').strip()
                        if response and self.data_callback:
                            self.data_callback(response, "received") # Pass type of data
                except serial.SerialException:
                    if self.is_connected: # Only if we thought we were connected
                        if self.error_callback: self.error_callback("Lost connection during monitoring.")
                        self.disconnect() # This will also update status via its callback
                    break # Exit monitoring loop
                except Exception as e:
                    if self.error_callback: self.error_callback(f"Serial monitoring error: {e}")
                    # Potentially disconnect here too if error is severe
            time.sleep(0.1) # Reduce CPU usage

    def cleanup(self):
        self.disconnect()

