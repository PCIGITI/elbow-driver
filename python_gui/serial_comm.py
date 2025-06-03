import serial
import threading
import time

class SerialCommunicator:
    def __init__(self, callback=None):
        self.serial_port = None
        self.is_connected = False
        self.callback = callback
        self.monitor_thread = None

    def connect(self, port, baud_rate=9600):
        try:
            self.serial_port = serial.Serial(
                port=port,
                baudrate=baud_rate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1
            )
            time.sleep(2)  # Allow Arduino to reset
            self.is_connected = True
            self.monitor_thread = threading.Thread(target=self.monitor_serial, daemon=True)
            self.monitor_thread.start()
            return True, f"Connected to {port}"
        except serial.SerialException as e:
            return False, f"Connection error: {str(e)}"

    def disconnect(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.is_connected = False

    def send_command(self, command):
        if self.is_connected and self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(f"{command}\r\n".encode('ascii'))
                self.serial_port.flush()
                return True, f"Sent: {command}"
            except serial.SerialException as e:
                self.is_connected = False
                return False, f"Error sending command: {e}. Lost connection."
            except Exception as e:
                self.is_connected = False
                return False, f"Unexpected error sending command: {e}"
        else:
            return False, "Error: Not connected. Cannot send command."

    def monitor_serial(self):
        while True:
            if self.is_connected and self.serial_port and self.serial_port.is_open:
                try:
                    if self.serial_port.in_waiting > 0:
                        response = self.serial_port.readline().decode('ascii', 'ignore').strip()
                        if response and self.callback:
                            # Use callback in main thread if possible
                            self.callback(response)
                except serial.SerialException:
                    if self.is_connected:
                        if self.callback:
                            self.callback("Error: Lost connection during serial monitoring.")
                        self.is_connected = False
                except Exception as e:
                    if self.callback:
                        self.callback(f"Unexpected serial monitoring error: {e}")
            time.sleep(0.1)
