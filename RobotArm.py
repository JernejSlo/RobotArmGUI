import serial
import time
import threading

class RobotArm:
    def __init__(self, terminal=None, port="COM6", baudrate=115200):
        self.ser = None
        self.port = port
        self.baudrate = baudrate
        self.terminal = terminal
        self.running = False
        self.ack_event = threading.Event()
        self.last_ack_line = ""
        self._ack_lock = threading.Lock()

        try:
            self.ser = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=1)
            time.sleep(2)
            print(f"[RobotArm] Connected to {self.port}")
        except serial.SerialException as e:
            print(f"[RobotArm] Serial error: {e}")
            return

        self.running = True
        self.read_thread = threading.Thread(target=self.read_loop, daemon=True)
        self.read_thread.start()

    def send_command(self, cmd: str):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write((cmd + "\n").encode())
                print(f"[Sent] {cmd}")
            except Exception as e:
                print(f"[RobotArm] Write error: {e}")

    def read_loop(self):
        while self.running:
            try:
                if self.ser.in_waiting:
                    raw = self.ser.readline()
                    print(f"[Raw Bytes] {raw}")  # Debug print

                    line = raw.decode('utf-8', errors='replace').strip()
                    print(f"[Decoded] {line}")
                    if line:
                        self.handle_receive(line)
            except Exception as e:
                print(f"[RobotArm] Read error: {e}")

    def handle_receive(self, line: str):
        if self.terminal:
            self.terminal.log(line)

        s = line.strip()

        # ACK when command is received/parsed
        if s.startswith("Received:"):
            with self._ack_lock:
                self.last_ack_line = s
            self.ack_event.set()
            return

        # Optional future: DONE completion
        if s.upper() == "DONE":
            with self._ack_lock:
                self.last_ack_line = s
            self.ack_event.set()

    def wait_for_ack(self, timeout_s: float = 2.0) -> bool:
        self.ack_event.clear()
        return self.ack_event.wait(timeout=timeout_s)

    def process_ai_guided_frame(self, frame):
        """
        Placeholder function for AI-guided robot vision.
        Currently just returns the raw frame.
        """
        return frame

"""
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
	Buf[*Len] = 0;  // Null-terminate
	const char* response = RobotControl_HandleCommand((char*)Buf);
	CDC_Transmit_FS((uint8_t*)response, strlen(response));

	USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
	USBD_CDC_ReceivePacket(&hUsbDeviceFS);
	return USBD_OK;

  /* USER CODE END 6 */
}
"""