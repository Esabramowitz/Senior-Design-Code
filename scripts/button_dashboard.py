from serial.tools import list_ports
import tkinter as tk

import serial


def main():
    matching_ports = []

    for device in list_ports.comports():
        if device.vid == 0x2341 and device.pid == 0x0069:
            matching_ports.append(device.device)

    if len(matching_ports) != 1:
        raise SystemExit(f"Expected one Uno R4 Minima. Found: {matching_ports}")

    port = matching_ports[0]

    try:
        arduino = serial.Serial(port, 115200, timeout=0)
    except serial.SerialException as error:
        raise SystemExit(f"Could not open {port}: {error}")

    window = tk.Tk()
    window.title("Arduino Button Monitor")
    window.geometry("460x240")
    window.configure(bg="#111827")

    tk.Label(
        window, text="Arduino Button Monitor", font=("Segoe UI", 17, "bold"),
        bg="#111827", fg="#f8fafc",
    ).pack(pady=(24, 4))

    tk.Label(
        window, text=f"{port}  /  115200 baud", font=("Segoe UI", 10),
        bg="#111827", fg="#94a3b8",
    ).pack()

    indicator = tk.Label(
        window, text="WAITING FOR DATA", font=("Segoe UI", 23, "bold"),
        bg="#1e293b", fg="#cbd5e1", pady=20,
    )
    indicator.pack(fill="x", padx=24, pady=20)

    pending = bytearray()

    def update():
        try:
            pending.extend(arduino.read(arduino.in_waiting))
        except serial.SerialException:
            indicator.config(text="DISCONNECTED", bg="#7f1d1d", fg="white")
            return

        # USB reads can split a line. Keep the unfinished part for next time.
        lines = pending.split(b"\n")
        pending[:] = lines.pop()

        for line in reversed(lines):
            value = line.strip()
            if value in (b"0", b"1"):
                pressed = value == b"1"
                indicator.config(
                    text="PRESSED" if pressed else "RELEASED",
                    bg="#0f766e" if pressed else "#1e293b",
                    fg="#ffffff" if pressed else "#cbd5e1",
                )
                break

        window.after(20, update)

    try:
        update()
        window.mainloop()
    finally:
        arduino.close()


if __name__ == "__main__":
    main()