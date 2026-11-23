import math
import tkinter as tk

import serial
from serial.tools import list_ports

AXES = ((1, 0, 0), (0, 1, 0), (0, 0, 1))
STAGE_MS = 5000
REST_MS = 1000
TEST_STAGES = ("Translate X", "Translate Y", "Translate Z", "Rotate X", "Rotate Y", "Rotate Z")


def quaternion_product(left, right):
    w, x, y, z = left
    a, b, c, d = right
    return (
        w*a - x*b - y*c - z*d,
        w*b + x*a + y*d - z*c,
        w*c - x*d + y*a + z*b,
        w*d + x*c - y*b + z*a,
    )


def rotate_vector(orientation, vector):
    w, x, y, z = orientation
    rotated = quaternion_product(orientation, (0, *vector))
    return quaternion_product(rotated, (w, -x, -y, -z))[1:]


def parse_imu(line):
    fields = line.split(b",")
    if len(fields) != 9 or fields[0] != b"IMU" or fields[1] not in (b"1", b"2"):
        return None

    try:
        timestamp = int(fields[2])
        values = tuple(float(field) for field in fields[3:])
    except ValueError:
        return None

    if 0 <= timestamp <= 0xFFFFFFFF and all(math.isfinite(value) for value in values):
        return int(fields[1]), timestamp, values[:3], values[3:]
    return None


class PoseEstimate:
    def __init__(self):
        self.timestamp = None
        self.ready = False
        self.position = self.origin = self.velocity = (0.0, 0.0, 0.0)
        self.orientation = (1.0, 0.0, 0.0, 0.0)
        self.orientation_origin = self.orientation
        self.angular_rate = (0.0, 0.0, 0.0)
        self.acceleration = (0.0, 0.0, 0.0)

    @property
    def displacement(self):
        return tuple(value - origin for value, origin in zip(self.position, self.origin))

    @property
    def relative_orientation(self):
        w, x, y, z = self.orientation_origin
        return quaternion_product(self.orientation, (w, -x, -y, -z))

    def rezero(self):
        self.origin = self.position
        self.orientation_origin = self.orientation

    def update(self, timestamp, accel, gyro):
        elapsed_ms = 0 if self.timestamp is None else (timestamp - self.timestamp) & 0xFFFFFFFF
        self.timestamp = timestamp
        # Each synthetic stage starts at rest with the sensor axes aligned.
        resting = (
            timestamp % STAGE_MS < REST_MS
            and accel == (0.0, 0.0, 1.0)
            and gyro == (0.0, 0.0, 0.0)
        )
        if elapsed_ms > 250:
            self.ready = False
        if not self.ready and not resting:
            return

        dt = elapsed_ms / 1000.0 if self.ready else 0.0
        self.ready = True
        angular_rate = tuple(math.radians(value) for value in gyro)
        rotation = tuple(
            (previous + current) * dt / 2
            for previous, current in zip(self.angular_rate, angular_rate)
        )
        angle = math.hypot(*rotation)
        if resting:
            self.orientation = (1.0, 0.0, 0.0, 0.0)
        elif angle:
            factor = math.sin(angle / 2) / angle
            delta = (math.cos(angle / 2), *(value * factor for value in rotation))
            orientation = quaternion_product(self.orientation, delta)
            norm = math.hypot(*orientation)
            self.orientation = tuple(value / norm for value in orientation)
        self.angular_rate = angular_rate
        x, y, z = rotate_vector(self.orientation, accel)
        world_accel = tuple(9.80665 * value for value in (x, y, z - 1))
        velocity = tuple(
            speed + (previous + current) * dt / 2
            for speed, previous, current in zip(self.velocity, self.acceleration, world_accel)
        )
        self.position = tuple(
            position + (previous + current) * dt / 2
            for position, previous, current in zip(self.position, self.velocity, velocity)
        )
        self.velocity = (0.0, 0.0, 0.0) if resting else velocity
        self.acceleration = world_accel


class VectorView(tk.Frame):
    arrow_length_m = 0.40

    def __init__(self, parent, title, color):
        super().__init__(parent, bg="#1e293b")
        self.color = color
        self.pose = PoseEstimate()

        tk.Label(
            self, text=title, font=("Segoe UI", 14, "bold"),
            bg="#1e293b", fg="#f8fafc",
        ).pack(pady=(12, 4))
        self.canvas = tk.Canvas(
            self, width=340, height=260, bg="#1e293b", highlightthickness=0,
        )
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Configure>", self.draw)
        self.readout = tk.Label(
            self, text="Waiting for IMU data", font=("Consolas", 10),
            bg="#1e293b", fg=color,
        )
        self.readout.pack(pady=(4, 12))

    def refresh(self):
        if self.pose.ready:
            x, y, z = self.pose.displacement
            w, qx, qy, qz = self.pose.relative_orientation
            roll = math.atan2(2 * (w*qx + qy*qz), 1 - 2 * (qx*qx + qy*qy))
            pitch = math.asin(max(-1, min(1, 2 * (w*qy - qz*qx))))
            yaw = math.atan2(2 * (w*qz + qx*qy), 1 - 2 * (qy*qy + qz*qz))
            self.readout.config(text=(
                f"X {x:+.2f}  Y {y:+.2f}  Z {z:+.2f} m\n"
                f"Roll {math.degrees(roll):+.0f}°  Pitch {math.degrees(pitch):+.0f}°"
                f"  Yaw {math.degrees(yaw):+.0f}°"
            ))
        else:
            self.readout.config(text="Waiting for the sensor to be at rest")
        self.draw()

    def draw(self, event=None):
        canvas = self.canvas
        width, height = canvas.winfo_width(), canvas.winfo_height()
        center_x, center_y = width / 2, height / 2
        extent = max(1.2, math.hypot(*self.pose.displacement) + self.arrow_length_m + 0.10)
        scale = min(width, height) * 0.39 / extent

        def project(x, y, z):
            # Fixed isometric view of the three sensor axes.
            return (
                center_x + scale * (x - y) * math.sqrt(3) / 2,
                center_y + scale * ((x + y) / 2 - z),
            )

        canvas.delete("all")
        for label, unit in zip("XYZ", AXES):
            axis = tuple(value * extent for value in unit)
            canvas.create_line(
                *project(*(-value for value in axis)), center_x, center_y,
                fill="#475569", dash=(3, 4),
            )
            canvas.create_line(
                center_x, center_y, *project(*axis), fill="#94a3b8", arrow=tk.LAST,
            )
            canvas.create_text(
                *project(*(value * 1.13 for value in axis)), text=f"+{label}", fill="#cbd5e1",
            )
            canvas.create_text(
                *project(*(-value * 1.13 for value in axis)), text=f"−{label}", fill="#94a3b8",
            )

        if self.pose.ready:
            x, y, z = self.pose.displacement
            base_x, base_y = project(x, y, z)
            orientation = self.pose.relative_orientation
            # Draw sensor +X (heading) and +Z (up) so roll is visible too.
            for label, axis, length, color in (
                ("x", AXES[0], self.arrow_length_m, self.color),
                ("z", AXES[2], self.arrow_length_m * 0.7, "#e2e8f0"),
            ):
                dx, dy, dz = rotate_vector(orientation, axis)
                tip = project(x + length * dx, y + length * dy, z + length * dz)
                canvas.create_line(base_x, base_y, *tip, fill=color, width=3, arrow=tk.LAST)
                canvas.create_text(*tip, text=label, fill=color, anchor="sw")
            canvas.create_oval(
                base_x - 3, base_y - 3, base_x + 3, base_y + 3, fill=self.color, outline="",
            )


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
    window.title("Arduino Sensor Dashboard")
    window.geometry("800x480")
    window.minsize(740, 440)
    window.configure(bg="#111827")

    tk.Label(
        window, text="Arduino Sensor Dashboard", font=("Segoe UI", 17, "bold"),
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
    indicator.pack(fill="x", padx=24, pady=12)

    panels = tk.Frame(window, bg="#111827")
    panels.pack(fill="both", expand=True, padx=24)
    panels.columnconfigure(0, weight=1, uniform="sensor")
    panels.columnconfigure(1, weight=1, uniform="sensor")
    panels.rowconfigure(0, weight=1)
    views = {
        1: VectorView(panels, "IMU 1", "#5eead4"),
        2: VectorView(panels, "IMU 2", "#fbbf24"),
    }
    views[1].grid(row=0, column=0, sticky="nsew", padx=(0, 6))
    views[2].grid(row=0, column=1, sticky="nsew", padx=(6, 0))

    def rezero():
        for view in views.values():
            if view.pose.ready:
                view.pose.rezero()
                view.refresh()

    reset = tk.Button(window, text="Re-zero pose", command=rezero, state=tk.DISABLED)
    reset.pack(pady=(8, 0))

    imu_status = tk.Label(
        window, text="Waiting for IMU timestamp", font=("Segoe UI", 10),
        bg="#111827", fg="#94a3b8",
    )
    imu_status.pack(pady=8)

    pending = bytearray()

    def update():
        try:
            pending.extend(arduino.read(arduino.in_waiting))
        except serial.SerialException:
            indicator.config(text="DISCONNECTED", bg="#7f1d1d", fg="white")
            imu_status.config(text="IMU stream disconnected")
            reset.config(state=tk.DISABLED)
            return

        # USB reads can split a line. Keep the unfinished part for next time.
        lines = pending.split(b"\n")
        pending[:] = lines.pop()

        button = timestamp = None
        updated = set()
        for line in lines:
            value = line.strip()
            if value in (b"0", b"1"):
                button = value == b"1"
            elif value.startswith(b"IMU,"):
                sample = parse_imu(value)
                if sample is not None:
                    sensor, timestamp, accel, gyro = sample
                    views[sensor].pose.update(timestamp, accel, gyro)
                    updated.add(sensor)

        if button is not None:
            indicator.config(
                text="PRESSED" if button else "RELEASED",
                bg="#0f766e" if button else "#1e293b",
                fg="#ffffff" if button else "#cbd5e1",
            )
        if updated:
            for sensor in updated:
                views[sensor].refresh()
            reset.config(state=tk.NORMAL if all(view.pose.ready for view in views.values()) else tk.DISABLED)
            stage = (timestamp // STAGE_MS) % len(TEST_STAGES)
            imu_status.config(text=f"{TEST_STAGES[stage]}  ·  Arduino uptime {timestamp:,} ms")

        window.after(20, update)

    try:
        update()
        window.mainloop()
    finally:
        arduino.close()


if __name__ == "__main__":
    main()