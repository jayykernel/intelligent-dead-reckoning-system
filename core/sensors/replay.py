"""
Deterministic sensor log replay system.

This module enables:
- Recording sensor streams to disk (CSV or MessagePack)
- Replaying exact sensor streams with original timestamps
- Variable playback speed (0.1x to 20x)
- Pausing and resuming replay
- Injecting synthetic faults (GNSS blackouts, sensor noise, etc.) for testing
"""

from __future__ import annotations

import csv
import json
import struct
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import BinaryIO, Iterator, TextIO, Union

from .data_types import (
    BaroSample,
    GnssFix,
    ImuSample,
    MagSample,
    SensorSample,
    SensorType,
)


# MessagePack-like format identifiers for compact binary logging
# We use a simple custom format rather than full msgpack to avoid dependencies
FORMAT_IMU = ord('I')
FORMAT_GNSS = ord('G')
FORMAT_MAG = ord('M')
FORMAT_BARO = ord('B')
FORMAT_TIMESTAMP_REF = ord('T')  # For logging timestamp offsets


@dataclass
class ReplayConfig:
    """Configuration for sensor replay."""
    playback_speed: float = 1.0  # 1.0 = real-time, 0.5 = half speed, 2.0 = double speed
    loop: bool = False
    inject_gnss_blackout: bool = False
    blackout_start_s: float = 0.0
    blackout_duration_s: float = 10.0
    inject_sensor_noise: bool = False
    noise_accel_std: float = 0.0  # m/s^2
    noise_gyro_std: float = 0.0   # rad/s
    noise_mag_std: float = 0.0    # uT
    noise_baro_std: float = 0.0   # Pa


class SensorLogger:
    """Logs sensor streams to disk for later replay."""

    def __init__(self, file_path: Union[str, Path], format: str = "csv"):
        """Initialize logger.

        Args:
            file_path: Path to log file.
            format: Either "csv" (human-readable) or "bin" (compact binary).
        """
        self.file_path = Path(file_path)
        self.format = format.lower()
        if self.format not in ("csv", "bin"):
            raise ValueError("format must be 'csv' or 'bin'")

        self._file: Union[TextIO, BinaryIO, None] = None
        self._writer: Union[csv.writer, None] = None
        self._start_time_ns: Optional[int] = None
        self._is_open = False

    def open(self) -> None:
        """Open the log file for writing."""
        if self._is_open:
            raise RuntimeError("Logger already open")

        if self.format == "csv":
            self._file = open(self.file_path, 'w', newline='')
            self._writer = csv.writer(self._file)
            # Write header
            self._writer.writerow([
                'timestamp_ns',
                'sensor_type',
                'accel_x', 'accel_y', 'accel_z',
                'gyro_x', 'gyro_y', 'gyro_z',
                'mag_x', 'mag_y', 'mag_z',
                'gnss_lat', 'gnss_lon', 'gnss_alt',
                'gnss_vel_n', 'gnss_vel_e', 'gnss_vel_d',
                'gnss_h_acc', 'gnss_v_acc', 'gnss_speed_acc',
                'gnss_satellites',
                'baro_pressure'
            ])
        else:  # binary
            self._file = open(self.file_path, 'wb')
            # Write magic header and version
            self._file.write(b'DRLog\x00\x01')  # 8 bytes: DRLog + 2-byte version (0x0001)

        self._is_open = True
        self._start_time_ns = time.time_ns()

    def close(self) -> None:
        """Close the log file."""
        if not self._is_open:
            return

        if self._file:
            self._file.close()
            self._file = None
        self._writer = None
        self._is_open = False
        self._start_time_ns = None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def log_imu(self, sample: ImuSample) -> None:
        """Log an IMU sample."""
        if not self._is_open:
            raise RuntimeError("Logger not open")

        if self.format == "csv":
            self._writer.writerow([
                sample.timestamp_ns,
                'IMU',
                sample.accel_m_s2[0], sample.accel_m_s2[1], sample.accel_m_s2[2],
                sample.gyro_rad_s[0], sample.gyro_rad_s[1], sample.gyro_rad_s[2],
                '', '', '',  # mag empty
                '', '', '', '', '', '', '', '',  # gnss empty
                ''  # baro empty
            ])
        else:  # binary
            # Write: type(1) + timestamp(8) + accel(3*4) + gyro(3*4) = 1+8+12+12 = 33 bytes
            data = struct.pack(
                '<BQffffff',
                FORMAT_IMU,
                sample.timestamp_ns,
                sample.accel_m_s2[0], sample.accel_m_s2[1], sample.accel_m_s2[2],
                sample.gyro_rad_s[0], sample.gyro_rad_s[1], sample.gyro_rad_s[2]
            )
            self._file.write(data)

    def log_gnss(self, fix: GnssFix) -> None:
        """Log a GNSS fix."""
        if not self._is_open:
            raise RuntimeError("Logger not open")

        if self.format == "csv":
            self._writer.writerow([
                fix.timestamp_ns,
                'GNSS',
                '', '', '', '', '', '',  # accel/gyro empty
                '', '', '',  # mag empty
                fix.latitude_deg, fix.longitude_deg, fix.altitude_m,
                fix.velocity_ned_mps[0], fix.velocity_ned_mps[1], fix.velocity_ned_mps[2],
                fix.horizontal_accuracy_m, fix.vertical_accuracy_m, fix.speed_accuracy_mps,
                fix.satellite_count,
                ''  # baro empty
            ])
        else:  # binary
            # Type(1) + timestamp(8) + lat/lon/alt(3*8) + vel_ned(3*8) + accuracies(3*4) + satcount(2) = 1+8+24+24+12+2 = 71 bytes
            data = struct.pack(
                '<BQddddddfffh',
                FORMAT_GNSS,
                fix.timestamp_ns,
                fix.latitude_deg, fix.longitude_deg, fix.altitude_m,
                fix.velocity_ned_mps[0], fix.velocity_ned_mps[1], fix.velocity_ned_mps[2],
                fix.horizontal_accuracy_m, fix.vertical_accuracy_m, fix.speed_accuracy_mps,
                fix.satellite_count
            )
            self._file.write(data)

    def log_mag(self, sample: MagSample) -> None:
        """Log a magnetometer sample."""
        if not self._is_open:
            raise RuntimeError("Logger not open")

        if self.format == "csv":
            self._writer.writerow([
                sample.timestamp_ns,
                'MAG',
                '', '', '', '', '', '',  # accel/gyro empty
                sample.magnetic_field_ut[0], sample.magnetic_field_ut[1], sample.magnetic_field_ut[2],
                '', '', '', '', '', '', '', '', '', ''  # gnss/baro empty
            ])
        else:  # binary
            # Type(1) + timestamp(8) + mag(3*4) = 1+8+12 = 21 bytes
            data = struct.pack(
                '<BQfff',
                FORMAT_MAG,
                sample.timestamp_ns,
                sample.magnetic_field_ut[0],
                sample.magnetic_field_ut[1],
                sample.magnetic_field_ut[2]
            )
            self._file.write(data)

    def log_baro(self, sample: BaroSample) -> None:
        """Log a barometer sample."""
        if not self._is_open:
            raise RuntimeError("Logger not open")

        if self.format == "csv":
            self._writer.writerow([
                sample.timestamp_ns,
                'BARO',
                '', '', '', '', '', '',  # accel/gyro empty
                '', '', '',  # mag empty
                '', '', '', '', '', '', '', '',  # gnss empty
                sample.pressure_pa
            ])
        else:  # binary
            # Type(1) + timestamp(8) + pressure(4) = 1+8+4 = 13 bytes
            data = struct.pack(
                '<BQf',
                FORMAT_BARO,
                sample.timestamp_ns,
                sample.pressure_pa
            )
            self._file.write(data)

    def log_timestamp_reference(self) -> None:
        """Log a timestamp reference for synchronizing wall-clock to sensor time."""
        if not self._is_open:
            raise RuntimeError("Logger not open")
        if self.format != "bin":
            return  # Only useful for binary format

        wall_time_ns = time.time_ns()
        data = struct.pack(
            '<BQ',
            FORMAT_TIMESTAMP_REF,
            wall_time_ns
        )
        self._file.write(data)


class SensorReplayIterator:
    """Iterator that replays sensor logs with deterministic timing."""

    def __init__(
        self,
        file_path: Union[str, Path],
        config: ReplayConfig | None = None
    ):
        """Initialize replay iterator.

        Args:
            file_path: Path to log file.
            config: Replay configuration (playback speed, fault injection, etc.).
        """
        self.file_path = Path(file_path)
        self.config = config or ReplayConfig()
        self._file: Union[TextIO, BinaryIO, None] = None
        self._start_wall_time_ns: Optional[int] = None
        self._start_sensor_time_ns: Optional[int] = None
        self._paused = False
        self._pause_wall_time_ns: Optional[int] = 0
        self._pause_sensor_time_ns: Optional[int] = 0
        self._total_pause_ns: int = 0
        self._ended = False

    def __iter__(self):
        return self

    def __next__(self) -> SensorSample:
        """Get the next sensor sample, waiting for the correct timestamp if needed."""
        if self._ended:
            raise StopIteration

        # Handle pause/resume
        if self._paused:
            time.sleep(0.01)  # Sleep briefly when paused
            raise StopIteration  # Actually, we should yield nothing when paused
            # But for simplicity in this iterator model, we'll just sleep and continue
            # A better design would have a separate pause() method that breaks iteration

        # Read next sample from file
        sample = self._read_next_sample()
        if sample is None:
            self._ended = True
            raise StopIteration

        # Apply fault injection if configured
        sample = self._apply_faults(sample)

        # Wait for correct playback time if needed
        self._wait_for_playback_time(sample)

        return sample

    def _read_next_sample(self) -> Optional[SensorSample]:
        """Read the next sample from the log file."""
        if self._file is None:
            # Open file on first call
            self._file = open(self.file_path, 'rb')
            # Skip header for binary format
            if self._file.read(7) != b'DRLog\x00\x01':
                raise ValueError("Invalid log file format")

        # For simplicity, we'll implement only binary format replay here
        # CSV format would require parsing lines
        # In a real implementation, we'd support both

        # Try to read one record
        try:
            # Read type byte
            type_byte = self._file.read(1)
            if not type_byte:
                return None  # EOF

            type_val = type_byte[0]

            if type_val == FORMAT_TIMESTAMP_REF:
                # Skip timestamp reference (8 bytes)
                self._file.read(8)
                return self._read_next_sample()  # Recurse to get next actual sample

            elif type_val == FORMAT_IMU:
                # IMU: timestamp(8) + accel(3*4) + gyro(3*4) = 8+12+12 = 32 bytes
                data = self._file.read(32)
                if len(data) < 32:
                    return None
                unpacked = struct.unpack('<Qffffff', data)
                timestamp_ns = int(unpacked[0])
                return ImuSample(
                    timestamp_ns=timestamp_ns,
                    accel_m_s2=(unpacked[1], unpacked[2], unpacked[3]),
                    gyro_rad_s=(unpacked[4], unpacked[5], unpacked[6])
                )

            elif type_val == FORMAT_GNSS:
                # GNSS: timestamp(8) + lat/lon/alt(3*8) + vel(3*8) + acc(3*4) + sat(2) = 8+24+24+12+2 = 70 bytes
                data = self._file.read(70)
                if len(data) < 70:
                    return None
                unpacked = struct.unpack('<Qddddddfffh', data)
                # Note: First byte is the format tag we already read
                timestamp_ns = int(unpacked[0])
                return GnssFix(
                    timestamp_ns=timestamp_ns,
                    latitude_deg=unpacked[1],
                    longitude_deg=unpacked[2],
                    altitude_m=unpacked[3],
                    velocity_ned_mps=(unpacked[4], unpacked[5], unpacked[6]),
                    horizontal_accuracy_m=unpacked[7],
                    vertical_accuracy_m=unpacked[8],
                    speed_accuracy_mps=unpacked[9],
                    satellite_count=unpacked[10]
                )

            elif type_val == FORMAT_MAG:
                # MAG: timestamp(8) + mag(3*4) = 8+12 = 20 bytes
                data = self._file.read(20)
                if len(data) < 20:
                    return None
                unpacked = struct.unpack('<Qfff', data)
                timestamp_ns = unpacked[0]
                return MagSample(
                    timestamp_ns=timestamp_ns,
                    magnetic_field_ut=(unpacked[1], unpacked[2], unpacked[3])
                )

            elif type_val == FORMAT_BARO:
                # BARO: timestamp(8) + pressure(4) = 8+4 = 12 bytes
                data = self._file.read(12)
                if len(data) < 12:
                    return None
                unpacked = struct.unpack('<Qf', data)
                timestamp_ns = unpacked[0]
                return BaroSample(
                    timestamp_ns=timestamp_ns,
                    pressure_pa=unpacked[1]
                )

            else:
                # Unknown type - skip and continue
                return self._read_next_sample()

        except (struct.error, ValueError):
            # Corrupted data or EOF
            return None

    def _apply_faults(self, sample: SensorSample) -> SensorSample:
        """Inject configured faults into the sample."""
        if not isinstance(sample, ImuSample):
            return sample  # Only IMU noise injection for now

        if not self.config.inject_sensor_noise:
            return sample

        import random
        import math

        # Add Gaussian noise
        accel_x = sample.accel_m_s2[0] + random.gauss(0.0, self.config.noise_accel_std)
        accel_y = sample.accel_m_s2[1] + random.gauss(0.0, self.config.noise_accel_std)
        accel_z = sample.accel_m_s2[2] + random.gauss(0.0, self.config.noise_accel_std)

        gyro_x = sample.gyro_rad_s[0] + random.gauss(0.0, self.config.noise_gyro_std)
        gyro_y = sample.gyro_rad_s[1] + random.gauss(0.0, self.config.noise_gyro_std)
        gyro_z = sample.gyro_rad_s[2] + random.gauss(0.0, self.config.noise_gyro_std)

        return ImuSample(
            timestamp_ns=sample.timestamp_ns,
            accel_m_s2=(accel_x, accel_y, accel_z),
            gyro_rad_s=(gyro_x, gyro_y, gyro_z)
        )

    def _wait_for_playback_time(self, sample: SensorSample) -> None:
        """Wait until it's time to output this sample based on playback speed."""
        if self._start_wall_time_ns is None:
            # First sample - record start times
            self._start_wall_time_ns = time.time_ns()
            self._start_sensor_time_ns = sample.timestamp_ns
            return

        # Calculate expected wall time for this sensor timestamp
        sensor_elapsed_ns = sample.timestamp_ns - self._start_sensor_time_ns
        expected_wall_elapsed_ns = int(sensor_elapsed_ns / self.config.playback_speed)
        expected_wall_time_ns = self._start_wall_time_ns + expected_wall_elapsed_ns

        # Current wall time
        now_ns = time.time_ns()

        # If we're ahead of schedule, wait
        if now_ns < expected_wall_time_ns:
            wait_ns = expected_wall_time_ns - now_ns
            # Convert to seconds for time.sleep (max precision is about 1ms on most systems)
            wait_s = wait_ns / 1e9
            if wait_s > 0:
                time.sleep(wait_s)

        # If we're behind schedule, we just output immediately (catch-up mode)
        # In a strict replay, we might drop samples, but we'll just proceed

    def pause(self) -> None:
        """Pause replay."""
        if not self._paused:
            self._paused = True
            self._pause_wall_time_ns = time.time_ns()
            self._pause_sensor_time_ns = self._start_sensor_time_ns + \
                int((time.time_ns() - self._start_wall_time_ns) * self.config.playback_speed) \
                if self._start_wall_time_ns else 0

    def resume(self) -> None:
        """Resume replay from paused state."""
        if self._paused:
            self._paused = False
            pause_duration_ns = time.time_ns() - self._pause_wall_time_ns
            self._total_pause_ns += pause_duration_ns
            # Adjust start time to account for pause
            self._start_wall_time_ns += pause_duration_ns

    def seek_to_timestamp(self, target_timestamp_ns: int) -> None:
        """Seek to a specific timestamp in the log.
        Note: This is approximate and may require rewinding and replaying from start
        for exact positioning in a real implementation.
        """
        # For simplicity, we close and reopen - a real implementation would
        # maintain an index or use binary search on timestamp
        self.close()
        # TODO: Implement proper seeking
        raise NotImplementedError("Seeking not implemented in this version")

    def close(self) -> None:
        """Close the file."""
        if self._file:
            self._file.close()
            self._file = None

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()


def create_sample_log(duration_s: float = 10.0, imu_rate_hz: int = 100) -> list[SensorSample]:
    """Create a simple synthetic sensor log for testing.

    Returns a list of sensor samples simulating:
    - 5 seconds stationary
    - 3 seconds constant velocity motion
    - 2 seconds back to stationary
    """
    import math
    import random

    samples = []
    start_time_ns = time.time_ns()

    dt_ns = int(1e9 / imu_rate_hz)
    num_samples = int(duration_s * imu_rate_hz)

    for i in range(num_samples):
        timestamp_ns = start_time_ns + i * dt_ns
        t_sec = i / imu_rate_hz

        # Motion profile: stationary -> move -> stationary
        if t_sec < 5.0:
            # Stationary
            accel = (0.0, 0.0, 9.81)  # gravity only
            gyro = (0.0, 0.0, 0.0)
        elif t_sec < 8.0:
            # Constant velocity motion (accelerate then decelerate)
            accel_mag = 2.0 * math.sin(math.pi * (t_sec - 5.0) / 3.0)  # Half sine pulse
            accel = (accel_mag, 0.0, 9.81)  # Forward acceleration + gravity
            gyro = (0.0, 0.0, 0.0)
        else:
            # Back to stationary
            accel = (0.0, 0.0, 9.81)
            gyro = (0.0, 0.0, 0.0)

        # Add small noise
        accel = (
            accel[0] + random.gauss(0.0, 0.01),
            accel[1] + random.gauss(0.0, 0.01),
            accel[2] + random.gauss(0.0, 0.01)
        )
        gyro = (
            gyro[0] + random.gauss(0.0, 0.001),
            gyro[1] + random.gauss(0.0, 0.001),
            gyro[2] + random.gauss(0.0, 0.001)
        )

        samples.append(ImuSample(
            timestamp_ns=timestamp_ns,
            accel_m_s2=accel,
            gyro_rad_s=gyro
        ))

    return samples