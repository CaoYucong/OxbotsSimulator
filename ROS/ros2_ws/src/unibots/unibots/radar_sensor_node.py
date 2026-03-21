import json
import time
import urllib.request

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class RadarSensorNode(Node):
    def __init__(self) -> None:
        super().__init__('radar_sensor_node')

        self.declare_parameter('use_real_sensor', False)
        self.declare_parameter('remote_host', '192.168.50.2')
        self.declare_parameter('remote_port', 5003)
        self.declare_parameter('poll_hz', 10.0)
        self.declare_parameter('request_timeout', 1.0)
        self.declare_parameter('time_topic', '/time')
        self.declare_parameter('tof_front_addr', 0x30)
        self.declare_parameter('tof_right_addr', 0x32)
        self.declare_parameter('tof_left_addr', 0x31)
        self.declare_parameter('tof_rear_addr', 0x33)
        self.declare_parameter('tof_front_xshut_gpio', 26)
        self.declare_parameter('tof_left_xshut_gpio', 16)
        self.declare_parameter('tof_right_xshut_gpio', 20)
        self.declare_parameter('tof_rear_xshut_gpio', 21)
        self.declare_parameter('tof_boot_delay_ms', 80)
        self.declare_parameter('tof_min_range_m', 0.02)
        self.declare_parameter('tof_max_range_m', 0.80)
        self.declare_parameter('tof_read_fail_value_m', 0.80)
        self.declare_parameter('tof_debug_enabled', True)
        self.declare_parameter('tof_debug_interval_sec', 1.0)
        self.declare_parameter('tof_init_retries', 5)
        self.declare_parameter('tof_reinit_after_fail_count', 8)

        self.use_real_sensor = self.get_parameter('use_real_sensor').get_parameter_value().bool_value
        self.remote_host = self.get_parameter('remote_host').get_parameter_value().string_value
        self.remote_port = int(self.get_parameter('remote_port').get_parameter_value().integer_value)
        poll_hz = float(self.get_parameter('poll_hz').get_parameter_value().double_value)
        self.request_timeout = float(self.get_parameter('request_timeout').get_parameter_value().double_value)
        self.time_topic = self.get_parameter('time_topic').get_parameter_value().string_value
        self.tof_min_range_m = float(self.get_parameter('tof_min_range_m').get_parameter_value().double_value)
        self.tof_max_range_m = float(self.get_parameter('tof_max_range_m').get_parameter_value().double_value)
        self.tof_read_fail_value_m = float(self.get_parameter('tof_read_fail_value_m').get_parameter_value().double_value)
        self.tof_debug_enabled = self.get_parameter('tof_debug_enabled').get_parameter_value().bool_value
        self.tof_debug_interval_sec = float(
            self.get_parameter('tof_debug_interval_sec').get_parameter_value().double_value
        )
        self.tof_boot_delay_ms = int(self.get_parameter('tof_boot_delay_ms').get_parameter_value().integer_value)
        self.tof_init_retries = int(self.get_parameter('tof_init_retries').get_parameter_value().integer_value)
        self.tof_reinit_after_fail_count = int(
            self.get_parameter('tof_reinit_after_fail_count').get_parameter_value().integer_value
        )

        if self.request_timeout <= 0.0:
            self.request_timeout = 1.0
        if self.tof_debug_interval_sec <= 0.0:
            self.tof_debug_interval_sec = 1.0
        if self.tof_boot_delay_ms <= 0:
            self.tof_boot_delay_ms = 80
        if self.tof_init_retries <= 0:
            self.tof_init_retries = 1
        if self.tof_reinit_after_fail_count <= 0:
            self.tof_reinit_after_fail_count = 8

        self._latest_time_s: float | None = None
        self._last_tof_debug = self.get_clock().now()
        self._tof_backend = 'none'
        self._tof_readers = {}
        self._tof_consecutive_fail_count = 0
        self._tof_xshut_pins = {}

        if self.time_topic:
            self.create_subscription(String, self.time_topic, self._on_time, 10)

        self._urls = [
            f'http://{self.remote_host}:{self.remote_port}/data/simulation_data',
            f'http://{self.remote_host}:{self.remote_port}/simulation_data',
        ]
        self._url_index = 0
        self._pub_radar_sensor = self.create_publisher(String, '/radar_sensor', 10)
        self._last_radar_text = None
        self._error_logged = False
        self._tof_error_logged = False

        if self.use_real_sensor:
            self._prepare_tof_addresses()
            self._tof_readers = self._build_tof_readers()
            self._tof_backend = 'vl53l0x' if self._tof_readers else 'none'

        period = 0.1 if poll_hz <= 0.0 else (1.0 / poll_hz)
        self.create_timer(period, self._poll_once)
        self._run_enabled: bool = False
        self.create_subscription(String, '/run', self._on_run, 10)

        if self.use_real_sensor:
            self.get_logger().info(
                'radar_sensor_node started in real-sensor mode; '
                f'backend={self._tof_backend}, poll_hz={poll_hz}, time_topic={self.time_topic}'
            )
        else:
            self.get_logger().info(
                f'radar_sensor_node started in simulation mode; upstream={self._urls}, poll_hz={poll_hz}'
            )

    def _on_time(self, msg: String) -> None:
        raw = (msg.data or '').strip()
        if not raw:
            return
        try:
            self._latest_time_s = float(raw)
        except ValueError:
            self._latest_time_s = None

    def _on_run(self, msg: String) -> None:
        self._run_enabled = (msg.data or '').strip().lower() != 'off'

    def _resolve_time_s(self) -> float:
        if self._latest_time_s is not None:
            return self._latest_time_s
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def _replace_radar_timestamp(self, radar_text: str) -> str:
        parts = [part.strip() for part in radar_text.split(',')]
        if len(parts) < 5:
            return radar_text
        parts[0] = f'{self._resolve_time_s():.6f}'
        return ','.join(parts[:5])

    def _poll_once(self) -> None:
        if not self._run_enabled:
            return
        if self.use_real_sensor:
            radar_text = self._build_radar_text_from_tof()
            if not radar_text:
                return
            if radar_text == self._last_radar_text:
                return
            msg = String()
            msg.data = radar_text
            self._pub_radar_sensor.publish(msg)
            self._last_radar_text = radar_text
            return

        try:
            body = self._fetch_simulation_data_body()
            payload = json.loads(body.decode('utf-8', errors='ignore'))
            if not isinstance(payload, dict):
                raise ValueError('simulation_data payload must be a JSON object')
        except Exception as exc:
            if not self._error_logged:
                self.get_logger().warn(f'upstream fetch failed for {self._urls}: {exc}')
                self._error_logged = True
            return

        self._error_logged = False
        radar_text = str(payload.get('radar_sensor', '')).strip()
        if not radar_text:
            return
        radar_text = self._replace_radar_timestamp(radar_text)
        if radar_text == self._last_radar_text:
            return

        msg = String()
        msg.data = radar_text
        self._pub_radar_sensor.publish(msg)
        self._last_radar_text = radar_text

    def _build_tof_readers(self):
        try:
            import board
            import busio
            import adafruit_vl53l0x
        except Exception as exc:
            self.get_logger().error(f'use_real_sensor=true but ToF dependencies unavailable: {exc}')
            return {}

        addresses = {
            'front': int(self.get_parameter('tof_front_addr').get_parameter_value().integer_value),
            'right': int(self.get_parameter('tof_right_addr').get_parameter_value().integer_value),
            'left': int(self.get_parameter('tof_left_addr').get_parameter_value().integer_value),
            'rear': int(self.get_parameter('tof_rear_addr').get_parameter_value().integer_value),
        }

        i2c = busio.I2C(board.SCL, board.SDA)
        readers = {}
        for name, addr in addresses.items():
            sensor = None
            last_exc = None
            for _ in range(self.tof_init_retries):
                try:
                    sensor = adafruit_vl53l0x.VL53L0X(i2c, address=addr)
                    break
                except Exception as exc:
                    last_exc = exc
                    time.sleep(0.05)

            if sensor is None:
                self.get_logger().warn(f'failed to init ToF {name}@0x{addr:02X}: {last_exc}')
                continue

            def _make_reader(dev):
                return lambda: float(dev.range) / 1000.0

            readers[name] = _make_reader(sensor)
        if not readers:
            self.get_logger().error('no ToF sensors initialized; radar_sensor_node cannot publish real radar data')
        return readers

    def _prepare_tof_addresses(self) -> None:
        try:
            import smbus2
            from gpiozero import OutputDevice
        except Exception as exc:
            self.get_logger().warn(f'cannot allocate ToF addresses at startup (missing deps): {exc}')
            return

        order = [
            ('front', int(self.get_parameter('tof_front_xshut_gpio').get_parameter_value().integer_value), int(self.get_parameter('tof_front_addr').get_parameter_value().integer_value)),
            ('left', int(self.get_parameter('tof_left_xshut_gpio').get_parameter_value().integer_value), int(self.get_parameter('tof_left_addr').get_parameter_value().integer_value)),
            ('right', int(self.get_parameter('tof_right_xshut_gpio').get_parameter_value().integer_value), int(self.get_parameter('tof_right_addr').get_parameter_value().integer_value)),
            ('rear', int(self.get_parameter('tof_rear_xshut_gpio').get_parameter_value().integer_value), int(self.get_parameter('tof_rear_addr').get_parameter_value().integer_value)),
        ]

        pins = {}
        for name, gpio, _ in order:
            try:
                pins[name] = OutputDevice(gpio, active_high=True, initial_value=False)
            except Exception as exc:
                self.get_logger().warn(f'failed to claim ToF XSHUT GPIO{gpio} for {name}: {exc}')
                for pin in pins.values():
                    pin.close()
                return

        time.sleep(0.1)
        bus = None
        try:
            bus = smbus2.SMBus(1)
            for name, _gpio, target in order:
                pins[name].on()
                time.sleep(max(10, self.tof_boot_delay_ms) / 1000.0)
                try:
                    bus.write_byte_data(0x29, 0x8A, target & 0x7F)
                except OSError as exc:
                    self.get_logger().warn(f'address assign failed for {name} -> 0x{target:02X}: {exc}')
                time.sleep(0.02)
            self.get_logger().info('ToF XSHUT/address allocation done at node startup')
        except Exception as exc:
            self.get_logger().warn(f'ToF startup address allocation failed: {exc}')
        finally:
            if bus is not None:
                try:
                    bus.close()
                except Exception:
                    pass

        self._tof_xshut_pins = pins

    def _build_radar_text_from_tof(self) -> str | None:
        if not self._tof_readers:
            if not self._tof_error_logged:
                self.get_logger().error('real-sensor mode enabled but no ToF readers available')
                self._tof_error_logged = True
            return None

        values = {}
        debug_values = {}
        directions = ('front', 'right', 'left', 'rear')
        for name in directions:
            reader = self._tof_readers.get(name)
            if reader is None:
                values[name] = self.tof_read_fail_value_m
                debug_values[name] = (None, 'FAIL', values[name])
                continue

            try:
                raw = float(reader())
                if raw < self.tof_min_range_m:
                    pub = self.tof_min_range_m
                    status = 'CLAMP_MIN'
                elif raw > self.tof_max_range_m:
                    pub = self.tof_max_range_m
                    status = 'CLAMP_MAX'
                else:
                    pub = raw
                    status = 'OK'
                values[name] = pub
                debug_values[name] = (raw, status, pub)
            except Exception:
                values[name] = self.tof_read_fail_value_m
                debug_values[name] = (None, 'FAIL', values[name])

        self._maybe_log_radar_debug(debug_values)

        all_failed = all(debug_values[name][1] == 'FAIL' for name in directions)
        if all_failed:
            self._tof_consecutive_fail_count += 1
        else:
            self._tof_consecutive_fail_count = 0

        if self._tof_consecutive_fail_count >= self.tof_reinit_after_fail_count:
            self.get_logger().warn(
                'all ToF reads failed repeatedly; attempting ToF address re-allocation + reinitialization '
                f'(count={self._tof_consecutive_fail_count})'
            )
            self._prepare_tof_addresses()
            self._tof_readers = self._build_tof_readers()
            self._tof_backend = 'vl53l0x' if self._tof_readers else 'none'
            self._tof_consecutive_fail_count = 0

        timestamp_s = self._resolve_time_s()
        return (
            f'{timestamp_s:.6f},'
            f'{values["front"]:.3f},{values["right"]:.3f},{values["left"]:.3f},{values["rear"]:.3f}'
        )

    def _maybe_log_radar_debug(self, debug_values: dict[str, tuple[float | None, str, float]]) -> None:
        if not self.tof_debug_enabled:
            return
        now = self.get_clock().now()
        delta = (now - self._last_tof_debug).nanoseconds / 1_000_000_000.0
        if delta < self.tof_debug_interval_sec:
            return
        self._last_tof_debug = now

        summary = []
        for name in ('front', 'right', 'left', 'rear'):
            raw, status, pub = debug_values[name]
            raw_mm = 'None' if raw is None else f'{int(raw * 1000.0)}'
            summary.append(f'{name}:raw_mm={raw_mm},status={status},pub={pub:.3f}')
        # self.get_logger().info('radar tof diag | ' + ' | '.join(summary))

    def _fetch_simulation_data_body(self) -> bytes:
        first_error: Exception | None = None
        total = len(self._urls)
        for offset in range(total):
            idx = (self._url_index + offset) % total
            url = self._urls[idx]
            try:
                req = urllib.request.Request(url, headers={'Connection': 'close'})
                with urllib.request.urlopen(req, timeout=self.request_timeout) as res:
                    body = res.read()
                self._url_index = idx
                return body
            except Exception as exc:
                if first_error is None:
                    first_error = exc
                continue

        if first_error is None:
            raise RuntimeError('no simulation_data urls configured')
        raise first_error

    def destroy_node(self) -> bool:
        for pin in self._tof_xshut_pins.values():
            try:
                pin.close()
            except Exception:
                pass
        self._tof_xshut_pins.clear()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RadarSensorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
