"""Windows localhost integration tests. Python 3 standard library only.

Run: python tests/integration.py --bin-dir build/Release --output-dir test-output
The applications and C++ unit tests do not require Python.
"""
import argparse
import csv
from pathlib import Path
import socket
import struct
import subprocess
import time


def packet(message_id, sequence, payload=b''):
    body = struct.pack('<IHIH', 0x44524F4E, message_id, sequence, len(payload)) + payload
    return body + struct.pack('<H', sum(body) & 0xFFFF)


def decode(data):
    assert len(data) >= 14, 'truncated packet received from application'
    magic, message_id, sequence, size = struct.unpack('<IHIH', data[:12])
    assert magic == 0x44524F4E and len(data) == size + 14, 'invalid header'
    assert sum(data[:-2]) & 0xFFFF == struct.unpack('<H', data[-2:])[0], 'invalid checksum'
    return message_id, sequence, data[12:-2]


def free_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


def wait_until(predicate, description, timeout=5):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.03)
    raise AssertionError('Timed out: ' + description)


class App:
    def __init__(self, executable, output, *arguments):
        self.path = output
        self.stream = output.open('w', encoding='utf-8')
        self.process = subprocess.Popen([str(executable), *map(str, arguments)], stdin=subprocess.PIPE,
                                        stdout=self.stream, stderr=subprocess.STDOUT,
                                        creationflags=subprocess.CREATE_NO_WINDOW)

    def text(self):
        return self.path.read_text(encoding='utf-8', errors='replace')

    def send(self, text):
        self.process.stdin.write((text + '\n').encode())
        self.process.stdin.flush()

    def wait_for(self, text, timeout=5):
        wait_until(lambda: text in self.text(), text, timeout)

    def finish(self, timeout=3):
        code = self.process.wait(timeout)
        assert code == 0, self.text()

    def close(self):
        if self.process.poll() is None:
            self.process.terminate()
            self.process.wait(3)
        self.process.stdin.close()
        self.stream.close()


def read_reply(sock, sequence, timeout=2):
    deadline = time.monotonic() + timeout
    sock.settimeout(0.15)
    while time.monotonic() < deadline:
        try:
            message_id, sender_sequence, payload = decode(sock.recv(65535))
        except socket.timeout:
            continue
        if message_id in (20, 21) and struct.unpack('<I', payload[2:6])[0] == sequence:
            return message_id, sender_sequence, payload
    raise AssertionError('Missing reply for command ' + str(sequence))


def test_drone_protocol(bin_dir, output):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as ground:
        ground.bind(('127.0.0.1', 0))
        drone_port = free_port()
        app = App(bin_dir / 'DroneSim.exe', output / 'drone-protocol.txt',
                  '--listen-port', drone_port, '--peer-port', ground.getsockname()[1], '--duration', 10)
        try:
            app.wait_for('DroneSim ready')
            address = ('127.0.0.1', drone_port)
            ground.sendto(packet(13, 1, struct.pack('<ddf', 55.751, 37.61, 10)), address)
            reply = read_reply(ground, 1)
            assert reply[0] == 21 and reply[2][-1] == 1, 'GOTO must reject NOT_ARMED'
            ground.sendto(packet(10, 2), address)
            first = read_reply(ground, 2)
            assert first[0] == 20, 'ARM accepted'
            ground.sendto(packet(10, 2), address)
            repeated = read_reply(ground, 2)
            assert repeated[0] == 20 and repeated[2] == first[2], 'retry must replay original ACK'
            assert repeated[1] != first[1], 'every response gets a new sender seq'
            ground.sendto(packet(11, 2), address)
            conflict = read_reply(ground, 2)
            assert conflict[0] == 21 and conflict[2][-1] == 3, 'same seq different command rejected'
            ground.sendto(packet(10, 3), address)
            assert read_reply(ground, 3)[2][-1] == 4, 'new ARM while armed returns BUSY'
            ground.sendto(packet(12, 4, b'\x09'), address)
            assert read_reply(ground, 4)[2][-1] == 3, 'invalid mode rejected'
            ground.sendto(packet(11, 5, b'\x00'), address)
            assert read_reply(ground, 5)[2][-1] == 3, 'wrong command payload rejected'
            damaged = bytearray(packet(11, 6))
            damaged[-1] ^= 1
            ground.sendto(damaged, address)
            ground.sendto(b'', address)
            ground.sendto(b'noise', address)
            ground.sendto(packet(11, 7), address)
            assert read_reply(ground, 7)[0] == 20, 'parser remains alive after garbage'
        finally:
            app.close()


def test_demo(bin_dir, output):
    drone_port, ground_port = free_port(), free_port()
    while ground_port == drone_port:
        ground_port = free_port()
    log = output / 'flight.csv'
    drone = App(bin_dir / 'DroneSim.exe', output / 'demo-drone.txt',
                '--listen-port', drone_port, '--peer-port', ground_port, '--duration', 12)
    ground = App(bin_dir / 'GCS.exe', output / 'demo-gcs.txt',
                 '--listen-port', ground_port, '--peer-port', drone_port, '--log', log)
    try:
        ground.wait_for('TELEMETRY')
        for line, reply in [('arm', 'ACK ARM'), ('mode 1', 'ACK SET_MODE'),
                            ('goto 55.751 37.61 15', 'ACK GOTO')]:
            ground.send(line)
            ground.wait_for(reply)
        def has_movement():
            with log.open(newline='') as stream:
                return any(float(row['lat']) > 55.75005 and float(row['alt']) > 0
                           for row in csv.DictReader(stream))
        wait_until(has_movement, 'coordinate movement in CSV')
        ground.send('status')
        ground.wait_for('heartbeat_age=')
        ground.send('disarm')
        ground.wait_for('ACK DISARM')
        time.sleep(0.3)
        ground.send('quit')
        ground.finish()
        with log.open(newline='') as stream:
            rows = list(csv.DictReader(stream))
        assert len(rows) >= 5, 'CSV must include ongoing telemetry'
        assert rows[-1]['armed'] == '0', 'CSV shows disarmed state'
        assert list(rows[0]) == ['timestamp', 'seq', 'lat', 'lon', 'alt', 'yaw', 'battery', 'armed', 'mode']
    finally:
        ground.close()
        drone.close()


def test_gcs_retry(bin_dir, output):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as board:
        board.bind(('127.0.0.1', 0))
        board.settimeout(3)
        ground_port = free_port()
        ground = App(bin_dir / 'GCS.exe', output / 'gcs-retry.txt', '--listen-port', ground_port,
                     '--peer-port', board.getsockname()[1], '--log', output / 'retry.csv')
        try:
            ground.wait_for('GCS ready')
            ground.send('arm')
            first, address = board.recvfrom(65535)
            _, sequence, _ = decode(first)
            # A correct cmd_seq paired with a wrong cmd_id must be ignored.
            board.sendto(packet(20, 1, struct.pack('<HI', 11, sequence)), address)
            repeated, _ = board.recvfrom(65535)
            assert first == repeated, 'retry preserves exact packet'
            board.sendto(packet(20, 2, struct.pack('<HI', 10, sequence)), address)
            ground.wait_for('ACK ARM')
            ground.send('disarm')
            attempts = [board.recvfrom(65535)[0] for _ in range(3)]
            assert attempts[0] == attempts[1] == attempts[2], 'exactly three same attempts'
            ground.wait_for('TIMEOUT DISARM', 4)
            board.settimeout(0.3)
            try:
                board.recvfrom(65535)
                raise AssertionError('retry limit exceeded')
            except socket.timeout:
                pass
            ground.send('quit')
            ground.finish()
        finally:
            ground.close()


def test_shutdown_and_errors(bin_dir, output):
    for name in ('GCS', 'DroneSim'):
        app = App(bin_dir / (name + '.exe'), output / (name + '-duration.txt'),
                  '--listen-port', free_port(), '--peer-port', free_port(), '--duration', 0.3,
                  *(['--log', output / 'shutdown.csv'] if name == 'GCS' else []))
        try:
            app.finish(3)  # stdin stays open and has no input throughout.
        finally:
            app.close()
    ground = App(bin_dir / 'GCS.exe', output / 'gcs-eof.txt', '--listen-port', free_port(),
                 '--peer-port', free_port(), '--log', output / 'eof.csv')
    try:
        ground.wait_for('GCS ready')
        ground.process.stdin.close()
        ground.finish()
    finally:
        ground.close()
    app = App(bin_dir / 'GCS.exe', output / 'bad-log.txt', '--log', output / 'absent' / 'flight.csv')
    try:
        assert app.process.wait(3) != 0, 'bad log path must fail without waiting for stdin'
    finally:
        app.close()
    for arguments in (['--speed', 'nan'], ['--listen-port', '70000'], ['--duration', '-1'], ['--unknown']):
        result = subprocess.run([str(bin_dir / 'DroneSim.exe'), *arguments], capture_output=True, timeout=3)
        assert result.returncode != 0, 'invalid options must fail: ' + str(arguments)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as occupied:
        occupied.bind(('127.0.0.1', 0))
        result = subprocess.run([str(bin_dir / 'DroneSim.exe'), '--listen-port', str(occupied.getsockname()[1])],
                                capture_output=True, timeout=3)
        assert result.returncode != 0, 'busy port must report startup failure'


def test_finite_input_and_tabs(bin_dir, output):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as board:
        board.bind(('127.0.0.1', 0))
        board.settimeout(2)
        ground = App(bin_dir / 'GCS.exe', output / 'gcs-finite-input.txt', '--listen-port', free_port(),
                     '--peer-port', board.getsockname()[1], '--log', output / 'finite.csv')
        try:
            ground.wait_for('GCS ready')
            ground.send('ar\tm')
            ground.wait_for('Input error:')
            board.settimeout(0.2)
            try:
                board.recvfrom(65535)
                raise AssertionError('tab inside command name must not turn it into ARM')
            except socket.timeout:
                pass
            board.settimeout(2)
            ground.send('arm\nmode\t1')
            ground.process.stdin.close()
            for expected_id, sender_sequence in [(10, 1), (12, 2)]:
                data, address = board.recvfrom(65535)
                message_id, sequence, payload = decode(data)
                assert message_id == expected_id, 'EOF must drain commands in order'
                if message_id == 12:
                    assert payload == b'\x01', 'tab can separate CLI arguments'
                board.sendto(packet(20, sender_sequence, struct.pack('<HI', message_id, sequence)), address)
            ground.finish()
        finally:
            ground.close()


def test_input_bounds_and_eof_timeout(bin_dir, output):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as board:
        board.bind(('127.0.0.1', 0))
        board.settimeout(3)
        ground = App(bin_dir / 'GCS.exe', output / 'gcs-input-bounds.txt', '--listen-port', free_port(),
                     '--peer-port', board.getsockname()[1], '--log', output / 'bounds.csv')
        try:
            ground.wait_for('GCS ready')
            ground.send('ar\x00m')
            ground.send('arm' + ' ' * 2048)
            wait_until(lambda: ground.text().count('Input error:') == 2, 'reject invalid and oversized lines')
            board.settimeout(0.2)
            try:
                board.recvfrom(65535)
                raise AssertionError('invalid or oversized line executed partially')
            except socket.timeout:
                pass
            board.settimeout(3)
            ground.send('arm')
            ground.process.stdin.close()
            attempts = [board.recvfrom(65535)[0] for _ in range(3)]
            assert attempts[0] == attempts[1] == attempts[2], 'EOF keeps bounded command retries'
            ground.finish(4)
            assert 'TIMEOUT ARM' in ground.text(), 'EOF must finish after timeout without an ACK'
        finally:
            ground.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bin-dir', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, default=Path('test-output'))
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    tests = [test_drone_protocol, test_demo, test_gcs_retry, test_shutdown_and_errors,
             test_finite_input_and_tabs, test_input_bounds_and_eof_timeout]
    for test in tests:
        test(args.bin_dir.resolve(), args.output_dir.resolve())
        print('PASS', test.__name__, flush=True)
    print('All integration scenarios passed.', flush=True)


if __name__ == '__main__':
    main()
