"""Exercise real Windows console input and Ctrl+C without opening visible windows.

Run: python tests/native_console.py --bin-dir build/Release --output-dir test-output
"""
import argparse
import ctypes
from ctypes import wintypes
from pathlib import Path
import socket
import struct
import subprocess
import sys
import time

from integration import decode, free_port, packet, wait_until


class Character(ctypes.Union):
    _fields_ = [('UnicodeChar', wintypes.WCHAR), ('AsciiChar', ctypes.c_char)]


class KeyEvent(ctypes.Structure):
    _fields_ = [('bKeyDown', wintypes.BOOL), ('wRepeatCount', wintypes.WORD),
                ('wVirtualKeyCode', wintypes.WORD), ('wVirtualScanCode', wintypes.WORD),
                ('uChar', Character), ('dwControlKeyState', wintypes.DWORD)]


class EventData(ctypes.Union):
    _fields_ = [('KeyEvent', KeyEvent), ('Padding', ctypes.c_byte * 16)]


class InputRecord(ctypes.Structure):
    _fields_ = [('EventType', wintypes.WORD), ('Event', EventData)]


class Coord(ctypes.Structure):
    _fields_ = [('X', wintypes.SHORT), ('Y', wintypes.SHORT)]


class SmallRect(ctypes.Structure):
    _fields_ = [('Left', wintypes.SHORT), ('Top', wintypes.SHORT),
                ('Right', wintypes.SHORT), ('Bottom', wintypes.SHORT)]


class BufferInfo(ctypes.Structure):
    _fields_ = [('dwSize', Coord), ('dwCursorPosition', Coord), ('wAttributes', wintypes.WORD),
                ('srWindow', SmallRect), ('dwMaximumWindowSize', Coord)]


kernel = ctypes.WinDLL('kernel32', use_last_error=True)
kernel.AttachConsole.argtypes = [wintypes.DWORD]
kernel.AttachConsole.restype = wintypes.BOOL
kernel.FreeConsole.restype = wintypes.BOOL
kernel.SetConsoleCtrlHandler.argtypes = [ctypes.c_void_p, wintypes.BOOL]
kernel.SetConsoleCtrlHandler.restype = wintypes.BOOL
kernel.GenerateConsoleCtrlEvent.argtypes = [wintypes.DWORD, wintypes.DWORD]
kernel.GenerateConsoleCtrlEvent.restype = wintypes.BOOL
kernel.CreateFileW.argtypes = [wintypes.LPCWSTR, wintypes.DWORD, wintypes.DWORD,
                              ctypes.c_void_p, wintypes.DWORD, wintypes.DWORD, wintypes.HANDLE]
kernel.CreateFileW.restype = wintypes.HANDLE
kernel.CloseHandle.argtypes = [wintypes.HANDLE]
kernel.WriteConsoleInputW.argtypes = [wintypes.HANDLE, ctypes.POINTER(InputRecord), wintypes.DWORD,
                                     ctypes.POINTER(wintypes.DWORD)]
kernel.WriteConsoleInputW.restype = wintypes.BOOL
kernel.GetConsoleScreenBufferInfo.argtypes = [wintypes.HANDLE, ctypes.POINTER(BufferInfo)]
kernel.GetConsoleScreenBufferInfo.restype = wintypes.BOOL
kernel.ReadConsoleOutputCharacterW.argtypes = [wintypes.HANDLE, wintypes.LPWSTR, wintypes.DWORD,
                                              Coord, ctypes.POINTER(wintypes.DWORD)]
kernel.ReadConsoleOutputCharacterW.restype = wintypes.BOOL
kernel.SetConsoleWindowInfo.argtypes = [wintypes.HANDLE, wintypes.BOOL, ctypes.POINTER(SmallRect)]
kernel.SetConsoleWindowInfo.restype = wintypes.BOOL


def require(result, action):
    if not result:
        raise OSError(ctypes.get_last_error(), action)


def attach(process):
    kernel.FreeConsole()
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        if kernel.AttachConsole(process.pid):
            # Ctrl+C goes to this test process as well; only the child should act on it.
            require(kernel.SetConsoleCtrlHandler(None, True), 'ignore Ctrl+C in harness')
            return
        if process.poll() is not None:
            raise AssertionError('Application exited before console attachment')
        time.sleep(0.03)
    raise AssertionError('Could not attach to application console')


def type_text(text):
    handle = kernel.CreateFileW('CONIN$', 0xC0000000, 3, None, 3, 0, None)
    if handle == wintypes.HANDLE(-1).value:
        raise OSError(ctypes.get_last_error(), 'open CONIN$')
    try:
        for character in text:
            record = InputRecord()
            record.EventType = 1  # KEY_EVENT
            record.Event.KeyEvent.bKeyDown = True
            record.Event.KeyEvent.wRepeatCount = 1
            record.Event.KeyEvent.uChar.UnicodeChar = character
            written = wintypes.DWORD()
            require(kernel.WriteConsoleInputW(handle, ctypes.byref(record), 1, ctypes.byref(written)),
                    'write console key')
            assert written.value == 1
    finally:
        kernel.CloseHandle(handle)


def type_line(text):
    type_text(text + '\r')


def read_screen():
    handle = kernel.CreateFileW('CONOUT$', 0xC0000000, 3, None, 3, 0, None)
    if handle == wintypes.HANDLE(-1).value:
        raise OSError(ctypes.get_last_error(), 'open CONOUT$')
    try:
        info = BufferInfo()
        require(kernel.GetConsoleScreenBufferInfo(handle, ctypes.byref(info)), 'get screen dimensions')
        width = info.srWindow.Right - info.srWindow.Left + 1
        lines = []
        for row in range(info.srWindow.Top, info.srWindow.Bottom + 1):
            buffer = ctypes.create_unicode_buffer(width + 1)
            read = wintypes.DWORD()
            require(kernel.ReadConsoleOutputCharacterW(handle, buffer, width,
                    Coord(info.srWindow.Left, row), ctypes.byref(read)), 'read visible console row')
            lines.append(buffer.value.rstrip())
        return lines, info
    finally:
        kernel.CloseHandle(handle)


def check_dashboard(process, peer, port, output):
    wait_until(lambda: read_screen()[0][-1].startswith('gcs>'), 'fixed bottom input prompt')
    partial = 'goto 55.751 37.61'
    type_text(partial)
    wait_until(lambda: read_screen()[0][-1] == 'gcs> ' + partial, 'partially typed command')
    for sequence in range(1, 5):
        payload = struct.pack('<ddfffBB', 55.75 + sequence / 100000, 37.61, float(sequence), 0, 99, 0, 0)
        peer.sendto(packet(2, sequence, payload), ('127.0.0.1', port))
        wait_until(lambda: ('seq=' + str(sequence) + ' ') in '\n'.join(read_screen()[0]),
                   'updated telemetry on same screen')
        lines, info = read_screen()
        assert lines[-1] == 'gcs> ' + partial, 'telemetry moved or overwrote input'
        assert info.dwCursorPosition.Y == info.srWindow.Bottom, 'cursor left bottom input row'
        assert '\n'.join(lines).count('TELEMETRY') == 1, 'telemetry rows accumulated'
    type_text(' 15\r')
    data, address = peer.recvfrom(65535)
    message_id, sequence, payload = decode(data)
    assert message_id == 13 and struct.unpack('<ddf', payload) == (55.751, 37.61, 15.0)
    peer.sendto(packet(21, 5, struct.pack('<HIB', 13, sequence, 1)), address)
    wait_until(lambda: 'NACK GOTO' in '\n'.join(read_screen()[0]), 'reply in event area')
    assert read_screen()[0][-1] == 'gcs>', 'input did not reset after Enter'
    (output / 'dashboard-screen.txt').write_text('\n'.join(read_screen()[0]) + '\n', encoding='utf-8')
    # A command wider than the window must scroll within its own row.
    type_text('x' * 150)
    wait_until(lambda: read_screen()[0][-1].endswith('x' * 20), 'long input remains on one row')
    handle = kernel.CreateFileW('CONOUT$', 0xC0000000, 3, None, 3, 0, None)
    try:
        require(kernel.SetConsoleWindowInfo(handle, True, ctypes.byref(SmallRect(0, 0, 59, 14))),
                'resize own console window')
    finally:
        kernel.CloseHandle(handle)
    wait_until(lambda: len(read_screen()[0]) == 15 and read_screen()[0][-1].startswith('gcs>'),
               'prompt follows resized window bottom')
    lines, info = read_screen()
    assert info.dwCursorPosition.Y == info.srWindow.Bottom
    assert lines[-1].endswith('x' * 20), 'resize lost input tail'
    (output / 'dashboard-resized-screen.txt').write_text('\n'.join(lines) + '\n', encoding='utf-8')


def run_case(bin_dir, output, name):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as peer:
        peer.bind(('127.0.0.1', 0))
        peer.settimeout(3)
        application = 'GCS' if name == 'Dashboard' else name
        port = free_port()
        arguments = [str(bin_dir / (application + '.exe')), '--listen-port', str(port),
                     '--peer-port', str(peer.getsockname()[1])]
        if application == 'GCS':
            arguments += ['--log', str(output / 'native-input.csv')]
        startup = subprocess.STARTUPINFO()
        startup.dwFlags |= subprocess.STARTF_USESHOWWINDOW
        startup.wShowWindow = 0  # SW_HIDE
        # Exercise the inherited Windows attribute that can suppress Ctrl+C.
        require(kernel.SetConsoleCtrlHandler(None, True), 'set inherited ignore attribute')
        process = subprocess.Popen(arguments, startupinfo=startup, creationflags=subprocess.CREATE_NEW_CONSOLE)
        try:
            attach(process)
            if name == 'Dashboard':
                check_dashboard(process, peer, port, output)
            elif name == 'GCS':
                type_line('arm')
                data, address = peer.recvfrom(65535)
                message_id, sequence, payload = decode(data)
                assert message_id == 10 and payload == b'', 'real console must send ARM'
                peer.sendto(packet(20, 1, struct.pack('<HI', 10, sequence)), address)
                time.sleep(0.1)
            else:
                decode(peer.recvfrom(65535)[0])  # model has started and handler is installed
            require(kernel.GenerateConsoleCtrlEvent(0, 0), 'generate Ctrl+C')
            assert process.wait(3) == 0, name + ' did not exit cleanly on Ctrl+C'
        except Exception:
            if application == 'GCS' and process.poll() is None:
                lines, info = read_screen()
                print('CONSOLE SNAPSHOT:', repr(lines), flush=True)
                print('CURSOR:', info.dwCursorPosition.X, info.dwCursorPosition.Y, flush=True)
            raise
        finally:
            kernel.FreeConsole()
            if process.poll() is None:
                process.terminate()
                process.wait(3)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bin-dir', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, default=Path('test-output'))
    parser.add_argument('--case', choices=['GCS', 'DroneSim', 'Dashboard'], help=argparse.SUPPRESS)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    if args.case:
        run_case(args.bin_dir.resolve(), args.output_dir.resolve(), args.case)
        return
    for name in ('GCS', 'DroneSim', 'Dashboard'):
        # Keep the user's original console attached. Only a disposable worker
        # switches consoles, with stdout/stderr on stable redirected pipes.
        result = subprocess.run([sys.executable, str(Path(__file__).resolve()), '--case', name,
                                 '--bin-dir', str(args.bin_dir.resolve()),
                                 '--output-dir', str(args.output_dir.resolve())],
                                capture_output=True, text=True, timeout=25,
                                creationflags=subprocess.CREATE_NO_WINDOW)
        assert result.returncode == 0, result.stdout + result.stderr
        print('PASS native console and Ctrl+C:', name, flush=True)


if __name__ == '__main__':
    main()
