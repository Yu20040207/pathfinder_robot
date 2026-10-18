"""Exclusive, raw ESP32 USB serial access."""

import fcntl
import os
import termios


def open_exclusive_serial(device: str) -> int:
    fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        attrs = termios.tcgetattr(fd)
        attrs[0] = 0
        attrs[1] = 0
        attrs[2] = ((attrs[2] & ~(termios.PARENB | termios.CSTOPB | termios.CSIZE)) |
                    termios.CS8 | termios.CLOCAL | termios.CREAD)
        if hasattr(termios, 'CRTSCTS'):
            attrs[2] &= ~termios.CRTSCTS
        attrs[3] = 0
        attrs[4] = termios.B115200
        attrs[5] = termios.B115200
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        return fd
    except BaseException:
        os.close(fd)
        raise
