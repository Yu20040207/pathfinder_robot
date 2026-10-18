import os
import pty

import pytest

from fwr_bringup.base_serial import open_exclusive_serial


def test_second_owner_cannot_open_same_serial_device():
    master, slave = pty.openpty()
    path = os.ttyname(slave)
    first = open_exclusive_serial(path)
    try:
        with pytest.raises(BlockingIOError):
            open_exclusive_serial(path)
    finally:
        os.close(first)
        os.close(master)
        os.close(slave)
