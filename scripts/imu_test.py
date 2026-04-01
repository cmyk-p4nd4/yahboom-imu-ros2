#!/usr/bin/env python3

import serial
import time
import signal
import sys

RESPONSE_LEN = 11


g_exit = False
kill_count = 0


def signal_handler(signum, frame):
    global g_exit
    global kill_count
    g_exit = True
    kill_count += 1
    if kill_count >= 4:
        print()
        sys.exit(-2)
    pass


def main():
    global g_exit

    signal.signal(signal.SIGINT, signal_handler)

    now = time.time()

    port = "/dev/ttyUSB0"
    baud = 921600

    ser = serial.Serial(port, baud, timeout=1)
    print("Serial open:")

    ser.reset_input_buffer()
    ser.reset_output_buffer()

    # unlock
    # print("Send key ...")
    # command = [0xFF, 0xAA, 0x69, 0x88, 0xB5]
    # print(f"\x1b[1;36m->\x1b[0m {bytes(command).hex(' ').upper()}")
    # ser.write(bytes(command))
    # time.sleep(0.1)
    # get baud rate
    print("reading output content")
    command = [0xFF, 0xAA, 0x27, 0x02, 0x00]
    print(f"\x1b[1;36m->\x1b[0m {bytes(command).hex(' ').upper()}")
    ser.write(bytes(command))
    # time.sleep(0.1)

    # print("Sending Save command ...")
    # command = [0xFF, 0xAA, 0x00, 0x00, 0x00]
    # print(f"\x1b[1;36m->\x1b[0m {bytes(command).hex(' ').upper()}")
    # ser.write(bytes(command))

    for _ in range(30):
        data = bytes()
        while data != b"\x55":
            data = ser.read(1)
        data += ser.read(RESPONSE_LEN - 1)
        elapsed = time.time() - now
        print(
            f"\x1b[2K\r[{elapsed:.6f}] \x1b[1;31m<-\x1b[0m ({len(data)}) {data.hex(' ').upper()}",
            end="",
        )
    print()

    time.sleep(1)
    g_exit = False
    ser.reset_output_buffer()
    print("\x1b[1mChanging Configuration Now!\x1b[0m")
    time.sleep(1)

    # unlock
    print("Send key ...")
    command = [0xFF, 0xAA, 0x69, 0x88, 0xB5]
    print(f"\x1b[1;36m->\x1b[0m {bytes(command).hex(' ').upper()}")
    ser.write(bytes(command))
    # time.sleep(0.1)

    # set baudate command
    print("Sending set rrate command ...")
    command = [0xFF, 0xAA, 0x03, 0x02, 0x00]
    print(f"\x1b[1;36m->\x1b[0m {bytes(command).hex(' ').upper()}")
    ser.write(bytes(command))
    # time.sleep(0.1)

    print("Sending Save command ...")
    command = [0xFF, 0xAA, 0x00, 0x00, 0x00]
    print(f"\x1b[1;36m->\x1b[0m {bytes(command).hex(' ').upper()}")
    ser.write(bytes(command))

    # for _ in range(10):
    #     data = bytes()
    #     while data != b"\x55":
    #         data = ser.read(1)
    #     data += ser.read(RESPONSE_LEN - 1)
    #     elapsed = time.time() - now
    #     print(
    #         f"[{elapsed:.6f}] \x1b[1;31m<-\x1b[0m ({len(data)}) {data.hex(' ').upper()}",
    #     )
    time.sleep(2e-3)
    print("-" * 50)

    # print("Send key ...")
    # command = [0xFF, 0xAA, 0x69, 0x88, 0xB5]
    # print(f"\x1b[1;36m->\x1b[0m {bytes(command).hex(' ').upper()}")
    # ser.write(bytes(command))
    print("reading output content")
    command = [0xFF, 0xAA, 0x27, 0x03, 0x00]
    print(f"\x1b[1;36m->\x1b[0m {bytes(command).hex(' ').upper()}")
    ser.write(bytes(command))

    for _ in range(100):
        data = bytes()
        while data != b"\x55":
            data = ser.read(1)
        data += ser.read(RESPONSE_LEN - 1)
        elapsed = time.time() - now
        if data[0:2] == b"\x55\x5f":
            print(
                f"[{elapsed:.6f}] \x1b[1;31m<-\x1b[0m ({len(data)}) \x1b[1;36m\x1b[1m{data.hex(' ').upper()}\x1b[0m",
            )
            pass
        else:
            print(
                f"[{elapsed:.6f}] \x1b[1;31m<-\x1b[0m ({len(data)}) {data.hex(' ').upper()}",
            )
    print()

    ser.close()
    pass


if __name__ == "__main__":
    main()
