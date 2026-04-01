#!/usr/bin/env python3
# encoding: utf-8

import serial
import os
import pty
import time
import argparse
import threading
import signal


def parse_args():
    parser = argparse.ArgumentParser(
        description="Simple Sniffer Program to capture IMU Traffic"
    )

    parser.add_argument("port", type=str)
    parser.add_argument("outfile", nargs="?", type=str)
    return parser.parse_args()


def setup_ptys(port: str, outfile: str | None = None):
    master, slave = pty.openpty()

    master_name = os.ttyname(master)
    slave_name = os.ttyname(slave)
    if outfile is not None:
        os.symlink(slave_name, outfile)

    return (master, master_name), (slave, slave_name)


def spin(m, s):
    PACKET_LEN = 11

    m_ser = serial.Serial(m, baudrate=9600)
    s_ser = serial.Serial(s, baudrate=9600)

    while True:
        try:
            pass
        except KeyboardInterrupt as ex:
            break

        if m_ser.in_waiting >= 0:
            m_data = m_ser.read(1)
            if m_data == b"\x55":
                m_data += m_ser.read(PACKET_LEN - 1)

                s_ser.write(m_data)

        if s_ser.in_waiting >= 0:
            s_data = s_ser.read(1)
            if s_data == b"\xff":
                s_data += s_ser.read(s_ser.in_waiting)

                m_ser.write(s_data)

    pass


def main():
    args = parse_args()
    (master, m_name), (slave, s_name) = setup_ptys(args.port, args.outfile)

    if args.outfile is not None:
        print(f"Create symlink: {s_name} -> {args.outfile}")

    print(f"Connected: {args.port} -> {s_name} -> {args.outfile}")

    spin(args.port, slave)

    os.close(master)
    os.close(slave)
    if args.outfile is not None:
        os.remove(args.outfile)


if __name__ == "__main__":
    main()
