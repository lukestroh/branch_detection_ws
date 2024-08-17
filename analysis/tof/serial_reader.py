#!/usr/bin/env python3
import serial
import sys

import re

re_pattern = re.compile('[\W_]+', re.UNICODE)

def create_connection(port: str = "/dev/ttyACM0") -> serial.Serial:
    try:
        ser = serial.Serial(port=port)
    except serial.serialutil.SerialException as e:
        print(f"{e}")
        sys.exit()
    ser.baudrate = 115200
    return ser

def read_stream(ser: serial.Serial) -> str | None:
    try:
        content = ser.readline().decode().strip()
    except UnicodeDecodeError:
        return None
    return content

def parse_line(data: str) -> dict | None:
    list_data = {}
    list_data = [subdata.split(":") for subdata in data.split(',')]
    try:
        parsed_data = {re_pattern.sub('', key.strip().lower()): int(value) for key, value in list_data}
    except ValueError:
        return None
    return parsed_data


def main():
    try:
        ser = create_connection()
    except serial.serialutil.SerialException:
        return
    while True:
        content = read_stream(ser)
        if content is None:
            continue
        parsed_content = parse_line(content)
        print(content)
    return

if __name__ == "__main__":
    main()
