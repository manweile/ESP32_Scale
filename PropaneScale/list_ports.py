#!/usr/bin/env python3
'''
@file list_ports.py
test run list@brief Lists active COM ports, optionally filtered by USB VID/PID.
@details Useful for VS Code task inputs to deterministically select the ESP32 port.

@author Gerald Manweiler
@copyright @showdate "%Y" GWN Software. All rights reserved.
'''

# standard modules
import argparse
import os
import serial.tools.list_ports
import sys


# Hardcode your adapter IDs here for deterministic ESP32 port selection.
# Common CP210x example values shown below.
ESP32_DEFAULT_VID = 0x0403
ESP32_DEFAULT_PID = 0x6015

# Toggle this on/off per project.
# True  = use hardcoded ESP32 defaults when no --vid/--pid or env values are provided.
# False = do not apply hardcoded defaults; list all ports unless --vid/--pid or env values are provided.
USE_HARDCODED_ESP32_FILTER = True

# Optional secondary discriminator when multiple devices share the same VID/PID.
# Match is case-insensitive against serial_number, description, manufacturer,
# product, interface, hwid, and location.
# Leave empty ('') to disable.
ESP32_PORT_HINT = 'DN0402DQ'

# Optional stable fallback port name if more than one match remains after hint.
# Example: 'COM4'. Leave empty to output all matching ports.
ESP32_PREFERRED_COM = ''


def parse_usb_id(text):
    '''
    @brief Parses VID/PID input as decimal or hex.
    @param text String value such as "4292", "0x10C4", or "10c4".
    @return Integer ID value.
    '''
    value = text.strip().lower()
    if value.startswith('0x'):
        return int(value, 16)

    # If letters a-f appear, treat as hex (example: "10c4")
    if any(c in 'abcdef' for c in value):
        return int(value, 16)

    return int(value, 10)


def list_com_ports(target_vid=None, target_pid=None):
    '''
    @brief Lists active COM ports.
    @details Called by tasks.json input rule to supply list of com ports for selection.
             When target_vid and target_pid are provided, only matching ports are printed.
    '''
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print('No COM ports found', file=sys.stderr)
        sys.exit(1)

    matches = []
    for port in ports:
        # The task will use each line as a selection option.
        if target_vid is None or target_pid is None:
            matches.append(port.device)
            continue

        if port.vid == target_vid and port.pid == target_pid:
            matches.append(port)

    if target_vid is not None and target_pid is not None and matches:
        hint = ESP32_PORT_HINT.strip().lower()
        if hint:
            hinted = []
            for port in matches:
                text = ' '.join(str(x).lower() for x in (
                    port.serial_number,
                    port.description,
                    port.manufacturer,
                    port.product,
                    port.interface,
                    port.hwid,
                    getattr(port, 'location', None),
                ) if x)
                if hint in text:
                    hinted.append(port)

            if hinted:
                matches = hinted

        preferred = ESP32_PREFERRED_COM.strip().upper()
        if preferred:
            preferred_matches = [p for p in matches if p.device.upper() == preferred]
            if preferred_matches:
                matches = preferred_matches

    if not matches:
        if target_vid is not None and target_pid is not None:
            print(f'No COM ports found for VID=0x{target_vid:04X} PID=0x{target_pid:04X}', file=sys.stderr)
        else:
            print('No COM ports found', file=sys.stderr)
        sys.exit(1)

    for port in matches:
        print(port.device)


def main():
    '''
    @brief Program entry point.
    '''
    parser = argparse.ArgumentParser(description='List COM ports, optionally filtered by USB VID/PID.')
    parser.add_argument('--vid', help='USB vendor ID, decimal or hex (example: 4292 or 0x10C4)')
    parser.add_argument('--pid', help='USB product ID, decimal or hex (example: 60000 or 0xEA60)')
    args = parser.parse_args()

    # Priority: CLI args > environment variables > hardcoded defaults.
    raw_vid = args.vid if args.vid is not None else os.getenv('ESP32_VID')
    raw_pid = args.pid if args.pid is not None else os.getenv('ESP32_PID')

    if USE_HARDCODED_ESP32_FILTER and raw_vid is None and raw_pid is None:
        raw_vid = str(ESP32_DEFAULT_VID)
        raw_pid = str(ESP32_DEFAULT_PID)

    if (raw_vid is None) != (raw_pid is None):
        print('Both VID and PID must be provided together.', file=sys.stderr)
        sys.exit(2)

    target_vid = None
    target_pid = None

    if raw_vid is not None and raw_pid is not None:
        try:
            target_vid = parse_usb_id(raw_vid)
            target_pid = parse_usb_id(raw_pid)
        except ValueError:
            print('Invalid VID/PID. Use decimal or hex values such as 0x10C4.', file=sys.stderr)
            sys.exit(2)

    list_com_ports(target_vid=target_vid, target_pid=target_pid)


if __name__ == '__main__':
    main()
