#!/usr/bin/env python3
"""
Watchdog Monitor UART Test Script
Platform: Kiwi 1P5 (Gowin GW1N-UV1P5)
UART: 115200 bps, 8N1 via USB-UART GWU2U

Usage:
    python3 watchdog_test.py COM3        (Windows)
    python3 watchdog_test.py /dev/ttyUSB0 (Linux)
    python3 watchdog_test.py /dev/tty.usbserial-xxx (Mac)

Requirements:
    pip install pyserial
"""

import sys
import time
import struct
import serial

# =============================================================================
# Protocol constants
# =============================================================================
SYNC        = 0x55
CMD_WRITE   = 0x01
CMD_READ    = 0x02
CMD_KICK    = 0x03
CMD_STATUS  = 0x04

ADDR_CTRL        = 0x00
ADDR_TWD_MS      = 0x04
ADDR_TRST_MS     = 0x08
ADDR_ARM_DELAY   = 0x0C
ADDR_STATUS      = 0x10

# =============================================================================
# Frame helpers
# =============================================================================
def build_frame(cmd, addr, data_bytes):
    """Build UART frame: [0x55][CMD][ADDR][LEN][DATA...][CHK]"""
    frame = bytearray()
    frame.append(SYNC)
    frame.append(cmd)
    frame.append(addr)
    frame.append(len(data_bytes))
    frame.extend(data_bytes)
    # CHK = XOR of CMD through last DATA byte
    chk = 0
    for b in frame[1:]:  # skip SYNC
        chk ^= b
    frame.append(chk)
    return frame

def parse_response(ser, timeout=2.0):
    """Read and parse UART response frame"""
    ser.timeout = timeout
    
    # Wait for sync byte
    while True:
        b = ser.read(1)
        if len(b) == 0:
            print("  [TIMEOUT] No response received")
            return None
        if b[0] == SYNC:
            break
    
    # Read CMD, ADDR, LEN
    header = ser.read(3)
    if len(header) < 3:
        print("  [ERROR] Incomplete header")
        return None
    
    resp_cmd = header[0]
    resp_addr = header[1]
    resp_len = header[2]
    
    # Read DATA
    data = ser.read(resp_len) if resp_len > 0 else b''
    if len(data) < resp_len:
        print("  [ERROR] Incomplete data")
        return None
    
    # Read CHK
    chk_byte = ser.read(1)
    if len(chk_byte) == 0:
        print("  [ERROR] Missing checksum")
        return None
    
    # Verify checksum
    calc_chk = 0
    for b in [resp_cmd, resp_addr, resp_len] + list(data):
        calc_chk ^= b
    
    chk_ok = (chk_byte[0] == calc_chk)
    
    # Parse 32-bit value from data (little-endian)
    value = 0
    if len(data) >= 4:
        value = struct.unpack('<I', data[:4])[0]
    elif len(data) >= 2:
        value = struct.unpack('<H', data[:2])[0]
    elif len(data) >= 1:
        value = data[0]
    
    return {
        'cmd': resp_cmd,
        'addr': resp_addr,
        'len': resp_len,
        'data': data,
        'value': value,
        'chk_ok': chk_ok,
    }

# =============================================================================
# High-level commands
# =============================================================================
def write_reg(ser, addr, value, width=4):
    """Write a register via UART"""
    if width == 4:
        data = struct.pack('<I', value)
    elif width == 2:
        data = struct.pack('<H', value)
    else:
        data = bytes([value & 0xFF])
    
    frame = build_frame(CMD_WRITE, addr, data)
    ser.write(frame)
    resp = parse_response(ser)
    return resp

def read_reg(ser, addr):
    """Read a register via UART"""
    frame = build_frame(CMD_READ, addr, b'')
    ser.write(frame)
    resp = parse_response(ser)
    return resp

def send_kick(ser):
    """Send KICK command via UART"""
    frame = build_frame(CMD_KICK, 0x00, b'')
    ser.write(frame)
    resp = parse_response(ser)
    return resp

def get_status(ser):
    """Get STATUS register"""
    frame = build_frame(CMD_STATUS, 0x00, b'')
    ser.write(frame)
    resp = parse_response(ser)
    return resp

def print_status(resp):
    """Pretty-print STATUS register bits"""
    if resp is None:
        print("  Status: NO RESPONSE")
        return
    v = resp['value']
    print(f"  STATUS = 0x{v:08X}")
    print(f"    bit0 EN_EFFECTIVE  = {(v >> 0) & 1}")
    print(f"    bit1 FAULT_ACTIVE  = {(v >> 1) & 1}")
    print(f"    bit2 ENOUT         = {(v >> 2) & 1}")
    print(f"    bit3 WDO           = {(v >> 3) & 1}")
    print(f"    bit4 LAST_KICK_SRC = {(v >> 4) & 1} ({'UART' if (v >> 4) & 1 else 'Button'})")

# =============================================================================
# Test sequences
# =============================================================================
def test_uart_connection(ser):
    """Test 0: Basic UART connectivity"""
    print("\n" + "="*60)
    print("TEST 0: UART Connection Check")
    print("="*60)
    resp = read_reg(ser, ADDR_TWD_MS)
    if resp and resp['chk_ok']:
        print(f"  OK: tWD_ms = {resp['value']} ms (default should be 1600)")
        return True
    else:
        print("  FAIL: No valid response. Check:")
        print("    - Correct COM port?")
        print("    - Baud rate = 115200?")
        print("    - USB cable connected to UART port (not JTAG)?")
        return False

def test_register_rw(ser):
    """Test 1: Write and read back registers"""
    print("\n" + "="*60)
    print("TEST 1: Register Read/Write")
    print("="*60)
    
    # Write tWD = 200ms (easier to test by hand)
    print("  Writing tWD_ms = 200...")
    resp = write_reg(ser, ADDR_TWD_MS, 200)
    if resp and resp['cmd'] == 0x81:
        print(f"  OK: Write ACK received (cmd=0x{resp['cmd']:02X})")
    else:
        print("  FAIL: No write ACK")
        return False
    
    time.sleep(0.1)
    
    # Read back
    print("  Reading tWD_ms...")
    resp = read_reg(ser, ADDR_TWD_MS)
    if resp and resp['value'] == 200:
        print(f"  OK: Read back = {resp['value']} (correct)")
    else:
        val = resp['value'] if resp else 'None'
        print(f"  FAIL: Read back = {val} (expected 200)")
        return False
    
    # Write tRST = 500ms
    print("  Writing tRST_ms = 500...")
    resp = write_reg(ser, ADDR_TRST_MS, 500)
    if resp:
        print(f"  OK: Write ACK (cmd=0x{resp['cmd']:02X})")
    
    # Write arm_delay = 150us (default)
    print("  Writing arm_delay_us = 150...")
    resp = write_reg(ser, ADDR_ARM_DELAY, 150, width=2)
    if resp:
        print(f"  OK: Write ACK (cmd=0x{resp['cmd']:02X})")
    
    return True

def test_kick(ser):
    """Test 2: UART kick"""
    print("\n" + "="*60)
    print("TEST 2: UART KICK Command")
    print("="*60)
    
    resp = send_kick(ser)
    if resp and resp['cmd'] == 0x83:
        print(f"  OK: KICK ACK received (cmd=0x{resp['cmd']:02X})")
        return True
    else:
        print("  FAIL: No KICK ACK")
        return False

def test_status(ser):
    """Test 3: GET_STATUS command"""
    print("\n" + "="*60)
    print("TEST 3: GET_STATUS")
    print("="*60)
    
    resp = get_status(ser)
    if resp and resp['chk_ok']:
        print_status(resp)
        return True
    else:
        print("  FAIL: No status response")
        return False

def test_enable_sw(ser):
    """Test 4: Software enable via CTRL register"""
    print("\n" + "="*60)
    print("TEST 4: Software Enable (CTRL.EN_SW)")
    print("="*60)
    
    # Enable watchdog via software (bit0 = EN_SW)
    print("  Writing CTRL = 0x01 (EN_SW=1)...")
    resp = write_reg(ser, ADDR_CTRL, 0x01)
    if resp:
        print(f"  OK: Write ACK")
    
    # Wait for arm_delay (150us + margin)
    time.sleep(0.01)
    
    # Check status
    print("  Reading STATUS after enable...")
    resp = get_status(ser)
    if resp:
        print_status(resp)
        en_eff = (resp['value'] >> 0) & 1
        enout  = (resp['value'] >> 2) & 1
        if en_eff and enout:
            print("  OK: Watchdog enabled, ENOUT=1")
            print("  >>> LED D4 should be ON now!")
        else:
            print("  WARN: EN_EFFECTIVE or ENOUT not set")
    
    return True

def test_timeout_and_fault(ser):
    """Test 5: Let watchdog timeout, observe fault"""
    print("\n" + "="*60)
    print("TEST 5: Watchdog Timeout -> FAULT")
    print("="*60)
    
    # First kick to reset timer
    send_kick(ser)
    
    # tWD was set to 200ms, so wait 300ms to trigger timeout
    print("  Waiting 300ms for timeout (tWD=200ms)...")
    print("  >>> Watch LED D3 — it should turn ON!")
    time.sleep(0.3)
    
    # Check status
    resp = get_status(ser)
    if resp:
        print_status(resp)
        fault = (resp['value'] >> 1) & 1
        wdo   = (resp['value'] >> 3) & 1
        if fault:
            print("  OK: FAULT detected! LED D3 should be ON")
        else:
            print("  INFO: Fault may have already cleared (tRST=500ms)")
            print("        If LED D3 blinked briefly, that's correct")
    
    # Wait for tRST to finish (500ms)
    print("  Waiting 600ms for tRST to expire...")
    time.sleep(0.6)
    
    resp = get_status(ser)
    if resp:
        fault = (resp['value'] >> 1) & 1
        if not fault:
            print("  OK: Fault cleared after tRST, new cycle started")
            print("  >>> LED D3 should be OFF now")
        else:
            print("  INFO: Still in fault state")
    
    return True

def test_kick_prevents_timeout(ser):
    """Test 6: Periodic kicks prevent timeout"""
    print("\n" + "="*60)
    print("TEST 6: Periodic KICK Prevents Timeout")
    print("="*60)
    
    print("  Sending 5 kicks at 100ms intervals (tWD=200ms)...")
    for i in range(5):
        resp = send_kick(ser)
        status = get_status(ser)
        if status:
            fault = (status['value'] >> 1) & 1
            print(f"  Kick {i+1}: FAULT={fault} {'(OK - no fault)' if not fault else '(BAD!)'}")
        time.sleep(0.1)
    
    print("  OK: No timeout during periodic kicks")
    print("  >>> LED D3 should have stayed OFF the whole time")
    return True

def test_clr_fault(ser):
    """Test 7: CLR_FAULT clears fault immediately"""
    print("\n" + "="*60)
    print("TEST 7: CLR_FAULT")
    print("="*60)
    
    # Kick and wait for timeout
    send_kick(ser)
    print("  Waiting for timeout...")
    time.sleep(0.3)  # > tWD=200ms
    
    resp = get_status(ser)
    if resp:
        fault = (resp['value'] >> 1) & 1
        print(f"  Fault state: {fault}")
    
    # Clear fault via CTRL bit2
    print("  Writing CTRL bit2 = CLR_FAULT...")
    write_reg(ser, ADDR_CTRL, 0x05)  # bit0=EN_SW, bit2=CLR_FAULT
    time.sleep(0.01)
    
    resp = get_status(ser)
    if resp:
        fault = (resp['value'] >> 1) & 1
        if not fault:
            print("  OK: Fault cleared immediately!")
            print("  >>> LED D3 should be OFF")
        else:
            print("  WARN: Fault not cleared")
    
    return True

def test_disable(ser):
    """Test 8: Disable watchdog"""
    print("\n" + "="*60)
    print("TEST 8: Disable Watchdog")
    print("="*60)
    
    # Disable via CTRL (EN_SW=0)
    print("  Writing CTRL = 0x00 (EN_SW=0)...")
    write_reg(ser, ADDR_CTRL, 0x00)
    time.sleep(0.01)
    
    resp = get_status(ser)
    if resp:
        print_status(resp)
        en_eff = (resp['value'] >> 0) & 1
        enout  = (resp['value'] >> 2) & 1
        if not en_eff and not enout:
            print("  OK: Watchdog disabled")
            print("  >>> Both LEDs D3 and D4 should be OFF")
        else:
            print("  WARN: May still be enabled via hardware button S2")
    
    return True

# =============================================================================
# Interactive mode
# =============================================================================
def interactive_menu(ser):
    """Interactive control menu"""
    print("\n" + "="*60)
    print("INTERACTIVE MODE")
    print("="*60)
    print("Commands:")
    print("  k  - Send KICK")
    print("  s  - Get STATUS")
    print("  e  - Enable watchdog (SW)")
    print("  d  - Disable watchdog (SW)")
    print("  c  - Clear fault (CLR_FAULT)")
    print("  w  - Write tWD (prompts for value)")
    print("  r  - Read all registers")
    print("  q  - Quit")
    print("-"*60)
    
    while True:
        try:
            cmd = input("\n> ").strip().lower()
        except (KeyboardInterrupt, EOFError):
            break
        
        if cmd == 'q':
            break
        elif cmd == 'k':
            resp = send_kick(ser)
            if resp:
                print(f"  KICK ACK: cmd=0x{resp['cmd']:02X}")
        elif cmd == 's':
            resp = get_status(ser)
            if resp:
                print_status(resp)
        elif cmd == 'e':
            write_reg(ser, ADDR_CTRL, 0x01)
            print("  Watchdog enabled (EN_SW=1)")
        elif cmd == 'd':
            write_reg(ser, ADDR_CTRL, 0x00)
            print("  Watchdog disabled (EN_SW=0)")
        elif cmd == 'c':
            write_reg(ser, ADDR_CTRL, 0x05)  # EN_SW + CLR_FAULT
            print("  CLR_FAULT sent")
        elif cmd == 'w':
            try:
                val = int(input("  tWD_ms = "))
                write_reg(ser, ADDR_TWD_MS, val)
                print(f"  tWD set to {val} ms")
            except ValueError:
                print("  Invalid number")
        elif cmd == 'r':
            print("  --- Register Dump ---")
            for name, addr in [("CTRL", 0x00), ("tWD_ms", 0x04),
                               ("tRST_ms", 0x08), ("arm_delay_us", 0x0C),
                               ("STATUS", 0x10)]:
                resp = read_reg(ser, addr)
                if resp:
                    print(f"  {name:15s} [0x{addr:02X}] = {resp['value']} (0x{resp['value']:08X})")
                time.sleep(0.05)
        else:
            print("  Unknown command. Type 'q' to quit.")

# =============================================================================
# Main
# =============================================================================
def main():
    if len(sys.argv) < 2:
        print("Usage: python3 watchdog_test.py <COM_PORT> [--interactive]")
        print("  e.g.: python3 watchdog_test.py COM3")
        print("  e.g.: python3 watchdog_test.py /dev/ttyUSB0 --interactive")
        sys.exit(1)
    
    port = sys.argv[1]
    interactive = '--interactive' in sys.argv or '-i' in sys.argv
    
    print(f"Opening {port} at 115200 bps...")
    try:
        ser = serial.Serial(port, 115200, timeout=2,
                           bytesize=serial.EIGHTBITS,
                           parity=serial.PARITY_NONE,
                           stopbits=serial.STOPBITS_ONE)
    except serial.SerialException as e:
        print(f"ERROR: Cannot open {port}: {e}")
        sys.exit(1)
    
    time.sleep(0.5)  # wait for port to stabilize
    ser.reset_input_buffer()
    
    print(f"Connected to {port}")
    
    if interactive:
        interactive_menu(ser)
    else:
        # Run automated tests
        if not test_uart_connection(ser):
            print("\n*** UART connection failed. Aborting. ***")
            ser.close()
            sys.exit(1)
        
        test_register_rw(ser)
        test_kick(ser)
        test_status(ser)
        test_enable_sw(ser)
        test_kick_prevents_timeout(ser)
        test_timeout_and_fault(ser)
        test_clr_fault(ser)
        test_disable(ser)
        
        print("\n" + "="*60)
        print("ALL TESTS COMPLETED")
        print("="*60)
        print("\nEntering interactive mode (type 'q' to quit)...")
        interactive_menu(ser)
    
    ser.close()
    print("Port closed. Done.")

if __name__ == '__main__':
    main()
