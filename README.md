# TPS3431-like Watchdog Monitor — FPGA Extended Contest 2026

## Overview

RTL implementation of a TPS3431-like watchdog supervisor on the **Kiwi 1P5** board (Gowin GW1N-UV1P5QN48XF, 27 MHz crystal). Supports runtime parameter configuration via UART (115200 bps, 8N1). All functionality verified on hardware.

## Architecture

```
 S1 (WDI) ──► sync_debounce ──┐
                               ├──► watchdog_core ──► LED D3 (WDO)
 S2 (EN)  ──► sync_debounce ──┤                  └──► LED D4 (ENOUT)
                               │
 UART RXD ──► uart_rx ──► uart_frame_parser ──► regfile ──┘
                               │
 UART TXD ◄── uart_tx ◄───────┘
```

### Module Summary

| Module              | Description                                                    |
|---------------------|----------------------------------------------------------------|
| `watchdog_top`      | Top-level integration, power-on reset                          |
| `sync_debounce`     | 2-FF metastability sync + counter-based debounce (15 ms)       |
| `watchdog_core`     | FSM (DISABLED/ARMING/ACTIVE/FAULT) + us/ms tick generators     |
| `regfile`           | Configuration registers (CTRL, tWD, tRST, arm_delay, STATUS)  |
| `uart_rx`           | UART receiver, 115200 8N1, 2-FF input sync                    |
| `uart_tx`           | UART transmitter, 115200 8N1                                   |
| `uart_frame_parser` | Frame protocol decode/encode, XOR checksum, register R/W       |

## Watchdog FSM States

| State        | Description                                                          |
|--------------|----------------------------------------------------------------------|
| **DISABLED** | EN=0. WDO released (high), ENOUT=0. WDI ignored.                    |
| **ARMING**   | EN just went high. Counting `arm_delay_us`. WDI kicks ignored.      |
| **ACTIVE**   | Watchdog running. Each WDI falling edge resets tWD counter.          |
| **FAULT**    | tWD expired. WDO asserted (low) for `tRST_ms`, then back to ACTIVE. |

## Default Parameters

| Parameter     | Default | Description                           |
|---------------|---------|---------------------------------------|
| tWD_ms        | 1600    | Watchdog timeout (ms) — emulates CWD=NC |
| tRST_ms       | 200     | WDO hold time during fault (ms)       |
| arm_delay_us  | 150     | WDI ignore window after enable (us)   |

## Pin Mapping (Verified on Hardware)

| Function   | Board Resource    | Gowin IO       | Pin | Notes                        |
|------------|-------------------|----------------|-----|------------------------------|
| Clock      | Crystal X3 27MHz  | IOL6A (GCLKT_7)| 4   | On-board 27 MHz oscillator   |
| WDI (kick) | Button S1         | IOR1B (KEY1)   | 35  | Active-low, 15 ms debounce   |
| EN         | Button S2         | IOR1A (KEY2)   | 36  | Active-low, hold to enable   |
| WDO        | LED D3            | IOR17A         | 27  | **LED ON = fault active**    |
| ENOUT      | LED D4            | IOR15B         | 28  | **LED ON = watchdog armed**  |
| UART RXD   | GWU2U USB-UART    | IOR11B         | 33  | FPGA receives from PC        |
| UART TXD   | GWU2U USB-UART    | IOR11A         | 34  | FPGA transmits to PC         |

## LED Convention (Active-HIGH)

Board schematic: `FPGA pin -> R330 -> LED -> GND`. Current flows when FPGA drives HIGH.

- **D3 (WDO):** LED ON when fault is active (WDO asserted)
- **D4 (ENOUT):** LED ON when watchdog is armed and running

## Open-Drain Emulation

**Approach B (push-pull)** is used. WDO and ENOUT use standard push-pull FPGA outputs with active-low convention for WDO internally. LED output is inverted to match the active-HIGH LED circuit on board.

## UART Protocol

**Baud rate:** 115200, 8N1

### Frame Format

```
TX (PC -> FPGA): [0x55][CMD][ADDR][LEN][DATA_0..DATA_N-1][CHK]
RX (FPGA -> PC): [0x55][CMD|0x80][ADDR][LEN][DATA...][CHK]

CHK = XOR of all bytes from CMD through last DATA byte
```

### Commands

| CMD  | Name       | Description                               |
|------|------------|-------------------------------------------|
| 0x01 | WRITE_REG  | Write register. Response: ACK (0x81)      |
| 0x02 | READ_REG   | Read register. Response: data (0x82)      |
| 0x03 | KICK       | Generate one WDI kick. Response: ACK (0x83)|
| 0x04 | GET_STATUS | Quick status read. Response: STATUS (0x84) |

### Register Map

| Addr | Name         | R/W | Width | Description                                          |
|------|--------------|-----|-------|------------------------------------------------------|
| 0x00 | CTRL         | R/W | 32    | bit0: EN_SW, bit1: WDI_SRC, bit2: CLR_FAULT (W1C)   |
| 0x04 | tWD_ms       | R/W | 32    | Watchdog timeout in ms                               |
| 0x08 | tRST_ms      | R/W | 32    | WDO hold time in ms                                  |
| 0x0C | arm_delay_us | R/W | 16    | Arm delay in us                                      |
| 0x10 | STATUS       | R   | 32    | bit0: EN_EFFECTIVE, bit1: FAULT_ACTIVE, bit2: ENOUT, bit3: WDO, bit4: LAST_KICK_SRC |

### Example: Write tWD = 500 ms

```
PC sends:   55 01 04 04 F4 01 00 00 F0
            |  |  |  |  └─data (500=0x1F4, LE)─┘  |
            |  |  |  └─ LEN=4                      |
            |  |  └─ ADDR=0x04                     |
            |  └─ CMD=WRITE_REG                    |
            └─ SYNC                          CHK (XOR)

FPGA resp:  55 81 04 00 85
```

## Enable Sources

The watchdog can be enabled by either:
- **Hardware:** Holding button S2 (active-low -> en=1 while pressed)
- **Software:** Setting CTRL bit0 (EN_SW) via UART

Both sources are OR'd: `en = en_button | en_sw`.

## How to Build

1. Open **Gowin FPGA Designer** (GOWIN EDA)
2. Create project for **GW1N-UV1P5QN48XFC7/I6**
3. Add all `.v` files from `rtl/` directory (7 files)
4. Add `constraints/kiwi_1p5.cst` as the Physical Constraints File
5. **Do NOT add** testbench files (`sim/*.v`) to the project
6. In Project -> Configuration -> Dual-Purpose Pin: enable **"Use JTAG as regular IO"** if UART pins conflict
7. Synthesize -> Place & Route -> Generate bitstream
8. Program via **Gowin Programmer** (SRAM Program or embFlash Program)

## How to Run Demo

### Button Demo (no PC needed)
1. After programming: both LEDs D3 and D4 are OFF
2. **Hold S2** (EN) -> LED D4 turns ON after arm delay (~150 us)
3. Continue holding S2, do NOT press S1 -> after ~1.6s, LED D3 turns ON (timeout fault)
4. Continue holding S2, press S1 repeatedly (< 1.6s intervals) -> LED D3 stays OFF (kick resets timer)
5. Release S2 -> both LEDs turn OFF (watchdog disabled)

### UART Demo (with PC)
1. Connect USB cable to **UART port** (GWU2U, not JTAG port)
2. Install driver if needed (see GWU2U Driver section below)
3. Install Python + pyserial: `pip install pyserial`
4. Run test script:
```bash
python watchdog_test.py COM11          # Windows (adjust COM port)
python watchdog_test.py /dev/ttyUSB0   # Linux
```
5. Interactive mode commands: `k`=kick, `s`=status, `e`=enable, `d`=disable, `r`=read all, `q`=quit

### GWU2U Driver Installation
1. Navigate to: `<Gowin_Install>/Programmer/driver/`
2. Run `GowinUSBCableDriverV5_for_win7+.exe` as Administrator
3. If GWU2U still shows under "Other devices" in Device Manager:
   - Right-click GWU2U -> Update driver -> Browse -> point to Gowin Programmer folder
   - Or use **Zadig** tool: select GWU2U, install USB Serial (CDC) driver

## Simulation

```bash
# Using Icarus Verilog (pure Verilog-2005 compatible)
iverilog -g2005 -o sim.vvp rtl/*.v sim/tb_watchdog_system.v
vvp sim.vvp

# Using ModelSim
vsim work.tb_watchdog_system
run -all
```

### Testbench Coverage

| Test | Description                                  | Status  |
|------|----------------------------------------------|---------|
| 1    | UART WRITE_REG: set tWD, tRST, arm_delay    | PASSED  |
| 2    | UART READ_REG: verify register content       | PASSED  |
| 3    | Enable via button + arm_delay verification   | PASSED  |
| 4    | Normal kick (button + UART KICK) resets timer| PASSED  |
| 5    | Timeout -> FAULT -> auto-recovery after tRST | PASSED  |
| 6    | Disable watchdog: WDI ignored, no timeout    | PASSED  |
| 7    | CLR_FAULT: clear fault immediately via UART  | PASSED  |

### Hardware Test Results

All 9 automated tests passed on Kiwi 1P5 board via `watchdog_test.py`:
- UART connection, register R/W, KICK, GET_STATUS
- Software enable/disable, periodic kick, timeout/fault detection
- CLR_FAULT immediate clear

## File Structure

```
FPGA/
├── fpga_code/
│   ├── src/
│   │   ├── gowin_osc/                # Gowin IP Core (OSC - generated)
│   │   │   └── gowin_osc.v           # Internal oscillator wrapper
│   │   ├── watchdog_top.v            # Top-level (27 MHz crystal clock)
│   │   ├── watchdog_core.v           # Watchdog FSM + timers
│   │   ├── sync_debounce.v           # Button synchronizer + debounce
│   │   ├── regfile.v                 # Configuration registers
│   │   ├── uart_rx.v                 # UART receiver
│   │   ├── uart_tx.v                 # UART transmitter
│   │   ├── uart_frame_parser.v       # Frame protocol engine
│   │   ├── tb_watchdog_core.v        # Testbench: watchdog core unit test
│   │   ├── tb_watchdog_system.v      # Testbench: full system with UART
│   │   └── kiwi_1p5.cst              # Pin constraints (verified on HW)
│   ├── impl/
│   │   ├── gwsynthesis/              # Synthesis output
│   │   ├── pnr/                      # Place & Route output + bitstream
│   │   └── temp/                     # Temporary build files
│   ├── fpga_project.gprj             # Gowin FPGA Designer project file
├── python_code/
│   └── watchdog_test.py              # Python UART test + interactive tool
└── README.md                         # Project documentation
```

## Language

All RTL code is **pure Verilog-2001/2005** (no SystemVerilog constructs). Compatible with Gowin EDA, Icarus Verilog, and ModelSim.
