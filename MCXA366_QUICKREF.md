# MCXA366 Development Quick Reference

Working notes for the FRDM-MCXA366 Klipper port.

## Paths

Klipper:

    ~/workspace/klipper

MCUXpresso SDK:

    ~/workspace/nxp/mcuxsdk/mcuxsdk

Klipper branch:

    mcxa366-port

Target:

    MCXA366:FRDM-MCXA366

MCU-Link probe serial:

    UHOJY12VUWBBV

Klipper UART:

    /dev/serial/by-id/usb-NXP_Semiconductors_MCU-LINK_FRDM-MCXA366__r2E4__CMSIS-DAP_V3.155_UHOJY12VUWBBV-if02


# NXP SDK / west

## Enter SDK

    cd ~/workspace/nxp/mcuxsdk/mcuxsdk

## See supported hello_world configurations

    west list_project -p examples/demo_apps/hello_world

## Build hello_world for FRDM-MCXA366

    west build -b frdmmcxa366 examples/demo_apps/hello_world

## Force a clean west build

    west build -b frdmmcxa366 examples/demo_apps/hello_world -p always

## Flash current west build with LinkServer

    west flash -r linkserver

## Start west/LinkServer debugger

    west debug -r linkserver


# Klipper

## Enter Klipper tree

    cd ~/workspace/klipper

## Check branch/status

    git status
    git branch --show-current

Expected branch:

    mcxa366-port

## Configure

    make menuconfig

Current important configuration:

    MCU: MCXA366
    CLOCK_FREQ: 240000000
    Communication: UART
    UART: LPUART2
    Baud: 250000

Pins:

    P2_2 = LPUART2 TX
    P2_3 = LPUART2 RX

## Build

    make clean && make

Firmware outputs are under:

    out/

Important ELF:

    out/klipper.elf

## Check formatting before committing

    git diff --check


# Flash Klipper with LinkServer

From the Klipper repo:

    cd ~/workspace/klipper

Flash the ELF:

    LinkServer flash MCXA366:FRDM-MCXA366 load out/klipper.elf

If multiple probes are connected, use the MCU-Link probe explicitly:

    LinkServer flash \
        --probe UHOJY12VUWBBV \
        MCXA366:FRDM-MCXA366 \
        load out/klipper.elf


# LinkServer GDB server

## Normal debug connection

    LinkServer gdbserver \
        --probe UHOJY12VUWBBV \
        MCXA366:FRDM-MCXA366

## Attach without resetting/restarting firmware

    LinkServer gdbserver \
        --attach \
        --probe UHOJY12VUWBBV \
        MCXA366:FRDM-MCXA366

MCU-Link currently reports:

    NXP MCU-Link
    firmware 3.155


# GDB

In another terminal after starting LinkServer:

    cd ~/workspace/klipper

    arm-none-eabi-gdb out/klipper.elf

Inside GDB:

    target remote localhost:3333

Useful commands:

    info registers
    bt
    x/10i $pc
    disassemble ResetHandler
    continue

ResetHandler address observed during bring-up:

    0x00000184

Do not manually set PC to ResetHandler while stopped in Handler/fault
state. A forced PC reset is only appropriate from a known-clean
Thread-mode state.


# Minimal Klippy test configuration

printer.cfg:

    [mcu]
    serial: /dev/serial/by-id/usb-NXP_Semiconductors_MCU-LINK_FRDM-MCXA366__r2E4__CMSIS-DAP_V3.155_UHOJY12VUWBBV-if02

    [printer]
    kinematics: none
    max_velocity: 1
    max_accel: 1


# Run Klippy manually

From ~/workspace/klipper:

    rm -f /tmp/klippy.log

    python3 klippy/klippy.py printer.cfg -l /tmp/klippy.log

Watch the log from another terminal:

    tail -F /tmp/klippy.log

Show recent clock statistics:

    grep 'Stats ' /tmp/klippy.log | tail -n 20

Look for MCU connection/configuration:

    grep -E \
        'Loaded MCU|MCU .* config:|Configured MCU|configured for.*Mhz' \
        /tmp/klippy.log


# Current clock architecture

Klipper configuration:

    CLOCK_FREQ = 240000000

Current MCU clock path:

    IFR1 factory 240 MHz trim
        -> FRO_HF
        -> MAIN_CLK

MAIN_CLK selectors used during bring-up:

    2 = FRO12M
    3 = FRO_HF
    6 = PLL1

Factory FRO_HF trims:

    IFR1 + 0x870 = 180 MHz trim
    IFR1 + 0x874 = 240 MHz trim

Current direct FRO_HF measurement from Klippy:

    ~241.7325 MHz

Nominal error:

    ~+0.72%

This is within Klippy's 1% clock sanity threshold.

Previous FRO12M -> PLL1 configuration measured:

    ~242.6376 MHz
    ~+1.10%

That exceeded Klippy's 1% sanity threshold.


# UART clock

LPUART2 functional clock:

    FRO12M
        -> FRO_LF_DIV / 1
        -> LPUART2

Nominal UART functional clock:

    12 MHz

UART baud:

    250000


# Useful USB checks

    lsusb

Expected devices include:

MCU-Link:

    1fc9:0143

DSLogic:

    2a0e:002a

USB-CAN adapter:

    1d50:606f

Find serial devices:

    ls -l /dev/serial/by-id/


# CAN

Show interface:

    ip -details link show can0

Bring can0 down:

    sudo ip link set can0 down

Configure Klipper-style 1 Mbit CAN:

    sudo ip link set can0 type can bitrate 1000000

Bring it up:

    sudo ip link set can0 up

Query Klipper CAN nodes:

    python3 scripts/canbus_query.py can0


# Current hardware/debug notes

FRDM-MCXA366 CPU:

    Cortex-M33

Usable RAM observed from factory vector table:

    0x20000000 - 0x2003bfff

Initial MSP:

    0x2003c000

Klipper flash size currently configured:

    0xFE000

Klipper RAM size:

    0x3C000

Klipper stack:

    2048 bytes

Current debugging caveat:

Some SCG/system registers cannot reliably be read through the LinkServer
debug path even while firmware can access them normally. A failed GDB
memory read of an SCG register is not by itself evidence that the clock
configuration failed.

The authoritative runtime clock check is Klippy clocksync.
