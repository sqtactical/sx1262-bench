"""Build sx1262_modem and flash it to every connected board without touching the reset button.

Boards running sx1262_modem get the "DFU" command (UF2 bootloader); boards that are already
in the bootloader just get the UF2 copied onto their NICENANO drive.
Usage: python flash.py [--no-build | --uf2 <image.uf2>] [--meshtastic]
  --meshtastic: the boards run Meshtastic, enter DFU with the CLI's --enter-dfu instead of our DFU command
"""
import os, shutil, string, subprocess, sys, time
import serial
from serial.tools import list_ports

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.join(os.environ["LOCALAPPDATA"], "flrc-tools")
CORE = os.path.join(os.environ["LOCALAPPDATA"], "Arduino15", "packages", "adafruit", "hardware", "nrf52", "1.7.0")
BUILD = os.path.join(TOOLS, "build1262")
UF2 = os.path.join(BUILD, "sx1262_modem.uf2")
BOOTLOADER_PID = 0x00B3   # nice!nano UF2 bootloader (Meshtastic on these boards keeps it); the modem enumerates as 0x80DA


def build():
    subprocess.check_call([os.path.join(TOOLS, "arduino-cli.exe"), "compile", "-b", "adafruit:nrf52:pca10056",
                           "--output-dir", BUILD, os.path.join(HERE, "..", "firmware", "sx1262_modem")])
    subprocess.check_call([sys.executable, os.path.join(CORE, "tools", "uf2conv", "uf2conv.py"), "-f", "0xADA52840",
                           "-c", "-o", UF2, os.path.join(BUILD, "sx1262_modem.ino.hex")])


def uf2_drives():
    out = []
    for d in string.ascii_uppercase:
        if os.path.exists(f"{d}:\\INFO_UF2.TXT"):
            out.append(f"{d}:\\")
    return out


def board_ports():
    return [p.device for p in list_ports.comports() if p.vid == 0x239A]


def to_bootloader(port):
    try:
        s = serial.Serial(port, 115200, timeout=1)
        s.reset_input_buffer()
        s.write(b"\nDFU\n")
    except Exception as e:
        print(f"{port}: DFU command failed ({e})")
        return
    try:
        s.close()  # the board usually vanishes first, which makes close() raise
    except Exception:
        pass
    print(f"{port}: DFU sent")


def main():
    global UF2
    if "--uf2" in sys.argv:  # flash a prebuilt image instead (e.g. Meshtastic)
        UF2 = sys.argv[sys.argv.index("--uf2") + 1]
    elif "--no-build" not in sys.argv:
        build()
    nboards = len(board_ports())
    if "--meshtastic" in sys.argv:  # boards currently running Meshtastic: ask for DFU through the admin API
        cli = os.path.join(TOOLS, "venv", "Scripts", "meshtastic.exe")
        for p in board_ports():
            subprocess.call([cli, "--port", p, "--enter-dfu"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            print(f"{p}: --enter-dfu sent")
    else:
        for p in [p.device for p in list_ports.comports() if p.vid == 0x239A and p.pid != BOOTLOADER_PID]:
            to_bootloader(p)
    t = time.time()
    while time.time() - t < 15 and len(uf2_drives()) < nboards:
        time.sleep(0.5)
    time.sleep(1)
    drives = uf2_drives()
    if not drives:
        sys.exit("no UF2 drive appeared - double-tap RESET on the boards")
    for d in drives:
        print(f"copy -> {d}")
        try:
            shutil.copyfile(UF2, os.path.join(d, "firmware.uf2"))
        except OSError:
            pass  # the bootloader resets as soon as the last block lands, which can abort the close
    time.sleep(4)
    print("ports now:", board_ports())


if __name__ == "__main__":
    main()
