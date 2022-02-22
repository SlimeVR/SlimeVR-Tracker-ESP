import os
import re
import shutil
import subprocess
from textwrap import dedent


def build_for_platform(platform: str) -> int:
    shutil.copy("src/defines.h", "src/defines.h.bak")
    board = "BOARD_"

    if platform == "esp12e":
        board += "SLIMEVR"
    elif platform == "d1_mini_lite":
        board += "WEMOSD1MINI"
    elif platform == "esp01":
        board += "ESP01"
    elif platform == "esp32dev":
        board += "WROOM32"
    else:
        raise Exception("Unknown platform")

    with open("src/defines.h", "r") as f:
        lines = f.read().rstrip().lstrip()
        lines = re.sub("#define BOARD .*", f"#define BOARD {board}", lines)

        with open("src/defines.h", "wt") as f:
            f.write(lines)

    print("Building for platform: " + platform)
    ret_code = os.system(f"platformio run -e {platform}")

    if ret_code == 0:
        print("Build succeeded")

        shutil.copy(f".pio/build/{platform}/firmware.bin",
                    f"build/{platform}.bin")
    else:
        print("Build failed")

    shutil.copy("src/defines.h.bak", "src/defines.h")
    os.remove("src/defines.h.bak")

    return ret_code


def main() -> None:
    shutil.copy("platformio.ini", "platformio.ini.bak")

    if os.path.exists("./build"):
        shutil.rmtree("./build")

    os.mkdir("./build")

    with open("platformio.ini", "r") as f:
        lines = f.read().rstrip().lstrip()
        matches = re.search(
            "\[env:.*\]\nplatform = .*\nboard = .*", lines)

        if matches:
            # remove lines
            lines = lines.replace(matches.group(0), "")

        # add new lines
        lines += dedent("""
        ; ESP32 Dev Board
        [env:esp32dev]
        platform = espressif32
        board = esp32dev

        ; SlimeVR PCB
        [env:esp12e]
        platform = espressif8266
        board = esp12e

        ; ESP01 (512kB flash)
        [env:esp01]
        platform = espressif8266
        board = esp01

        ; WEMOS D1 mini lite (1MB flash)
        [env:d1_mini_lite]
        platform = espressif8266
        board = d1_mini_lite
        """)

        with open("platformio.ini", "wt") as f:
            f.write(lines)

    # TODO: Build a matrix of all platforms and all IMUs
    esp32_ret_code = build_for_platform("esp32dev")
    esp12e_ret_code = build_for_platform("esp12e")
    esp01_ret_code = build_for_platform("esp01")
    d1_mini_lite_ret_code = build_for_platform("d1_mini_lite")

    shutil.copy("platformio.ini.bak", "platformio.ini")
    os.remove("platformio.ini.bak")

    print("\n\n\n\n")

    if esp32_ret_code != 0:
        print("Error occurred while building for ESP32 Dev Board")

    if esp12e_ret_code != 0:
        print("Error occurred while building for SlimeVR PCB")

    if esp01_ret_code != 0:
        print("Error occurred while building for ESP01")

    if d1_mini_lite_ret_code != 0:
        print("Error occurred while building for WEMOS D1 mini lite")

    if esp32_ret_code != 0 or esp12e_ret_code != 0 or esp01_ret_code != 0 or d1_mini_lite_ret_code != 0:
        print("One or more builds failed")
        os._exit(1)


if __name__ == "__main__":
    main()
