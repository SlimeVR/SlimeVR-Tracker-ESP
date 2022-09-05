import json
import os
import shutil
from enum import Enum
from textwrap import dedent
from typing import List

COLOR_ESC = '\033['
COLOR_RESET = f'{COLOR_ESC}0m'
COLOR_GREEN = f'{COLOR_ESC}32m'
COLOR_RED = f'{COLOR_ESC}31m'
COLOR_CYAN = f'{COLOR_ESC}36m'
COLOR_GRAY = f'{COLOR_ESC}30;1m'


class Board(Enum):
    SLIMEVR = "BOARD_SLIMEVR"
    WROOM32 = "BOARD_WROOM32"
    LOLIN_C3_MINI = "BOARD_LOLIN_C3_MINI"
    ES32C3DEVKITM1 = "BOARD_ES32C3DEVKITM1"


class DeviceConfiguration:
    def __init__(self,  platform: str, board: Board, platformio_board: str) -> None:
        self.platform = platform
        self.board = board
        self.platformio_board = platformio_board

    def get_platformio_section(self) -> str:
        section = f"[env:{self.platformio_board}]\n"
        section += f"platform = {self.platform}\n"
        section += f"board = {self.platformio_board}\n"

        if self.board == Board.LOLIN_C3_MINI:
            section += "build_flags = \n"
            section += " ${env.build_flags}\n"
            section += " -DESP32C3\n"

        if self.board == Board.ES32C3DEVKITM1:
            section += "build_flags = \n"
            section += " ${env.build_flags}\n"
            section += " -DESP32C3\n"

        return section

    def filename(self) -> str:
        return f"{self.platformio_board}.bin"

    def build_header(self) -> str:
        sda = ""
        scl = ""
        imu_int = ""
        imu_int2 = ""
        battery_level = ""
        led_pin = 2
        led_invert = False

        if self.board == Board.SLIMEVR:
            sda = "4"
            scl = "5"
            imu_int = "10"
            imu_int2 = "13"
            battery_level = "17"
            led_invert = True
        elif self.board == Board.WROOM32:
            sda = "21"
            scl = "22"
            imu_int = "23"
            imu_int2 = "25"
            battery_level = "36"
        elif self.board == Board.LOLIN_C3_MINI:
            sda = "5"
            scl = "4"
            imu_int = "6"
            imu_int2 = "8"
            battery_level = "3"
        elif self.board == Board.ES32C3DEVKITM1:
            sda = "5"
            scl = "4"
            imu_int = "6"
            imu_int2 = "7"
            battery_level = "3"
        else:
            raise Exception(f"Unknown board: {self.board.value}")

        return f"""
#define IMU IMU_BNO085
#define SECOND_IMU IMU
#define BOARD {self.board.value}
#define IMU_ROTATION DEG_90
#define SECOND_IMU_ROTATION DEG_90

#define BATTERY_MONITOR BAT_EXTERNAL
#define BATTERY_SHIELD_RESISTANCE 180

#define PIN_IMU_SDA {sda}
#define PIN_IMU_SCL {scl}
#define PIN_IMU_INT {imu_int}
#define PIN_IMU_INT_2 {imu_int2}
#define PIN_BATTERY_LEVEL {battery_level}
#define LED_PIN {led_pin}
#define LED_INVERTED {led_invert.__str__().lower()}
"""

    def __str__(self) -> str:
        return f"{self.platform}@{self.board.value}"


def get_matrix() -> List[DeviceConfiguration]:
    matrix: List[DeviceConfiguration] = []

    configFile = open("./ci/devices.json", "r")
    config = json.load(configFile)

    for deviceConfig in config:
        matrix.append(DeviceConfiguration(
            deviceConfig["platform"], Board[deviceConfig["board"]], deviceConfig["platformio_board"]))

    return matrix


def prepare() -> None:
    print(f"🡢 {COLOR_CYAN}Preparation{COLOR_RESET}")

    print(f"  🡢 {COLOR_GRAY}Backing up src/defines.h{COLOR_RESET}")
    shutil.copy("src/defines.h", "src/defines.h.bak")

    print(f"  🡢 {COLOR_GRAY}Backing up platformio.ini{COLOR_RESET}")
    shutil.copy("./platformio.ini", "platformio.ini.bak")

    print(f"  🡢 {COLOR_GRAY}Copying over build/platformio.ini{COLOR_RESET}")
    shutil.copy("./ci/platformio.ini", "platformio.ini")

    if os.path.exists("./build"):
        print(f"  🡢 {COLOR_GRAY}Removing existing build folder...{COLOR_RESET}")
        shutil.rmtree("./build")

    print(f"  🡢 {COLOR_GRAY}Creating build folder...{COLOR_RESET}")
    os.mkdir("./build")

    print(f"  🡢 {COLOR_GREEN}Success!{COLOR_RESET}")


def cleanup() -> None:
    print(f"🡢 {COLOR_CYAN}Cleanup{COLOR_RESET}")

    print(f"  🡢 {COLOR_GRAY}Restoring src/defines.h...{COLOR_RESET}")
    shutil.copy("src/defines.h.bak", "src/defines.h")

    print(f"  🡢 {COLOR_GRAY}Removing src/defines.h.bak...{COLOR_RESET}")
    os.remove("src/defines.h.bak")

    print(f"  🡢 {COLOR_GRAY}Restoring platformio.ini...{COLOR_RESET}")
    shutil.copy("platformio.ini.bak", "platformio.ini")

    print(f"  🡢 {COLOR_GRAY}Removing platformio.ini.bak...{COLOR_RESET}")
    os.remove("platformio.ini.bak")

    print(f"  🡢 {COLOR_GREEN}Success!{COLOR_RESET}")


def build() -> int:
    print(f"🡢 {COLOR_CYAN}Build{COLOR_RESET}")

    failed_builds: List[str] = []
    code = 0

    matrix = get_matrix()

    with open("./platformio.ini", "a") as f1:
        for device in matrix:
            f1.write(device.get_platformio_section())

    for device in matrix:
        print(f"  🡢 {COLOR_CYAN}Building for {device.platform}{COLOR_RESET}")

        status = build_for_device(device)

        if status == False:
            failed_builds.append(device.platformio_board)

    if len(failed_builds) > 0:
        print(f"  🡢 {COLOR_RED}Failed!{COLOR_RESET}")

        for failed_build in failed_builds:
            print(f"    🡢 {COLOR_RED}{failed_build}{COLOR_RESET}")

        code = 1
    else:
        print(f"  🡢 {COLOR_GREEN}Success!{COLOR_RESET}")

    return code


def build_for_device(device: DeviceConfiguration) -> bool:
    success = True

    print(f"::group::Build {device}")

    with open("src/defines.h", "wt") as f:
        f.write(device.build_header())

    code = os.system(
        f"platformio run -e {device.platformio_board}")

    if code == 0:
        shutil.copy(f".pio/build/{device.platformio_board}/firmware.bin",
                    f"build/{device.filename()}")

        print(f"    🡢 {COLOR_GREEN}Success!{COLOR_RESET}")
    else:
        success = False

        print(f"    🡢 {COLOR_RED}Failed!{COLOR_RESET}")

    print(f"::endgroup::")

    return success


def main() -> None:
    prepare()
    code = build()
    cleanup()

    os._exit(code)


if __name__ == "__main__":
    main()
