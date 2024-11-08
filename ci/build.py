import json
import os
import sys
import shutil
from enum import Enum
from textwrap import dedent
from typing import List
import configparser

COLOR_ESC = '\033['
COLOR_RESET = f'{COLOR_ESC}0m'
COLOR_GREEN = f'{COLOR_ESC}32m'
COLOR_RED = f'{COLOR_ESC}31m'
COLOR_CYAN = f'{COLOR_ESC}36m'
COLOR_GRAY = f'{COLOR_ESC}30;1m'


class DeviceConfiguration:
    def __init__(self,  platform: str, board: str, platformio_board: str) -> None:
        self.platform = platform
        self.board = board
        self.platformio_board = platformio_board

    def filename(self) -> str:
        return f"{self.board}-firmware.bin"

    def __str__(self) -> str:
        return f"{self.platform}@{self.board}"


def get_matrix() -> List[DeviceConfiguration]:
    matrix: List[DeviceConfiguration] = []

    config = configparser.ConfigParser()
    config.read("./platformio-tools.ini")
    for section in config.sections():
        if section == "env":
            continue

        board = section.split(":")[1]
        platform = config[section]["platform"]
        platformio_board = config[section]["board"]

        matrix.append(DeviceConfiguration(
            platform,
            board,
            platformio_board))

    return matrix


def prepare() -> None:
    print(f"🡢 {COLOR_CYAN}Preparation{COLOR_RESET}")

    print(f"  🡢 {COLOR_GRAY}Backing up platformio.ini{COLOR_RESET}")
    shutil.copy("./platformio.ini", "platformio.ini.bak")

    print(
        f"  🡢 {COLOR_GRAY}Switching platformio.ini to platformio-tools.ini{COLOR_RESET}")
    shutil.copy("./platformio-tools.ini", "platformio.ini")

    if os.path.exists("./build"):
        print(f"  🡢 {COLOR_GRAY}Removing existing build folder...{COLOR_RESET}")
        shutil.rmtree("./build")

    print(f"  🡢 {COLOR_GRAY}Creating build folder...{COLOR_RESET}")
    os.mkdir("./build")

    print(f"  🡢 {COLOR_GREEN}Success!{COLOR_RESET}")


def cleanup() -> None:
    print(f"🡢 {COLOR_CYAN}Cleanup{COLOR_RESET}")

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

    for device in matrix:
        print(f"  🡢 {COLOR_CYAN}Building for {device.platform}{COLOR_RESET}")

        status = build_for_device(device)

        if not status:
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

    code = os.system(f"platformio run -e {device.board}")

    if code == 0:
        shutil.copy(f".pio/build/{device.board}/firmware.bin",
                    f"build/{device.filename()}")

        print(f"    🡢 {COLOR_GREEN}Success!{COLOR_RESET}")
    else:
        success = False

        print(f"    🡢 {COLOR_RED}Failed!{COLOR_RESET}")

    print("::endgroup::")

    return success


def main() -> None:
    prepare()
    code = build()
    cleanup()

    sys.exit(code)


if __name__ == "__main__":
    main()
