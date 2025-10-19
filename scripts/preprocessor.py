import json
import re
import os
from pathlib import Path
from typing import Union, Optional, Dict, Any, List

Import("env")

try:
	import jsonschema
except:
    env.Execute(
        env.VerboseAction(
            '$PYTHONEXE -m pip install "jsonschema==4.22.0"',
            "Installing jsonschema for validation",
        )
    )

from jsonschema import Draft202012Validator, exceptions as jsonschema_exceptions

def _load_json(maybe_path_or_dict: Union[str, Path, dict]) -> dict:
    """Load JSON file or accept dict directly."""
    if isinstance(maybe_path_or_dict, dict):
        return maybe_path_or_dict
    p = Path(maybe_path_or_dict)
    if not p.exists():
        raise FileNotFoundError(f"File not found: {p}")
    try:
        return json.loads(p.read_text(encoding="utf-8"))
    except json.JSONDecodeError as e:
        raise ValueError(f"Invalid JSON file {p}: {e}")


def _format_raw_value(value: Any) -> str:
    """Format booleans for c/cpp, otherwise str(value)."""
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def _build_board_flags(defaults: dict, board_name: str) -> List[str]:
    """Construct list of -D flags for one board."""
    if "defaults" not in defaults:
        raise ValueError("Missing top-level 'defaults' key in defaults JSON.")
    if board_name not in defaults["defaults"]:
        raise ValueError(f"Invalid board selected - {board_name}")

    board_defaults = defaults["defaults"][board_name]
    values = board_defaults.get("values", {})

    args: Dict[str, Dict[str, Any]] = {}

    def add(key: str, value: Any, value_type: str):
        if value is not None:
            args[key] = {"value": value, "type": value_type}

    add("BOARD", board_name, "raw")
    add("LED_PIN", values.get("LED").get("LED_PIN"), "pin")
    add("LED_INVERTED", values.get("LED").get("LED_INVERTED"), "raw")

    sensors = values.get("SENSORS")
    if sensors:
        for index, sensor in enumerate(sensors):
            if index == 0:
                add("IMU", sensor.get("imu"), "raw")
                add("PIN_IMU_INT", sensor.get("int"), "pin")
                add("IMU_ROTATION", sensor.get("rotation"), "raw")
                if sensor.get("protocol") == "I2C":
                    add("PRIMARY_IMU_ADDRESS_ONE", sensor.get("address"), "number")
            if index == 1:
                add("SECOND_IMU", sensor.get("imu"), "raw")
                add("PIN_IMU_INT_2", sensor.get("int"), "pin")
                add("SECOND_IMU_ROTATION", sensor.get("rotation"), "raw")
                if sensor.get("protocol") == "I2C":
                    add("SECONDARY_IMU_ADDRESS_TWO", sensor.get("address"), "number")

            if sensor.get("protocol") == "I2C":
                add("PIN_IMU_SDA", sensor.get("sda"), "pin")
                add("PIN_IMU_SCL", sensor.get("scl"), "pin")

    battery = values.get("BATTERY")
    if battery:
        add("BATTERY_MONITOR", battery.get("type"), "raw")
        add("PIN_BATTERY_LEVEL", battery.get("pin"), "pin")
        add("BATTERY_SHIELD_RESISTANCE", battery.get("shieldR"), "number")
        add("BATTERY_SHIELD_R1", battery.get("r1"), "number")
        add("BATTERY_SHIELD_R2", battery.get("r2"), "number")

    parts: List[str] = []
    for key, meta in args.items():
        val = meta["value"]
        typ = meta["type"]

        if typ == "pin":
            if isinstance(val, str) and re.search(r"[AD]", val):
                parts.append(f"-D{key}='{val}'")
            else:
                parts.append(f"-D{key}={_format_raw_value(val)}")
        elif typ == "string":
            parts.append(f"-D{key}='{val}'")
        elif typ in ("raw", "number"):
            parts.append(f"-D{key}={_format_raw_value(val)}")

    return parts


def build_boards(
    schema_obj,
    defaults_obj,
    board_name: Optional[str] = None,
) -> Dict[str, List[str]]:
    """
    Validate defaults.json against board-defaults.schema.json using jsonschema,
    and return { board_name: [list of -D flags] }.
    """
    validator = Draft202012Validator(schema_obj)
    errors = sorted(validator.iter_errors(defaults_obj), key=lambda e: e.path)

    if errors:
        print("✖ JSON Schema validation failed:")
        for err in errors:
            path = "/".join(map(str, err.path)) or "(root)"
            print(f"  • Path: {path}")
            print(f"    Error: {err.message}")
            if err.context:
                for ctx in err.context:
                    print(f"      ↳ {ctx.message}")
        raise ValueError(f"{len(errors)} schema validation errors found.")

    out: Dict[str, List[str]] = {}
    if board_name:
        out[board_name] = _build_board_flags(defaults_obj, board_name)
    else:
        for name in defaults_obj.get("defaults", {}).keys():
            out[name] = _build_board_flags(defaults_obj, name)

    return out

schema_obj = _load_json("./board-defaults.schema.json")
defaults_obj = _load_json("./board-defaults.json")
slime_board = env.GetProjectOption("custom_slime_board") or "BOARD_CUSTOM"

if 'SLIMEVR_OVERRIDE_DEFAULTS' in os.environ and slime_board in defaults_obj['defaults']:
	print(">>> OVERIDING BOARD DEFAULTS ", os.environ['SLIMEVR_OVERRIDE_DEFAULTS'])
	defaults_obj['defaults'][slime_board]['values'] = json.loads(os.environ['SLIMEVR_OVERRIDE_DEFAULTS'])

output_flags = build_boards(
	schema_obj,
	defaults_obj,
	slime_board,
)
output_flags = output_flags.get(slime_board, []) if isinstance(output_flags, dict) else []

print(">>> Appending build flags:", output_flags)
env.Append(BUILD_FLAGS=output_flags)
