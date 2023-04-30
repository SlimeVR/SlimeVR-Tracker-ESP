import subprocess
import os

revision = "00000000000000000000000000000000000000000"

env_rev = os.environ.get("GIT_REV")
if env_rev != "":
    revision = env_rev

try:
    revision = (
        subprocess.check_output(["git", "rev-parse", "HEAD"])
        .strip()
        .decode("utf-8")
    )
except Exception:
    print("==========================================================================")
    print("  WARNING: You are not in a Git repository or do not have Git installed.")
    print("           The version string will be incorrect.")
    print("==========================================================================")

print(f"'-DGIT_REV=\"{revision}\"'")
