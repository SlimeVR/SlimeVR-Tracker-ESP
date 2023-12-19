import subprocess
import os

revision = ""

env_rev = os.environ.get("GIT_REV")
if not env_rev is None and env_rev != "":
    revision = env_rev
else:
    try:
        revision = (
            subprocess.check_output(["git", "rev-parse", "HEAD"])
            .strip()
            .decode("utf-8")
        )
    except Exception:
        revision = "NOT_GIT"

print(f"'-DGIT_REV=\"{revision}\"'")
