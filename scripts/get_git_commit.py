import subprocess

revision = "00000000000000000000000000000000000000000"

try:
    revision = (
        subprocess.check_output(["git", "rev-parse", "HEAD"])
        .strip()
        .decode("utf-8")
    )
except:
    pass

print(f"'-DGIT_REV=\"{revision}\"'")
