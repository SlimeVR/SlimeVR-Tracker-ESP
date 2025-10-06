import subprocess, os
Import("env")

project_dir = env["PROJECT_DIR"]

if not os.path.isdir(os.path.join(project_dir, "node_modules")):
    print("Bootstrapping frontend dependencies with pnpm …")
    subprocess.run("pnpm install", cwd=project_dir, shell=True, check=True)

slime_board = env.GetProjectOption("slime_board") or "BOARD_DEFAULT"
cmd = f"pnpm gh:ci -b {slime_board}"

result = subprocess.run(
	cmd,
	cwd=project_dir,
	shell=True,
	check=True,
	capture_output=True,
	text=True
)
output_flags = []
for line in result.stdout.splitlines():
	line = line.strip()
	if not line or line.startswith(">") or "@" in line:
		continue  # skip junk lines
	output_flags.extend(line.split())

if output_flags:
    print(">>> Appending build flags:", output_flags)
    env["BUILD_FLAGS"] += output_flags
