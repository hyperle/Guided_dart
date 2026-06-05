Import("env")

from pathlib import Path
import subprocess


project_dir = Path(env["PROJECT_DIR"])
config = env.GetProjectConfig()
port = config.get("openmv", "port")
script = project_dir / config.get("openmv", "script")


def upload_openmv(source, target, env):
    if not script.exists():
        raise RuntimeError("OpenMV script not found: %s" % script)

    cmd = [
        "python3",
        str(project_dir / "tools" / "write_openmv_sources.py"),
        "--port",
        port,
        "--script",
        str(script),
    ]
    result = subprocess.run(cmd, check=False)
    if result.returncode != 0:
        raise RuntimeError("OpenMV upload failed")


env.AddCustomTarget(
    "upload_openmv",
    None,
    upload_openmv,
    title="Upload OpenMV Sources",
    description="Write OpenMV Python sources to the board over the REPL UART",
)

upload_alias = env.Alias("upload")[0]
upload_alias.get_executor().set_action_list(
    env.VerboseAction(upload_openmv, "Uploading OpenMV sources")
)
env.AlwaysBuild(upload_alias)
