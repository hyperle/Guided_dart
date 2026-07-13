Import("env")

from pathlib import Path
import subprocess


project_dir = Path(env["PROJECT_DIR"]).resolve()
protocol_script = project_dir.parent / "protocol" / "guidance_protocol.py"
schema_path = project_dir.parent / "protocol" / "guidance_protocol.yaml"
header_path = project_dir / "Drv" / "Inc" / "guidance_protocol_generated.h"


def generate_protocol_header(*args, **kwargs):
    del args
    del kwargs

    cmd = [
        env.subst("$PYTHONEXE"),
        str(protocol_script),
        "--schema",
        str(schema_path),
        "--header",
        str(header_path),
    ]
    result = subprocess.run(cmd, check=False)
    if result.returncode != 0:
        raise RuntimeError("guidance protocol header generation failed")


# Generate immediately so all compile units see the fresh header before dependency scanning.
generate_protocol_header()

# Keep a pre-link generation hook as a safety net for non-standard build flows.
env.AddPreAction(
    "$BUILD_DIR/${PROGNAME}.elf",
    env.VerboseAction(
        generate_protocol_header,
        "Generating $PROJECT_DIR/Drv/Inc/guidance_protocol_generated.h",
    ),
)
