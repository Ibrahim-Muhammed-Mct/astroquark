"""Build AstroQuark C utilities in a local temporary directory."""
import shutil
import subprocess
import tempfile
from contextlib import contextmanager
from pathlib import Path


@contextmanager
def compiled_tool(name):
    compiler = shutil.which("gcc")
    if compiler is None:
        raise RuntimeError("Install GCC in Ubuntu/WSL before running the AstroQuark C tools.")

    source = Path(__file__).resolve().with_name(f"{name}.c")
    with tempfile.TemporaryDirectory(prefix="astroquark-") as directory:
        executable = Path(directory) / name
        subprocess.run(
            [compiler, "-std=c11", "-Wall", "-Wextra", str(source), "-lm", "-o", str(executable)],
            check=True,
        )
        yield executable
