"""Pyodide guard: the package may import only the standard library, numpy and itself."""
import ast
import sys
from pathlib import Path

import powerkettle

PACKAGE = Path(powerkettle.__file__).parent
ALLOWED = set(sys.stdlib_module_names) | {"numpy", "powerkettle"}


def imported_roots(path: Path) -> set[str]:
    roots = set()
    for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"), filename=str(path))):
        if isinstance(node, ast.Import):
            roots |= {a.name.split(".")[0] for a in node.names}
        elif isinstance(node, ast.ImportFrom):
            roots.add("powerkettle" if node.level > 0 else node.module.split(".")[0])
    return roots


def test_package_imports_only_stdlib_numpy_and_itself():
    files = sorted(PACKAGE.glob("*.py"))
    assert {"pipeline.py", "session_io.py", "__main__.py"} <= {f.name for f in files}
    bad = {f.name: sorted(imported_roots(f) - ALLOWED) for f in files}
    assert {k: v for k, v in bad.items() if v} == {}


def test_guard_catches_third_party(tmp_path):
    p = tmp_path / "m.py"
    p.write_text("import scipy.signal\nfrom numpy import fft\nfrom . import quat\nimport os\n")
    assert imported_roots(p) - ALLOWED == {"scipy"}
