"""Run only pure-Python tests, AST/import checks and protected-input hash audit.

Always creates a new evidence JSON. It never overwrites earlier runs.
"""
import ast
import argparse
import importlib
import io
import json
import sys
import unittest
import uuid
from hashlib import sha256
from pathlib import Path
import numpy
import scipy
import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--allow-maintenance-report', action='store_true',
                        help='report historical main-report differences, but still reject changes during tests')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    maintenance = root.parents[3] / '项目报告.md'
    maintenance_before = sha256(maintenance.read_bytes()).hexdigest()
    sources = sorted(root.rglob('*.py'))
    for path in sources:
        ast.parse(path.read_text(), filename=str(path))
    for module in ('core.contracts', 'core.math_tools', 'core.limits', 'core.interfaces',
                   'validation.collision', 'validation.numerical'):
        importlib.import_module('v2.'+module)
    stream = io.StringIO()
    suite = unittest.defaultTestLoader.discover(str(root/'tests'), top_level_dir=str(root.parent))
    result = unittest.TextTestRunner(stream=stream, verbosity=2).run(suite)
    protected = json.loads((root/'docs/source_protection.json').read_text())
    changed = [p for p, h in protected.items() if sha256(Path(p).read_bytes()).hexdigest() != h]
    allowed = [str(maintenance)] if args.allow_maintenance_report and sha256(maintenance.read_bytes()).hexdigest() == maintenance_before else []
    blocking = [p for p in changed if p not in allowed]
    forbidden = [n for n in sys.modules if n.split('.')[0] in ('rospy', 'moveit_commander', 'roslaunch')]
    evidence = dict(tests_run=result.testsRun, failures=len(result.failures), errors=len(result.errors),
                    skipped=len(result.skipped), transcript=stream.getvalue(),
                    python=sys.version, numpy=numpy.__version__, scipy=scipy.__version__, pyyaml=yaml.__version__,
                    syntax_files_checked=len(sources), import_modules_checked=6,
                    protected_files_checked=len(protected), changed_protected_files=changed,
                    allowed_maintenance_differences=[p for p in changed if p in allowed],
                    blocking_protected_changes=blocking,
                    forbidden_imports=forbidden,
                    source_hashes={str(p.relative_to(root)): sha256(p.read_bytes()).hexdigest() for p in sources})
    output = root/'docs'/('test_run_'+uuid.uuid4().hex[:12]+'.json')
    with output.open('x') as f:
        json.dump(evidence, f, indent=2, ensure_ascii=False)
    print(stream.getvalue())
    print('Evidence:', output)
    print('Syntax:', len(sources), 'Imports:', 6, 'Protected:', len(protected), 'changed:', changed, 'ROS imports:', forbidden)
    return 0 if result.wasSuccessful() and not blocking and not forbidden else 1


if __name__ == '__main__':
    raise SystemExit(main())
