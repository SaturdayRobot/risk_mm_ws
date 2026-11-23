#!/usr/bin/env python3
from pathlib import Path
import json, subprocess, sys

def main():
    ws=Path(__file__).resolve().parents[2]
    run=subprocess.run([sys.executable,'-m','pytest','-q',str(ws/'scripts/validation/test_core.py'),
        '--junitxml='+str(ws/'reports/core.xml')],text=True,capture_output=True)
    report={'returncode':run.returncode,'stdout':run.stdout,'stderr':run.stderr,
            'scope':'risk / switching / arm-first contracts; test doubles do not establish real planning or execution success'}
    (ws/'reports/components.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(run.stdout,end='');print(run.stderr,end='')
    return run.returncode

if __name__=='__main__':raise SystemExit(main())
