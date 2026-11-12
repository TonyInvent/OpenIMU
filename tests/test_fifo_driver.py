#!/usr/bin/env python3
"""Run the actual FIFO driver against a native mocked SPI transport (no firmware build)."""
import os
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory(prefix='openimu-fifo-test-') as directory:
    for source in ('fifo_driver_test.c', 'fifo_task_test.c'):
        binary = Path(directory) / source.removesuffix('.c')
        subprocess.run([os.environ.get('CC', 'cc'), '-std=c11', '-Wall', '-Wextra', '-Werror',
                        '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
                        '-I', str(root / 'Core/Inc'), '-I', str(root / 'tests/native'),
                        str(root / 'tests/native' / source), '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True)
