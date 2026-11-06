#!/usr/bin/env python3
"""Run software regressions and firmware-to-host wire compatibility checks.

Requires a native C compiler supporting ASan/UBSan. Register side effects are
modeled; physical CAN, GUI operation and hardware timing accuracy are not tested.
"""
from pathlib import Path
import os
import subprocess
import sys
import tempfile

from clock_sync import Anchor


def main():
    root = Path(__file__).resolve().parents[1]
    subprocess.run([sys.executable, '-m', 'unittest', 'discover', '-s', str(root / 'tools'), '-p', 'test_*.py'], check=True)
    with tempfile.TemporaryDirectory(prefix='openimu-sync-') as temporary:
        binary = Path(temporary) / 'sync-test'
        subprocess.run([os.environ.get('CC', 'cc'), '-std=c11', '-Wall', '-Wextra', '-Werror',
                        '-g', '-fsanitize=address,undefined', '-fno-pie', '-no-pie',
                        '-I', str(root / 'Core/Inc'), str(root / 'tests/native/sync_test.c'),
                        '-o', str(binary)], check=True)
        result = subprocess.run([str(binary)], capture_output=True, text=True)
        if result.returncode:
            print(result.stdout, end='')
            print(result.stderr, end='', file=sys.stderr)
            result.check_returncode()
        print(result.stdout, end='')
        encoded = next(line.split('=', 1)[1] for line in result.stdout.splitlines() if line.startswith('ANCHOR_HEX='))
        anchor = Anchor.decode(0x711, bytes.fromhex(encoded))
        assert (anchor.session, anchor.sequence, anchor.epoch, anchor.local_sync,
                anchor.local_sample, anchor.source_sample) == (77, 2, 123, 5000, 5010, 65541)
        revoked = next(line.split('=', 1)[1] for line in result.stdout.splitlines() if line.startswith('INVALID_HEX='))
        invalid = Anchor.decode(0x711, bytes.fromhex(revoked))
        assert invalid.flags == 0 and (invalid.session, invalid.sequence) == (77, 2)
        subprocess.run([str(binary), '--init-failure'], check=True)
        print('Firmware encoder → host decoder: PASS')
        faults = Path(temporary) / 'sync-fault-test'
        subprocess.run([os.environ.get('CC', 'cc'), '-std=c11', '-Wall', '-Wextra', '-Werror',
                        '-g', '-fsanitize=address,undefined', '-fno-pie', '-no-pie',
                        '-I', str(root / 'Core/Inc'), str(root / 'tests/native/sync_fault_test.c'),
                        '-o', str(faults)], check=True)
        cases = ['config-fail-' + str(i) for i in range(1, 5)] + [
            'rng-timeout', 'reinit-failure', 'rx-preemption', 'rx-failure-age',
            'rx-overflow-during-read', 'rx-overflow-before-report',
            'revocation-retry', 'overflow-read-race', 'capture-arrival-race',
            'capture-across-wrap', 'stale-capture-one-wrap', 'first-irq-lost-wrap', 'capture-token-reuse',
            'capture-overrun', 'lease-tick-wrap', 'malformed-controls', 'source-two-hours']
        for case in cases:
            subprocess.run([str(faults), case], check=True)
        integration = Path(temporary) / 'task-integration'
        subprocess.run([os.environ.get('CC', 'cc'), '-std=c11', '-Wall', '-Wextra', '-Werror',
                        '-g', '-fsanitize=address,undefined', '-fno-pie', '-no-pie',
                        '-I', str(root / 'tests/native'), '-I', str(root / 'Core/Inc'),
                        str(root / 'tests/native/task_integration_test.c'), '-o', str(integration)], check=True)
        for scenario in ('normal', 'backlog', 'spi-failure'):
            subprocess.run([str(integration), scenario], check=True)



if __name__ == '__main__':
    main()
