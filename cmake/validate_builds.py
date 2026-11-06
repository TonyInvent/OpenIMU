#!/usr/bin/env python3
"""Build the sync/legacy matrix and analyze application C with the Arm compiler.

Needs CMake, Ninja and Arm GNU GCC (PATH or ARM_GNU_TOOLCHAIN_ROOT).
Outputs evidence, not a claim about MCU register behavior or hardware timing.
"""
from __future__ import annotations
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shlex
import subprocess
import tempfile


def run(command, cwd=None):
    result = subprocess.run(command, cwd=cwd, capture_output=True, text=True)
    if result.returncode:
        raise RuntimeError(f"Command failed: {shlex.join(map(str, command))}\n{result.stdout}{result.stderr}")
    return result.stdout


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline-dir', type=Path, help='Official debug/release directories with open_imu.bin')
    parser.add_argument('--report', type=Path, default=root / 'build/validation-builds.json')
    args = parser.parse_args()
    cases = [(mode, config, node) for node in (1, 5) for mode in ('OFF', 'ON')
             for config in ('Debug', 'Release')]
    cases += [('ON', 'Release', node) for node in (2, 3, 4)]
    report = {'time_utc': datetime.now(timezone.utc).isoformat(), 'hardware_validated': False,
              'source_head': run(['git', 'rev-parse', 'HEAD'], root).strip(), 'builds': [],
              'static_analysis': [], 'source_sha256': {},
              'analyzer_exceptions': {'files': ['main.c', 'stm32g4xx_it.c', 'syscalls.c', 'newlib_lock_glue.c'],
                                      'flag': '-Wno-analyzer-infinite-loop',
                                      'reason': 'Unchanged upstream fatal handlers and scheduler-return traps intentionally never return.'}}
    for mode, config, node in cases:
        build = root / 'build' / f'audit-{mode.lower()}-{config.lower()}-node{node}'
        run(['cmake', '-S', str(root), '-B', str(build), '-G', 'Ninja',
             f'-DCMAKE_BUILD_TYPE={config}', f'-DOPENIMU_CAN_SYNC={mode}', f'-DOPENIMU_NODE_ID={node}'])
        run(['cmake', '--build', str(build), '--parallel', '4'])
        entries = json.loads((build / 'compile_commands.json').read_text())
        compiler = Path(shlex.split(entries[0]['command'])[0])
        report['compiler'] = run([str(compiler), '--version']).splitlines()[0]
        size = run([str(compiler.with_name('arm-none-eabi-size')), '-B', str(build / 'open_imu.elf')])
        sections = list(map(int, size.splitlines()[1].split()[:3]))
        binary = (build / 'open_imu.bin').read_bytes()
        row = {'sync': mode, 'configuration': config, 'node': node,
               'flash_image_bytes': len(binary), 'ram_static_bytes': sections[1] + sections[2],
               'sha256': hashlib.sha256(binary).hexdigest(), 'official_binary_identical': None}
        if args.baseline_dir and mode == 'OFF' and node == 1:
            baseline = args.baseline_dir / config.lower() / 'open_imu.bin'
            row['official_binary_identical'] = binary == baseline.read_bytes()
            if not row['official_binary_identical']:
                raise RuntimeError(f'Legacy {config} binary differs from supplied official baseline')
        report['builds'].append(row)
        print(f'{mode} {config} node{node}: PASS Flash={len(binary)} RAM={sections[1]+sections[2]}')
        if node == 1:
            application = [entry for entry in entries if '/Core/' in entry['file'] and entry['file'].endswith('.c')]
            with tempfile.TemporaryDirectory(prefix='openimu-static-') as temporary:
                for index, entry in enumerate(application):
                    command = shlex.split(entry['command'])
                    command[command.index('-o') + 1] = str(Path(temporary) / f'{index}.o')
                    flags = ['-fanalyzer', '-Werror']
                    if Path(entry['file']).name in report['analyzer_exceptions']['files']:
                        flags.append('-Wno-analyzer-infinite-loop')
                    run(command + flags, entry['directory'])
            report['static_analysis'].append({'sync': mode, 'configuration': config,
                                             'translation_units': len(application), 'result': 'PASS'})
    for base in ('Core', 'tools', 'tests', 'cmake'):
        for path in sorted((root / base).rglob('*')):
            if path.is_file() and path.suffix in ('.c', '.h', '.py', '.cmake'):
                report['source_sha256'][str(path.relative_to(root))] = hashlib.sha256(path.read_bytes()).hexdigest()
    report['result'] = 'PASS'
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2) + '\n')
    print(f'Build/static evidence: {args.report}')


if __name__ == '__main__':
    main()
