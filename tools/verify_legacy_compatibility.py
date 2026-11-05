#!/usr/bin/env python3
"""Build upstream/default-OFF/explicit-OFF with the same Arm GCC and compare BIN.

Uses Python's standard library, git, CMake, Ninja and an installed Arm toolchain.
All three builds are isolated in a temporary directory; existing build caches,
CubeIDE files and working sources are untouched. No fetch, checkout or flashing.
"""
import argparse
import hashlib
import io
import json
import os
import shlex
import shutil
import subprocess
import sys
import tarfile
import tempfile
from pathlib import Path

UPSTREAM_COMMIT = '084aacd90565bac838c9c7f650ea869d62d6522b'


def command(argv, cwd=None):
    completed = subprocess.run([str(arg) for arg in argv], cwd=cwd, text=True,
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT, check=False)
    if completed.returncode:
        raise RuntimeError('Command failed: ' + ' '.join(map(str, argv)) + '\n' + completed.stdout)
    return completed.stdout


def archive_commit(repo, commit, destination):
    data = subprocess.check_output(['git', '-C', str(repo), 'archive', commit])
    with tarfile.open(fileobj=io.BytesIO(data)) as archive:
        for member in archive:
            target = destination / member.name
            if not target.resolve().is_relative_to(destination.resolve()):
                raise ValueError('Unsafe archive path: ' + member.name)
            if member.isdir():
                target.mkdir(parents=True, exist_ok=True)
            elif member.isfile():
                target.parent.mkdir(parents=True, exist_ok=True)
                with archive.extractfile(member) as source, target.open('wb') as output:
                    shutil.copyfileobj(source, output)
            else:
                raise ValueError('Unsupported archive entry: ' + member.name)


def build(source, folder, toolchain, explicit_off, jobs):
    argv = ['cmake', '-S', source, '-B', folder, '-G', 'Ninja',
            '-DCMAKE_BUILD_TYPE=Debug', f'-DARM_GNU_TOOLCHAIN_ROOT={toolchain}',
            # Keep __FILE__ paths equivalent if assertions are enabled later.
            '-DCMAKE_C_FLAGS=' + shlex.quote(f'-ffile-prefix-map={source}=/openimu')]
    if explicit_off:
        argv.append('-DOPENIMU_SOURCE_TIMESTAMP=OFF')
    command(argv)
    log = command(['cmake', '--build', folder, '--parallel', jobs])
    cache = (folder / 'CMakeCache.txt').read_text()
    if 'OPENIMU_SOURCE_TIMESTAMP:BOOL=OFF' not in cache:
        raise RuntimeError('Default timestamp option is no longer OFF')
    entries = json.loads((folder / 'compile_commands.json').read_text())
    if any(Path(entry['file']).name in ('imu_timestamp.c', 'icm45686_fifo.c') for entry in entries):
        raise RuntimeError('An optional timestamp/FIFO source was compiled with OFF')
    for suffix in ('elf', 'bin', 'map'):
        path = folder / ('open_imu.' + suffix)
        if not path.is_file() or not path.stat().st_size:
            raise RuntimeError('Missing or empty artifact: ' + str(path))
    image = (folder / 'open_imu.bin').read_bytes()
    return image, {'bytes': len(image), 'sha256': hashlib.sha256(image).hexdigest(),
                   'source_timestamp': False, 'translation_units': len(entries),
                   'warnings': [line for line in log.splitlines() if 'warning:' in line.lower()]}


def verify(args, report):
    repo = args.repo.resolve()
    toolchain = args.toolchain_root.resolve()
    compiler = toolchain / 'bin' / 'arm-none-eabi-gcc'
    report['compiler'] = command([compiler, '--version']).splitlines()[0]
    report['upstream_commit'] = command(['git', '-C', repo, 'rev-parse', '--verify',
                                          args.upstream_commit + '^{commit}']).strip()
    report['comparison'] = 'Complete loadable flash BIN bytes, including vectors/code/data; excludes ELF debug metadata.'
    with tempfile.TemporaryDirectory(prefix='openimu-legacy-') as temporary:
        temporary = Path(temporary)
        baseline = temporary / 'upstream'
        baseline.mkdir()
        archive_commit(repo, report['upstream_commit'], baseline)
        # Use exactly the current build definition with the untouched old code.
        shutil.copy2(repo / 'CMakeLists.txt', baseline / 'CMakeLists.txt')
        shutil.copytree(repo / 'cmake', baseline / 'cmake', dirs_exist_ok=True)
        images = []
        report['builds'] = {}
        for label, source, explicit_off in [('upstream', baseline, False),
                                             ('default_off', repo, False), ('explicit_off', repo, True)]:
            image, details = build(source, temporary / ('build-' + label), toolchain, explicit_off, args.jobs)
            images.append(image)
            report['builds'][label] = details
            print(f'PASS {label}: {details["bytes"]} B, SHA256 {details["sha256"]}', flush=True)
        if not images[0] == images[1] == images[2]:
            raise RuntimeError('Upstream/default-OFF/explicit-OFF firmware BIN contents differ')
    report['passed'] = True
    print('PASS complete upstream/default-OFF/explicit-OFF BIN byte identity', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo', type=Path, default=Path(__file__).resolve().parent.parent)
    parser.add_argument('--upstream-commit', default=UPSTREAM_COMMIT)
    parser.add_argument('--toolchain-root', type=Path, default=os.environ.get('ARM_GNU_TOOLCHAIN_ROOT'))
    parser.add_argument('--jobs', type=int, default=4)
    parser.add_argument('--output', type=Path, help='Optional JSON validation report')
    args = parser.parse_args()
    if args.toolchain_root is None or args.jobs < 1:
        parser.error('Set --toolchain-root or ARM_GNU_TOOLCHAIN_ROOT, and use positive --jobs')
    report = {'passed': False}
    try:
        verify(args, report)
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError, tarfile.TarError) as error:
        report['error'] = str(error)
        print('FAIL ' + str(error), file=sys.stderr)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + '\n')
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
