#!/usr/bin/env python3
"""Build once, then upload that build using the project's Arduino settings."""
import argparse
import hashlib
import json
import shutil
import subprocess
from pathlib import Path

PROJECT = Path(__file__).resolve().parent.parent
BUILD = PROJECT / 'build' / 'direct-upload'
STAMP = BUILD / 'source-state.json'


def source_state(config):
    digest = hashlib.sha256(json.dumps(config, sort_keys=True).encode())
    for path in sorted(PROJECT.iterdir()):
        if path.suffix in ('.ino', '.cpp', '.h'):
            digest.update(path.name.encode())
            digest.update(path.read_bytes())
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['build', 'upload', 'build-upload'])
    args = parser.parse_args()
    config = json.loads((PROJECT / '.vscode' / 'arduino.json').read_text())
    cli = shutil.which('arduino-cli')
    if not cli:
        cli = '/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli'
    if not Path(cli).is_file():
        parser.error('Arduino CLI not found. Install Arduino IDE or add arduino-cli to PATH.')
    fqbn = config['board']
    if config.get('configuration'):
        fqbn += ':' + config['configuration']
    print(f'Project: {PROJECT}\nBoard: {fqbn}\nPort: {config.get("port", "not selected")}', flush=True)

    if args.action in ('build', 'build-upload'):
        BUILD.mkdir(parents=True, exist_ok=True)
        # Invalidate any previous result before compiling. Never upload an old
        # binary after a failed build or an edit made during compilation.
        STAMP.unlink(missing_ok=True)
        state = source_state(config)
        print('Building firmware once...', flush=True)
        subprocess.run([cli, 'compile', '--fqbn', fqbn, '--build-path', str(BUILD), str(PROJECT)], check=True)
        current_config = json.loads((PROJECT / '.vscode' / 'arduino.json').read_text())
        if state != source_state(current_config):
            raise RuntimeError('Files or board settings changed during compilation. Run the task again after saving edits.')
        STAMP.write_text(json.dumps({'source': state}))
        print('Build finished successfully.', flush=True)

    if args.action in ('upload', 'build-upload'):
        if not STAMP.exists() or json.loads(STAMP.read_text()).get('source') != source_state(config):
            raise RuntimeError('No current build. Run the Build and Upload task; no automatic recompile was started.')
        if not (BUILD / 'SensorShield.ino.bin').is_file():
            raise RuntimeError('Built firmware is missing. Run Build and Upload.')
        if not config.get('port'):
            raise RuntimeError('Select a USB port in .vscode/arduino.json.')
        print('Uploading the built firmware now (no second compilation). Close Serial Monitor first.', flush=True)
        subprocess.run([cli, 'upload', '--fqbn', fqbn, '--port', config['port'],
                        '--input-dir', str(BUILD), str(PROJECT)], check=True)
        print('Upload finished. Open Serial Monitor at 115200 baud.', flush=True)


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, subprocess.CalledProcessError) as error:
        raise SystemExit(f'Stopped: {error}')
