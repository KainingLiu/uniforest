#!/usr/bin/env python3
"""Import cube JPEGs and their sidecars from a collection.zip archive."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import zipfile


ROOT = Path(__file__).resolve().parents[1]
ARCHIVE = ROOT.parents[2] / 'collection.zip'
OUTPUT = ROOT / 'data/raw/collection_20260926'
IMAGE_MEMBER = re.compile(
    r'(?:^|/)runs/(\d{8}T\d{6}_\d{6}Z_[0-9a-f]{8})/images/(\d{6}\.jpg)$')


def sha256(path: Path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive', type=Path, default=ARCHIVE)
    parser.add_argument('--output', type=Path, default=OUTPUT)
    parser.add_argument('--check-only', action='store_true')
    args = parser.parse_args()
    if args.output.exists() and not args.check_only:
        raise FileExistsError(f'raw collection already exists: {args.output}')
    with zipfile.ZipFile(args.archive) as source:
        names = source.namelist()
        if len(names) != len(set(names)):
            raise ValueError('archive contains duplicate member names')
        members = set(names)
        if sum(info.file_size for info in source.infolist()) > 200_000_000:
            raise ValueError('archive is unexpectedly large after decompression')
        image_members = sorted((name, match) for name in names
                               if (match := IMAGE_MEMBER.search(name)))
        if not image_members:
            raise ValueError('archive contains no collected cube images')
        records = []
        for member, match in image_members:
            session, filename = match.groups()
            sidecar_member = member[:-4] + '.json'
            if sidecar_member not in members:
                raise ValueError(f'missing image sidecar: {member}')
            sidecar = json.loads(source.read(sidecar_member))
            relative = f'runs/{session}/images/{filename}'
            if (sidecar['image_path'] != relative
                    or sidecar['session_id'] != session
                    or sidecar['annotation_status'] != 'unlabelled'
                    or sidecar['camera']['selector'] != 'cube'):
                raise ValueError(f'inconsistent cube image sidecar: {member}')
            records.append({'id': len(records) + 1, 'image': relative,
                            'session_id': session, 'task': sidecar['task'],
                            'phase': sidecar['phase'],
                            'profile': sidecar['profile'],
                            'split_group': sidecar['split_group']})
        if len({record['image'] for record in records}) != len(records):
            raise ValueError('multiple archive members map to the same collected image')
        manifest = {'source_archive': str(args.archive.resolve()),
                    'source_sha256': sha256(args.archive),
                    'image_count': len(records), 'images': records}
        if args.check_only:
            print(f'Checked {len(records)} images across '
                  f'{len({record["session_id"] for record in records})} sessions; '
                  f'SHA256 {manifest["source_sha256"]}')
            return
        args.output.mkdir(parents=True)
        for member, match in image_members:
            session, filename = match.groups()
            destination = args.output / 'runs' / session / 'images'
            destination.mkdir(parents=True, exist_ok=True)
            (destination / filename).write_bytes(source.read(member))
            (destination / (filename[:-4] + '.json')).write_bytes(
                source.read(member[:-4] + '.json'))
        (args.output / 'import_manifest.json').write_text(
            json.dumps(manifest, indent=2, ensure_ascii=False) + '\n',
            encoding='utf-8')
    print(f'Imported {len(records)} reviewed-source images to {args.output}')


if __name__ == '__main__':
    main()
