#!/usr/bin/env python3
# Copyright 2026 Boombroke
# Licensed under the Apache License, Version 2.0
"""Merge a sortie directory of rosbag2 MCAP shards into a single MCAP file.

A sortie directory is what ``match_recorder_node`` produces: a directory like
``sortie_YYYYMMDD_HHMMSS/`` containing ``metadata.yaml`` and one or more
``*.mcap`` shards (rosbag2 ``--max-bag-duration`` slices). This CLI walks
shards in chronological order (using ``metadata.yaml`` if present, falling
back to filename suffix) and concatenates them into ``<basename>_full.mcap``
while preserving topic metadata, QoS, message-definition records, and
serialized payloads.

Usage::

    ros2 run sentry_match_recorder merge_sortie <SORTIE_DIR>
    python3 -m sentry_match_recorder.merge_sortie <SORTIE_DIR>

Implementation note: we use ``rosbag2_py`` (which is always available in a
ROS 2 Jazzy environment) plus the ``rosbag2_storage_mcap`` plugin. We copy
serialized CDR bytes verbatim, so no msg type lookup is required and the
merge does not depend on every message package being installed.
"""

from __future__ import annotations

import argparse
import os
import re
import shutil
import sys
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

try:
    import yaml  # noqa: F401  (used opportunistically for shard ordering)
    _HAS_YAML = True
except ImportError:
    _HAS_YAML = False


# --------------------------------------------------------------------------- #
# Shard discovery                                                             #
# --------------------------------------------------------------------------- #
_SUFFIX_RE = re.compile(r'_(\d+)\.mcap$', re.IGNORECASE)


def _shard_index(path: Path) -> int:
    """Return the trailing ``_<N>`` index from a shard filename, or -1."""
    m = _SUFFIX_RE.search(path.name)
    return int(m.group(1)) if m else -1


def _discover_shards(sortie_dir: Path) -> List[Path]:
    """Return shard paths in playback order.

    Preference order:
      1) ``metadata.yaml`` ``relative_file_paths`` (rosbag2's own ordering).
      2) Lexicographic sort by the trailing ``_<N>.mcap`` integer suffix.
      3) Plain alphabetical fallback.
    """
    metadata = sortie_dir / 'metadata.yaml'
    if metadata.is_file() and _HAS_YAML:
        try:
            with metadata.open('r', encoding='utf-8') as f:
                meta = yaml.safe_load(f) or {}
            info = meta.get('rosbag2_bagfile_information') or {}
            rel_paths = info.get('relative_file_paths') or []
            files = [sortie_dir / Path(p) for p in rel_paths]
            files = [p for p in files if p.is_file() and p.suffix.lower() == '.mcap']
            if files:
                return files
        except Exception:
            # Fall through to filename-based discovery.
            pass

    shards = sorted(sortie_dir.glob('*.mcap'), key=lambda p: (_shard_index(p), p.name))
    return shards


# --------------------------------------------------------------------------- #
# Merge implementation                                                        #
# --------------------------------------------------------------------------- #
def _open_writer(out_path: Path):
    """Open a SequentialWriter for a single MCAP file at ``out_path``.

    rosbag2 wants a *directory* URI for output, but we want a single file. We
    create a temporary working directory next to ``out_path``, write into it,
    then move the resulting ``.mcap`` to ``out_path`` and discard the dir.
    Returns ``(writer, work_dir, work_uri)``.
    """
    import rosbag2_py

    out_path = out_path.resolve()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    work_dir = out_path.parent / (out_path.stem + '.merge_tmp')
    if work_dir.exists():
        shutil.rmtree(work_dir)
    work_uri = str(work_dir)  # rosbag2 will create the directory itself.

    storage_opts = rosbag2_py.StorageOptions(uri=work_uri, storage_id='mcap')
    converter_opts = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr',
    )
    writer = rosbag2_py.SequentialWriter()
    writer.open(storage_opts, converter_opts)
    return writer, work_dir, work_uri


def _finalize_output(work_dir: Path, out_path: Path) -> None:
    """Move the single mcap shard out of work_dir into out_path; cleanup."""
    if not work_dir.is_dir():
        raise RuntimeError(f'merger work dir vanished: {work_dir}')
    mcaps = sorted(work_dir.glob('*.mcap'))
    if not mcaps:
        raise RuntimeError(f'merger produced no mcap in {work_dir}')
    if len(mcaps) > 1:
        # Should never happen; we never set max_bagfile_duration/size.
        raise RuntimeError(
            f'merger produced {len(mcaps)} mcap files in {work_dir}; expected 1'
        )
    if out_path.exists():
        out_path.unlink()
    shutil.move(str(mcaps[0]), str(out_path))
    shutil.rmtree(work_dir)


def merge_sortie(
    sortie_dir: Path,
    output: Optional[Path] = None,
    remove_shards: bool = False,
    dry_run: bool = False,
    quiet: bool = False,
) -> Path:
    """Merge all MCAP shards in ``sortie_dir`` into a single MCAP file.

    Returns the path to the merged file (or the planned path in ``dry_run``
    mode). Raises ``FileNotFoundError`` / ``ValueError`` on bad input.
    """
    sortie_dir = sortie_dir.resolve()
    if not sortie_dir.is_dir():
        raise FileNotFoundError(f'not a directory: {sortie_dir}')

    shards = _discover_shards(sortie_dir)
    if not shards:
        raise FileNotFoundError(f'no .mcap shards found in {sortie_dir}')

    basename = sortie_dir.name
    out_path = (output.resolve() if output is not None
                else sortie_dir / f'{basename}_full.mcap')

    # Refuse to overwrite a shard accidentally.
    if out_path in shards:
        raise ValueError(
            f'output path {out_path} would overwrite a shard; '
            f'pass --output to disambiguate'
        )

    total_size = sum(p.stat().st_size for p in shards)

    def log(msg: str) -> None:
        if not quiet:
            print(msg, flush=True)

    log(f'[merge_sortie] sortie dir: {sortie_dir}')
    log(f'[merge_sortie] {len(shards)} shard(s), total size {_human(total_size)}')
    for i, p in enumerate(shards):
        log(f'  shard {i+1}/{len(shards)}: {p.name} ({_human(p.stat().st_size)})')
    log(f'[merge_sortie] output -> {out_path}')

    if dry_run:
        log('[merge_sortie] --dry-run: not writing anything')
        return out_path

    # Lazy-import rosbag2_py so --dry-run / --help work without ROS sourced.
    import rosbag2_py

    writer, work_dir, _work_uri = _open_writer(out_path)
    seen_topics: dict = {}
    seen_definitions: set = set()
    total_messages = 0

    try:
        for shard_idx, shard in enumerate(shards):
            log(f'[merge_sortie] reading shard {shard_idx+1}/{len(shards)}: '
                f'{shard.name}')
            reader = rosbag2_py.SequentialReader()
            storage_opts = rosbag2_py.StorageOptions(
                uri=str(shard), storage_id='mcap')
            converter_opts = rosbag2_py.ConverterOptions(
                input_serialization_format='cdr',
                output_serialization_format='cdr',
            )
            reader.open(storage_opts, converter_opts)

            # Register topics seen in this shard (idempotent on (name, type)).
            for tm in reader.get_all_topics_and_types():
                key = (tm.name, tm.type)
                prev = seen_topics.get(tm.name)
                if prev is None:
                    writer.create_topic(tm)
                    seen_topics[tm.name] = tm.type
                elif prev != tm.type:
                    raise RuntimeError(
                        f'topic {tm.name} has conflicting types across '
                        f'shards: {prev!r} vs {tm.type!r}'
                    )

            # Forward message definitions if available (Jazzy+ feature).
            try:
                defs = reader.get_all_message_definitions()
            except Exception:
                defs = []
            for d in defs:
                key = (d.topic_type, d.encoding, d.type_hash)
                if key in seen_definitions:
                    continue
                seen_definitions.add(key)
                # Best-effort: not all writer builds expose add_message_definition;
                # rosbag2 handles definitions internally for MCAP, so we don't
                # fail if the API isn't there.
                add_fn = getattr(writer, 'add_message_definition', None)
                if add_fn is not None:
                    try:
                        add_fn(d)
                    except Exception:
                        pass

            shard_messages = 0
            while reader.has_next():
                topic, data, t = reader.read_next()
                writer.write(topic, data, t)
                shard_messages += 1
                total_messages += 1
                if shard_messages % 50000 == 0:
                    log(f'    ... {shard_messages} msgs (cumulative '
                        f'{total_messages})')
            log(f'    shard done: {shard_messages} messages')
            del reader  # close current shard before opening next

        log(f'[merge_sortie] wrote {total_messages} messages across '
            f'{len(seen_topics)} topics')
    finally:
        del writer
        try:
            _finalize_output(work_dir, out_path)
        except Exception:
            # Leave work_dir in place for forensics if finalize failed.
            raise

    if remove_shards:
        log('[merge_sortie] --remove-shards: deleting source shards')
        for p in shards:
            try:
                p.unlink()
            except OSError as e:
                log(f'    WARN: cannot remove {p.name}: {e}')
        meta = sortie_dir / 'metadata.yaml'
        if meta.is_file():
            try:
                meta.unlink()
            except OSError as e:
                log(f'    WARN: cannot remove metadata.yaml: {e}')

    log(f'[merge_sortie] OK -> {out_path} ({_human(out_path.stat().st_size)})')
    return out_path


# --------------------------------------------------------------------------- #
# CLI                                                                         #
# --------------------------------------------------------------------------- #
def _human(n: int) -> str:
    """Pretty-print a byte count."""
    units = ('B', 'KiB', 'MiB', 'GiB', 'TiB')
    f = float(n)
    for u in units:
        if f < 1024.0 or u == units[-1]:
            return f'{f:.1f} {u}'
        f /= 1024.0
    return f'{n} B'


def _parse_args(argv: Optional[Sequence[str]]) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        prog='merge_sortie',
        description='Merge a rosbag2 sortie directory of MCAP shards into a '
                    'single MCAP file.',
    )
    p.add_argument(
        'sortie_dir',
        type=Path,
        help='sortie directory containing *.mcap shards (and metadata.yaml).',
    )
    p.add_argument(
        '--output', '-o',
        type=Path,
        default=None,
        help='output mcap path (default: <sortie_dir>/<basename>_full.mcap).',
    )
    p.add_argument(
        '--remove-shards',
        action='store_true',
        help='delete source shards (and metadata.yaml) after a successful merge.',
    )
    p.add_argument(
        '--dry-run',
        action='store_true',
        help='only list shards / planned output, do not write anything.',
    )
    p.add_argument(
        '--quiet', '-q',
        action='store_true',
        help='suppress per-shard progress output.',
    )
    return p.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _parse_args(argv)
    try:
        merge_sortie(
            sortie_dir=args.sortie_dir,
            output=args.output,
            remove_shards=args.remove_shards,
            dry_run=args.dry_run,
            quiet=args.quiet,
        )
    except FileNotFoundError as e:
        print(f'error: {e}', file=sys.stderr)
        return 2
    except ValueError as e:
        print(f'error: {e}', file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print('interrupted', file=sys.stderr)
        return 130
    except Exception as e:  # noqa: BLE001
        print(f'merge failed: {e}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
