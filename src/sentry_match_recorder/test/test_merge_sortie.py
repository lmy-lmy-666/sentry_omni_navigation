#!/usr/bin/env python3
# Copyright 2026 Boombroke
# Licensed under the Apache License, Version 2.0
"""Self-test for merge_sortie.

Builds a synthetic 2-shard sortie directory using rosbag2_py SequentialWriter
(MCAP backend), runs merge_sortie.merge_sortie(), then reads the merged file
back with SequentialReader and asserts:

  * total message count == sum of shard message counts
  * every (topic, type) seen in the shards is present in the merged file
  * payload bytes round-trip exactly (we use a small distinguishable payload
    per message)

This intentionally does NOT require any real ROS .msg type to be importable:
rosbag2_py just shovels serialized bytes through, so we hand it bogus but
deterministic CDR-shaped payloads.
"""

from __future__ import annotations

import sys
import tempfile
from pathlib import Path

# Make the package importable when running this file directly.
_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parent))

from sentry_match_recorder.merge_sortie import merge_sortie  # noqa: E402


def _build_shard(shard_path: Path, topic_msgs):
    """Write a single mcap shard at shard_path containing topic_msgs.

    topic_msgs: list of (topic_name, type_name, payload_bytes, t_ns).
    """
    import rosbag2_py

    work_dir = shard_path.parent / (shard_path.stem + '_tmp')
    if work_dir.exists():
        import shutil
        shutil.rmtree(work_dir)

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(work_dir), storage_id='mcap'),
        rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr',
        ),
    )
    seen = set()
    for topic, typ, _payload, _t in topic_msgs:
        if topic in seen:
            continue
        seen.add(topic)
        writer.create_topic(rosbag2_py.TopicMetadata(
            id=0,
            name=topic,
            type=typ,
            serialization_format='cdr',
        ))
    for topic, _typ, payload, t in topic_msgs:
        writer.write(topic, payload, t)
    del writer

    # Move the single produced mcap to the requested path.
    import shutil
    mcaps = sorted(work_dir.glob('*.mcap'))
    assert len(mcaps) == 1, f'expected 1 mcap, got {mcaps}'
    if shard_path.exists():
        shard_path.unlink()
    shutil.move(str(mcaps[0]), str(shard_path))
    shutil.rmtree(work_dir)


def _read_all(mcap_path: Path):
    import rosbag2_py
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(mcap_path), storage_id='mcap'),
        rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr',
        ),
    )
    topics = {(tm.name, tm.type) for tm in reader.get_all_topics_and_types()}
    msgs = []
    while reader.has_next():
        topic, data, t = reader.read_next()
        msgs.append((topic, bytes(data), int(t)))
    del reader
    return topics, msgs


def main() -> int:
    with tempfile.TemporaryDirectory() as tmp:
        sortie_dir = Path(tmp) / 'sortie_19700101_000001'
        sortie_dir.mkdir()

        # Shard 0: topic /foo with 3 msgs, topic /bar with 2 msgs.
        shard0 = [
            ('/foo', 'std_msgs/msg/String', b'\x00' * 4 + b'foo-0', 1_000_000_000),
            ('/foo', 'std_msgs/msg/String', b'\x00' * 4 + b'foo-1', 1_100_000_000),
            ('/bar', 'std_msgs/msg/Int32',  b'\x00' * 4 + b'bar-0', 1_200_000_000),
            ('/foo', 'std_msgs/msg/String', b'\x00' * 4 + b'foo-2', 1_300_000_000),
            ('/bar', 'std_msgs/msg/Int32',  b'\x00' * 4 + b'bar-1', 1_400_000_000),
        ]
        # Shard 1: topic /foo continues, plus a new topic /baz.
        shard1 = [
            ('/foo', 'std_msgs/msg/String', b'\x00' * 4 + b'foo-3', 2_000_000_000),
            ('/baz', 'std_msgs/msg/Float32', b'\x00' * 4 + b'baz-0', 2_100_000_000),
            ('/foo', 'std_msgs/msg/String', b'\x00' * 4 + b'foo-4', 2_200_000_000),
        ]

        _build_shard(sortie_dir / 'sortie_19700101_000001_0.mcap', shard0)
        _build_shard(sortie_dir / 'sortie_19700101_000001_1.mcap', shard1)

        out = merge_sortie(sortie_dir, quiet=True)

        # 1) output file exists and is non-empty
        assert out.is_file(), f'merged file not produced: {out}'
        assert out.stat().st_size > 0, 'merged file is empty'

        topics, msgs = _read_all(out)

        # 2) all (topic, type) pairs present
        expected_topics = {
            ('/foo', 'std_msgs/msg/String'),
            ('/bar', 'std_msgs/msg/Int32'),
            ('/baz', 'std_msgs/msg/Float32'),
        }
        assert expected_topics.issubset(topics), \
            f'missing topics: {expected_topics - topics}'

        # 3) message count matches
        expected_msgs = shard0 + shard1
        assert len(msgs) == len(expected_msgs), \
            f'message count mismatch: got {len(msgs)}, want {len(expected_msgs)}'

        # 4) payload bytes round-trip; merged messages may be reordered
        # internally but the multiset must match.
        got = sorted((t, p) for (t, p, _ts) in msgs)
        want = sorted((t, p) for (t, _ty, p, _ts) in expected_msgs)
        assert got == want, 'payload contents differ between shards and merged'

        # 5) source shards untouched (default behavior)
        remaining_shards = sorted(sortie_dir.glob('*.mcap'))
        # 2 shards + the merged _full.mcap
        assert len(remaining_shards) == 3, \
            f'expected shards preserved + 1 merged, got {remaining_shards}'

        # --- additional check: --remove-shards drops shards ---
        out2_dir = Path(tmp) / 'sortie_19700101_000002'
        out2_dir.mkdir()
        _build_shard(out2_dir / 'sortie_19700101_000002_0.mcap', shard0)
        _build_shard(out2_dir / 'sortie_19700101_000002_1.mcap', shard1)
        merge_sortie(out2_dir, remove_shards=True, quiet=True)
        leftover = sorted(p.name for p in out2_dir.glob('*.mcap'))
        assert leftover == ['sortie_19700101_000002_full.mcap'], \
            f'--remove-shards left extra files: {leftover}'

        # --- dry-run: must not produce file ---
        out3_dir = Path(tmp) / 'sortie_19700101_000003'
        out3_dir.mkdir()
        _build_shard(out3_dir / 'sortie_19700101_000003_0.mcap', shard0)
        merge_sortie(out3_dir, dry_run=True, quiet=True)
        assert not (out3_dir / 'sortie_19700101_000003_full.mcap').exists(), \
            'dry-run should not create output'

    print('OK: merge_sortie self-test passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
