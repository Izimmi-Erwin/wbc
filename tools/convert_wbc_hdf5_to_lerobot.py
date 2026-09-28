"""Convert WBC kitchen HDF5 episodes to LeRobot v2.1 without altering input."""

import argparse
import hashlib
import json
import math
import uuid
from pathlib import Path

import cv2
import h5py
import numpy as np

LEROBOT_REVISION = '55198de096f46a8e0447a8795129dd9ee84c088c'
IMAGE = 'observations/images/ego_jpeg'
TASKS = {
    'apple': 'pick up the apple and put it on the red plate',
    'banana': 'pick up the yellow banana and put it on the red plate',
}
# Output name -> (HDF5 path, width). All poses use XYZ + wxyz.
EXTRAS = {
    'observation.joint_velocity': ('observations/joint_velocity', None),
    'observation.root_pose': ('observations/root_pose', 7),
    'observation.root_velocity': ('observations/root_velocity', 6),
    'observation.plate_pose': ('objects/plate_pose', 7),
    'observation.plate_velocity': ('objects/plate_velocity', 6),
}


def extras_for(fruit):
    if fruit not in TASKS:
        raise ValueError(f'Unsupported fruit: {fruit}')
    return {
        **EXTRAS,
        f'observation.{fruit}_pose': (f'objects/{fruit}_pose', 7),
        f'observation.{fruit}_velocity': (f'objects/{fruit}_velocity', 6),
    }


def decode_image(value):
    image = cv2.imdecode(np.asarray(value, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError('Undecodable ego JPEG')
    return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def resample_segments(times, valid, fps, max_gap, alignment):
    """Hold the latest observed row; never bridge invalid rows or long gaps."""
    if fps <= 0 or not math.isfinite(max_gap) or max_gap <= 0:
        raise ValueError('fps and max_gap must be positive')
    if alignment not in ('recorded', 'next'):
        raise ValueError('Unknown action alignment')
    if not np.isfinite(times).all() or np.any(np.diff(times) <= 0):
        raise ValueError('Wall timestamps must be finite and strictly rising')
    indices = np.flatnonzero(valid)
    if not len(indices):
        return []
    split = ((np.diff(indices) != 1) | (np.diff(times[indices]) > max_gap))
    runs = np.split(indices, np.flatnonzero(split) + 1)
    segments = []
    for run in runs:
        # A single observation cannot define a duration. Next alignment also
        # needs a following valid source row, not the next duplicated frame.
        if len(run) < (3 if alignment == 'next' else 2):
            continue
        stop = run[-2] if alignment == 'next' else run[-1]
        duration = times[stop] - times[run[0]]
        count = int(np.floor(duration * fps + 1e-6)) + 1
        relative = np.arange(count, dtype=np.float64) / fps
        source_times = times[run] - times[run[0]]
        offsets = np.searchsorted(source_times, relative, side='right') - 1
        observation = run[np.maximum(offsets, 0)]
        action = observation + (alignment == 'next')
        segments.append({
            'observation': observation,
            'action': action,
            'relative_time': relative,
            'wall_start': float(times[run[0]]),
        })
    return segments


def inspect_episode(path, fps, max_gap, alignment, fruit='apple'):
    with h5py.File(path, 'r') as data:
        if int(data.attrs.get('schema_version', -1)) != 1:
            raise ValueError('Unsupported WBC schema_version')
        metadata = json.loads(data.attrs['metadata_json'])
        if metadata.get('task_object', fruit) != fruit:
            raise ValueError(f'Episode task_object does not match {fruit}')
        if metadata.get('control_mode') != 'position':
            raise ValueError('43-DOF target conversion requires position mode')
        names = metadata['joint_names']
        if (len(names) != 43 or len(names) != len(set(names))
                or not all(isinstance(name, str) for name in names)):
            raise ValueError('joint_names must contain 43 unique names')
        times = data['time/wall'][:]
        size = len(times)
        if size < 2 or int(data.attrs['num_frames']) != size:
            raise ValueError('Invalid episode length')
        arrays = {
            'observation.state': data['observations/joint_position'][:],
            'action': data['actions/joint_position_target'][:],
        }
        widths = dict.fromkeys(arrays, len(names))
        for key, (source, width) in extras_for(fruit).items():
            arrays[key] = data[source][:]
            widths[key] = len(names) if width is None else width
        for key, array in arrays.items():
            if array.shape != (size, widths[key]):
                raise ValueError(f'Invalid shape for {key}: {array.shape}')
        paths = [
            IMAGE, 'actions/valid', 'observations/images/ego_valid',
            'manager/mode', 'time/simulation', 'actions/received_wall_time',
            'observations/images/ego_render_time'
        ]
        if any(data[key].shape != (size, ) for key in paths):
            raise ValueError('Image/flag/timestamp row counts differ')
        valid = (
            data['actions/valid'][:].astype(bool)
            & data['observations/images/ego_valid'][:].astype(bool)
            & (data['manager/mode'][:] == 1))
        for array in arrays.values():
            valid &= np.isfinite(array).all(axis=1)
        for key in ('time/simulation', 'actions/received_wall_time',
                    'observations/images/ego_render_time'):
            valid &= np.isfinite(data[key][:])
        shape = None
        for row in np.flatnonzero(valid):
            try:
                image = decode_image(data[IMAGE][row])
            except (ValueError, cv2.error):
                valid[row] = False
                continue
            if shape is None:
                shape = image.shape
            if image.shape != shape:
                raise ValueError('Ego image size changed within episode')
        segments = resample_segments(times, valid, fps, max_gap, alignment)
        if not segments:
            raise ValueError('No continuous valid segment to convert')
        return {
            'path': path,
            'fruit': fruit,
            'metadata': metadata,
            'names': names,
            'image_shape': shape,
            'segments': segments,
            'summary': {
                'source': str(path.resolve()),
                'task_object': fruit,
                'sha256': sha256(path),
                'outcome': str(data.attrs['outcome']),
                'success': bool(data.attrs['success']),
                'source_rows': size,
                'valid_rows': int(valid.sum()),
                'wall_duration_s': float(times[-1] - times[0]),
                'median_source_interval_s': float(np.median(np.diff(times))),
                'output_segments': len(segments),
                'output_frames': sum(len(s['action']) for s in segments),
            },
        }


def scan(raw_dir, outcomes, fps, max_gap, alignment, limit=None, fruit='apple'):
    episodes, skipped = [], []
    for path in sorted(raw_dir.rglob('episode_*.hdf5')):
        if '.partial.' in path.name:
            skipped.append({'source': str(path), 'reason': 'partial file'})
            continue
        try:
            with h5py.File(path, 'r') as data:
                outcome = str(data.attrs.get('outcome', 'unknown'))
                if not bool(data.attrs.get('complete', False)):
                    raise ValueError('incomplete episode')
                if outcome not in outcomes:
                    raise ValueError(f'outcome excluded: {outcome}')
                if (outcome == 'success') != bool(data.attrs['success']):
                    raise ValueError('success flag disagrees with outcome')
            episode = inspect_episode(path, fps, max_gap, alignment, fruit)
        except (OSError, KeyError, ValueError, TypeError) as error:
            skipped.append({'source': str(path), 'reason': str(error)})
            continue
        if episodes:
            if episode['names'] != episodes[0]['names']:
                raise ValueError(f'Joint order differs in {path}')
            if episode['image_shape'] != episodes[0]['image_shape']:
                raise ValueError(f'Camera shape differs in {path}')
        episodes.append(episode)
        if limit is not None and len(episodes) >= limit:
            break
    return episodes, skipped


def features_for(episode):
    names = episode['names']
    features = {
        'observation.state': {
            'dtype': 'float32',
            'shape': (len(names), ),
            'names': names
        },
        'action': {
            'dtype': 'float32',
            'shape': (len(names), ),
            'names': names
        },
        'observation.images.ego': {
            'dtype': 'video',
            'shape': episode['image_shape'],
            'names': ['height', 'width', 'channels']
        },
    }
    for key, (_, width) in extras_for(episode['fruit']).items():
        features[key] = {
            'dtype': 'float32',
            'shape': (len(names) if width is None else width, ),
            'names': names if width is None else None,
        }
    for key in ('observation_row', 'action_row'):
        features[f'source.{key}'] = {
            'dtype': 'int64',
            'shape': (1, ),
            'names': None
        }
    for key in ('observation_wall_time', 'action_wall_time', 'simulation_time',
                'action_received_wall_time', 'ego_render_time'):
        features[f'source.{key}'] = {
            'dtype': 'float64',
            'shape': (1, ),
            'names': None
        }
    return features


def write_dataset(episodes, output, repo_id, fps, task, report):
    # Keep heavy LeRobot imports out of --help and --dry-run.
    from hdf_to_lerobot_direct_video import enable_direct_video_patch
    from lerobot.datasets.lerobot_dataset import (CODEBASE_VERSION,
                                                  LeRobotDataset)

    if CODEBASE_VERSION != 'v2.1':
        raise ValueError(f'Expected LeRobot v2.1, found {CODEBASE_VERSION}')
    if output.exists():
        raise FileExistsError(f'Refusing to overwrite {output}')
    output.parent.mkdir(parents=True, exist_ok=True)
    staging = output.with_name(
        f'.{output.name}.partial-{uuid.uuid4().hex[:8]}')
    dataset = LeRobotDataset.create(
        repo_id=repo_id,
        root=staging,
        fps=fps,
        features=features_for(episodes[0]),
        robot_type='sonic_g1_43dof',
        video_backend='pyav',
    )
    dataset = enable_direct_video_patch(dataset, codec='libx264')
    report['episodes'] = []
    try:
        for episode in episodes:
            if sha256(episode['path']) != episode['summary']['sha256']:
                raise ValueError('Source changed after inspection')
            with h5py.File(episode['path'], 'r') as data:
                for segment in episode['segments']:
                    index = dataset.num_episodes
                    cached_row, rgb = None, None
                    for row, action_row in zip(segment['observation'],
                                               segment['action']):
                        row, action_row = int(row), int(action_row)
                        if cached_row != row:
                            rgb = decode_image(data[IMAGE][row])
                            cached_row = row
                        frame = {
                            'observation.state':
                            np.asarray(
                                data['observations/joint_position'][row],
                                dtype=np.float32),
                            'action':
                            np.asarray(
                                data['actions/joint_position_target']
                                [action_row],
                                dtype=np.float32),
                            'observation.images.ego':
                            rgb,
                            'source.observation_row':
                            np.array([row]),
                            'source.action_row':
                            np.array([action_row]),
                        }
                        for key, (source, _) in extras_for(episode['fruit']).items():
                            frame[key] = np.asarray(
                                data[source][row], dtype=np.float32)
                        provenance = {
                            'observation_wall_time':
                            data['time/wall'][row],
                            'action_wall_time':
                            data['time/wall'][action_row],
                            'simulation_time':
                            data['time/simulation'][row],
                            'action_received_wall_time':
                            data['actions/received_wall_time'][action_row],
                            'ego_render_time':
                            data['observations/images/ego_render_time'][row],
                        }
                        for key, value in provenance.items():
                            frame[f'source.{key}'] = np.array([value],
                                                              dtype=np.float64)
                        dataset.add_frame(frame, task)
                    dataset.save_episode()
                    report['episodes'].append({
                        'episode_index':
                        index,
                        'source':
                        episode['summary'],
                        'source_metadata':
                        episode['metadata'],
                        'wall_start':
                        segment['wall_start'],
                        'first_source_row':
                        int(segment['observation'][0]),
                        'last_source_row':
                        int(segment['observation'][-1]),
                        'frames':
                        len(segment['action']),
                    })
                    frame_count = len(segment['action'])
                    print(
                        f'Saved episode {index}: {frame_count} frames',
                        flush=True)
        # Exercise the official reader, including actual MP4 decoding.
        loaded = LeRobotDataset(repo_id, root=staging, video_backend='pyav')
        for boundaries in zip(loaded.episode_data_index['from'],
                              loaded.episode_data_index['to']):
            for position in (int(boundaries[0]), int(boundaries[1]) - 1):
                item = loaded[position]
                if tuple(item['observation.images.ego'].shape) != (
                        3, *episodes[0]['image_shape'][:2]):
                    raise ValueError('LeRobot reader image shape mismatch')
        report['validation'] = {
            'official_reader': True,
            'episodes': loaded.num_episodes,
            'frames': loaded.num_frames,
            'video_boundary_frames': 'passed',
        }
        (staging / 'meta/wbc_conversion.json'
         ).write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
        if output.exists():
            raise FileExistsError(
                f'Output appeared during conversion: {output}')
        staging.rename(output)
    except BaseException:
        # Preserve only this attempt for diagnosis, never delete user data.
        for writer in dataset._direct_video_writers.values():
            writer.close()
        print(f'Incomplete conversion retained at {staging}', flush=True)
        raise
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('raw_dir', type=Path)
    parser.add_argument(
        '--output', type=Path, help='New dataset directory; never overwritten')
    parser.add_argument('--repo-id', help='Default: local/wbc_<fruit>')
    parser.add_argument('--fruit', choices=tuple(TASKS), default='apple')
    parser.add_argument('--fps', type=int, default=30)
    parser.add_argument('--task', help='Default: the selected fruit placement task')
    parser.add_argument(
        '--outcomes',
        nargs='+',
        choices=['success', 'timeout'],
        default=['success'])
    parser.add_argument(
        '--action-alignment', choices=['recorded', 'next'], default='recorded')
    parser.add_argument(
        '--max-gap',
        type=float,
        default=0.5,
        help='Split at source gaps longer than this (seconds)')
    parser.add_argument(
        '--limit', type=int, help='Limit accepted source files')
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    args.task = args.task or TASKS[args.fruit]
    args.repo_id = args.repo_id or f'local/wbc_{args.fruit}'
    if not args.raw_dir.is_dir():
        parser.error('raw_dir must be an existing directory')
    if (not 1 <= args.fps <= 200 or not math.isfinite(args.max_gap)
            or args.max_gap <= 0
            or (args.limit is not None and args.limit < 1)):
        parser.error('Use fps 1..200, positive max-gap and positive limit')
    if not args.dry_run:
        if args.output is None:
            parser.error('--output is required for conversion')
        if args.output.exists():
            parser.error('Output already exists; choose a new directory')
        if args.output.resolve().is_relative_to(args.raw_dir.resolve()):
            parser.error('Output must be outside the raw HDF5 directory')
    episodes, skipped = scan(args.raw_dir, args.outcomes, args.fps,
                             args.max_gap, args.action_alignment, args.limit, args.fruit)
    report = {
        'format':
        'LeRobot v2.1',
        'lerobot_revision':
        LEROBOT_REVISION,
        'fps':
        args.fps,
        'task':
        args.task,
        'fruit':
        args.fruit,
        'outcomes':
        args.outcomes,
        'action_alignment':
        args.action_alignment,
        'time_basis':
        'time/wall; zero-order hold of paired source rows',
        'max_gap_s':
        args.max_gap,
        'limitations': [
            'Repeated frames do not create additional observed motion.',
            'recorded actions were applied before their paired observation.',
            'next uses the next recorded row, not the immediate next command.',
            'Wall timestamps are row times, not exact camera exposure times.',
        ],
        'accepted': [ep['summary'] for ep in episodes],
        'skipped':
        skipped,
    }
    print(json.dumps(report, indent=2, ensure_ascii=False))
    if args.dry_run:
        return
    if not episodes:
        parser.error('No eligible episodes; inspect --dry-run output')
    path = write_dataset(episodes, args.output.resolve(), args.repo_id,
                         args.fps, args.task, report)
    print(f'Converted dataset: {path}')


if __name__ == '__main__':
    main()
