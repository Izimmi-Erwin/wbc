"""Data integrity checks without Isaac GPU, DDS, or the teleop manager."""
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import cv2
import h5py
import numpy as np

from gear_sonic.simulation_server.episode_cameras import DEFAULT_WRIST_CONFIG, EpisodeRGBCamera
from gear_sonic.simulation_server.kitchen_episodes import Hdf5Episode, KitchenEpisodeCollector
from tools.convert_wbc_hdf5_to_lerobot import CAMERAS, extras_for, features_for, inspect_episode, scan


def make_episode(directory, cameras=CAMERAS):
    metadata = {'task_object': 'apple', 'control_mode': 'position',
                'joint_names': [f'joint_{i}' for i in range(43)],
                'synthetic_test_only': True,
                'cameras': json.loads(DEFAULT_WRIST_CONFIG.read_text())}
    writer = Hdf5Episode(directory, metadata)
    for i in range(12):
        row = {'time/wall': 1000. + i / 30., 'time/simulation': i / 30.,
               'actions/received_wall_time': 1000. + i / 30. - .001,
               'observations/joint_position': np.full(43, i / 100., dtype=np.float32),
               'actions/joint_position_target': np.full(43, (i + 1) / 100., dtype=np.float32),
               'actions/valid': True, 'manager/mode': 1}
        for source, width in extras_for('apple').values():
            row[source] = np.zeros(width or 43, dtype=np.float32)
        for j, name in enumerate(cameras):
            # Different moving colored patches catch swapped or duplicated views.
            image = np.full((480, 640, 3), 24, np.uint8)
            image[80:320, 20 + i * 8:200 + i * 8, j] = 220
            ok, jpeg = cv2.imencode('.jpg', image)
            assert ok
            prefix = f'observations/images/{name}'
            row.update({f'{prefix}_jpeg': jpeg.tobytes(), f'{prefix}_valid': True,
                        f'{prefix}_render_frame': i + 20, f'{prefix}_render_time': i / 30.})
            if name != 'ego':
                row[f'observations/cameras/{name}_world_pose'] = np.array([j, 0, 0, 1, 0, 0, 0], np.float32)
        writer.append(row)
    return writer.finish('timeout')


class CameraTests(unittest.TestCase):
    def test_reads_pixels_from_timestamped_callback_and_caches_jpeg(self):
        camera = EpisodeRGBCamera()
        camera.metadata = {'resolution': [64, 48]}
        frame = {'rendering_frame': 10, 'rendering_time': .1, 'rgba': np.full((48, 64, 4), 80, np.uint8)}
        camera.camera = Mock()
        camera.camera.get_current_frame.return_value = frame
        with patch('cv2.imencode', wraps=cv2.imencode) as encode:
            first = camera.read()
            self.assertTrue(first[1])
            self.assertEqual(camera.read(), first)
            self.assertEqual(encode.call_count, 1)
            frame.update(rendering_frame=11, rendering_time=.2)
            camera.read()
            self.assertEqual(encode.call_count, 2)
        camera.camera.get_rgba.assert_not_called()

    def test_begin_waits_for_both_wrist_frames(self):
        collector = KitchenEpisodeCollector(SimpleNamespace(), '/unused')
        collector.camera = Mock()
        collector.camera.read.return_value = (b'ego', True, 12, .5)
        wrist = Mock()
        collector.wrist_cameras = {'left_wrist': wrist, 'right_wrist': wrist}
        for sample in ((b'', False, -1, -1.), (b'old', True, 11, .4)):
            wrist.read.return_value = sample
            collector._begin()
            self.assertIsNone(collector.writer)
            self.assertIsNone(collector.recording_started_at)


class ConversionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = make_episode(self.temp.name)

    def inspect(self, cameras=None):
        return inspect_episode(self.path, 30, .5, 'recorded', cameras=cameras)

    def test_three_views_roundtrip_bytes_and_pose_features(self):
        episode = self.inspect()
        self.assertEqual(episode['cameras'], CAMERAS)
        self.assertEqual(episode['summary']['valid_rows'], 12)
        features = features_for(episode)
        for name in CAMERAS:
            self.assertEqual(features[f'observation.images.{name}']['shape'], (480, 640, 3))
            with h5py.File(self.path) as data:
                self.assertEqual(data[f'observations/images/{name}_jpeg'].dtype, np.dtype('O'))
        self.assertIn('observation.left_wrist_camera_pose', features)

    def test_corrupt_or_unsynchronized_view_splits_all_streams(self):
        with h5py.File(self.path, 'r+') as data:
            data['observations/images/left_wrist_jpeg'][3] = np.array([1, 2, 3], np.uint8)
            data['observations/images/right_wrist_render_frame'][6] = 999
            data['observations/images/right_wrist_valid'][9] = False
        episode = self.inspect()
        self.assertEqual(episode['summary']['valid_rows'], 9)
        used = np.concatenate([segment['observation'] for segment in episode['segments']])
        for bad in (3, 6, 9):
            self.assertNotIn(bad, used)

    def test_legacy_ego_and_explicit_wrist_selection(self):
        old_dir = Path(self.temp.name) / 'old'
        old = make_episode(old_dir, cameras=('ego',))
        episode = inspect_episode(old, 30, .5, 'recorded')
        self.assertEqual(episode['cameras'], ('ego',))
        with self.assertRaisesRegex(ValueError, 'Missing requested'):
            inspect_episode(old, 30, .5, 'recorded', cameras=CAMERAS)
        self.assertEqual(self.inspect(('left_wrist', 'right_wrist'))['cameras'], ('left_wrist', 'right_wrist'))
        with self.assertRaisesRegex(ValueError, 'Camera sets'):
            scan(Path(self.temp.name), ['timeout'], 30, .5, 'recorded')
        episodes, skipped = scan(Path(self.temp.name), ['timeout'], 30, .5, 'recorded', cameras=('ego',))
        self.assertEqual(len(episodes), 2)
        self.assertFalse(skipped)

    def test_no_valid_wrist_frames_does_not_make_dataset(self):
        with h5py.File(self.path, 'r+') as data:
            data['observations/images/left_wrist_valid'][:] = False
        with self.assertRaisesRegex(ValueError, 'No continuous'):
            self.inspect()


if __name__ == '__main__':
    unittest.main()
