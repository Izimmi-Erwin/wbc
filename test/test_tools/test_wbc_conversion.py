"""WBC conversion checks: alignment, data validity and input preservation."""

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import cv2
import h5py
import numpy as np

SCRIPT = (
    Path(__file__).resolve().parents[2] /
    'tools/convert_wbc_hdf5_to_lerobot.py')
SPEC = importlib.util.spec_from_file_location('wbc_converter', SCRIPT)
converter = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(converter)


def make_episode(path, outcome='success', times=None):
    times = np.array([0., .1, .2, .3]) if times is None else np.array(times)
    size = len(times)
    with h5py.File(path, 'w') as data:
        data.attrs.update({
            'schema_version':
            1,
            'complete':
            True,
            'success':
            outcome == 'success',
            'outcome':
            outcome,
            'num_frames':
            size,
            'metadata_json':
            json.dumps({
                'control_mode': 'position',
                'joint_names': [f'joint_{i}' for i in range(43)],
                'robot_model': 'sonic_g1_43dof',
            }),
        })
        data['time/wall'] = times + 1000
        data['time/simulation'] = times
        data['actions/received_wall_time'] = times + 999.99
        data['observations/images/ego_render_time'] = times
        data['observations/joint_position'] = np.zeros((size, 43))
        data['actions/joint_position_target'] = np.broadcast_to(
            np.arange(size, dtype=np.float32)[:, None], (size, 43)).copy()
        data['actions/valid'] = np.ones(size, dtype=bool)
        data['observations/images/ego_valid'] = np.ones(size, dtype=bool)
        data['manager/mode'] = np.ones(size, dtype=np.int32)
        for source, width in converter.EXTRAS.values():
            data[source] = np.zeros((size, 43 if width is None else width))
        images = data.create_dataset(
            converter.IMAGE, (size, ), dtype=h5py.vlen_dtype(np.uint8))
        for row in range(size):
            rgb = np.full((48, 64, 3), (row * 20, 30, 180), dtype=np.uint8)
            images[row] = cv2.imencode('.jpg', rgb)[1]


class TimeTests(unittest.TestCase):

    def sample(self, times, valid=None, alignment='recorded'):
        times = np.array(times)
        if valid is None:
            valid = np.ones(len(times), dtype=bool)
        return converter.resample_segments(times, np.array(valid), 20, .5,
                                           alignment)

    def test_hold_keeps_paired_source_rows(self):
        segment = self.sample([0., .1, .3])[0]
        np.testing.assert_array_equal(segment['observation'],
                                      [0, 0, 1, 1, 1, 1, 2])
        np.testing.assert_array_equal(segment['observation'],
                                      segment['action'])
        self.assertEqual(segment['relative_time'][-1], .3)

    def test_next_means_next_source_not_next_resampled_frame(self):
        segment = self.sample([0., .1, .3], alignment='next')[0]
        np.testing.assert_array_equal(segment['observation'], [0, 0, 1])
        np.testing.assert_array_equal(segment['action'], [1, 1, 2])

    def test_invalid_rows_and_long_gaps_split_episodes(self):
        segments = self.sample([0., .1, .2, .3, .4, 1., 1.1],
                               [True, True, False, True, True, True, True])
        self.assertEqual(len(segments), 3)
        self.assertEqual([s['wall_start'] for s in segments], [0., .3, 1.])
        self.assertTrue(all(2 not in s['observation'] for s in segments))

    def test_bad_clock_is_rejected(self):
        for times in ([0., 0.], [1., 0.], [0., np.nan]):
            with self.subTest(times=times), self.assertRaises(ValueError):
                self.sample(times)

    def test_single_sample_has_no_fabricated_duration(self):
        self.assertEqual(self.sample([1.]), [])


class EpisodeTests(unittest.TestCase):

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.path = self.root / 'episode_0.hdf5'
        make_episode(self.path)

    def tearDown(self):
        self.temp.cleanup()

    def test_success_only_and_no_input_mutation(self):
        make_episode(self.root / 'episode_1.hdf5', outcome='timeout')
        (self.root / 'episode_2.partial.hdf5').write_bytes(b'open recording')
        original = self.path.read_bytes()
        episodes, skipped = converter.scan(self.root, ['success'], 30, .5,
                                           'recorded')
        self.assertEqual(len(episodes), 1)
        self.assertEqual(len(skipped), 2)
        self.assertEqual(self.path.read_bytes(), original)
        features = converter.features_for(episodes[0])
        self.assertEqual(features['action']['shape'], (43, ))
        self.assertEqual(features['observation.images.ego']['shape'],
                         (48, 64, 3))

    def test_bad_action_or_image_is_not_a_training_row(self):
        with h5py.File(self.path, 'r+') as data:
            data['actions/joint_position_target'][0] = np.nan
            data[converter.IMAGE][3] = np.array([1, 2, 3], np.uint8)
        episode = converter.inspect_episode(self.path, 20, .5, 'recorded')
        self.assertEqual(episode['summary']['valid_rows'], 2)
        self.assertTrue(
            all(
                row in (1, 2)
                for row in episode['segments'][0]['observation']))

    def test_effort_mode_is_rejected(self):
        with h5py.File(self.path, 'r+') as data:
            meta = json.loads(data.attrs['metadata_json'])
            meta['control_mode'] = 'effort'
            data.attrs['metadata_json'] = json.dumps(meta)
        with self.assertRaisesRegex(ValueError, 'position mode'):
            converter.inspect_episode(self.path, 20, .5, 'recorded')

    def test_joint_order_mismatch_is_not_silently_accepted(self):
        other = self.root / 'episode_1.hdf5'
        make_episode(other)
        with h5py.File(other, 'r+') as data:
            meta = json.loads(data.attrs['metadata_json'])
            meta['joint_names'].reverse()
            data.attrs['metadata_json'] = json.dumps(meta)
        with self.assertRaisesRegex(ValueError, 'Joint order'):
            converter.scan(self.root, ['success'], 30, .5, 'recorded')

    def test_incomplete_and_inconsistent_success_are_excluded(self):
        with h5py.File(self.path, 'r+') as data:
            data.attrs['success'] = False
        episodes, skipped = converter.scan(self.root, ['success'], 30, .5,
                                           'recorded')
        self.assertEqual(episodes, [])
        self.assertIn('disagrees', skipped[0]['reason'])
        with h5py.File(self.path, 'r+') as data:
            data.attrs['complete'] = False
        episodes, skipped = converter.scan(self.root, ['success'], 30, .5,
                                           'recorded')
        self.assertEqual(episodes, [])
        self.assertIn('incomplete', skipped[0]['reason'])

    @unittest.skipUnless(
        importlib.util.find_spec('lerobot'),
        'Requires the isolated LeRobot environment')
    def test_official_reader_video_stats_and_no_overwrite(self):
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
        from lerobot.datasets.video_utils import decode_video_frames

        episodes, skipped = converter.scan(self.root, ['success'], 20, .5,
                                           'recorded')
        output = self.root / 'converted'
        before = self.path.read_bytes()
        with patch.object(sys, 'path', [str(SCRIPT.parent), *sys.path]):
            converter.write_dataset(episodes, output, 'local/test_wbc', 20,
                                    'test task', {'skipped': skipped})
            with self.assertRaises(FileExistsError):
                converter.write_dataset(episodes, output, 'local/test_wbc', 20,
                                        'test task', {})
        loaded = LeRobotDataset(
            'local/test_wbc', root=output, video_backend='pyav')
        self.assertEqual(loaded.num_episodes, 1)
        for index in range(len(loaded)):
            row = loaded[index]
            source = int(row['source.action_row'].item())
            np.testing.assert_array_equal(row['action'].numpy(),
                                          np.full(43, source))
            self.assertEqual(tuple(row['observation.state'].shape), (43, ))
        video = output / loaded.meta.get_video_file_path(
            0, 'observation.images.ego')
        frames = decode_video_frames(video,
                                     [i / 20 for i in range(len(loaded))],
                                     1e-4, 'pyav')
        stats = loaded.meta.stats['observation.images.ego']
        expected = frames.numpy().mean(axis=(0, 2, 3))[:, None, None]
        np.testing.assert_allclose(stats['mean'], expected, atol=1e-5)
        self.assertGreater(float(np.max(stats['max'])), .5)
        self.assertEqual(self.path.read_bytes(), before)


if __name__ == '__main__':
    unittest.main()
