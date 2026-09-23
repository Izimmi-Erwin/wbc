import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import h5py
import numpy as np
import zmq

from gear_sonic.simulation_server.kitchen_episodes import Hdf5Episode, KitchenEpisodeCollector, PlacementDetector
from gear_sonic.utils.teleop.episode_control import EpisodeControlClient, EpisodeControlServer


class PlacementTests(unittest.TestCase):
    def test_contact_release_and_continuous_dwell(self):
        detector = PlacementDetector()
        good = dict(inside=True, upright=True, plate_contact=1., hand_contact=0.,
                    fruit_speed=0., plate_speed=0., fruit_angular_speed=0.)
        for bad in (dict(inside=False), dict(upright=False), dict(plate_contact=0.),
                    dict(hand_contact=1.), dict(fruit_speed=.1), dict(fruit_angular_speed=1.)):
            self.assertFalse(detector.update(0., **good))
            self.assertFalse(detector.update(.4, **(good | bad)))
            self.assertFalse(detector.update(.7, **good))
            self.assertTrue(detector.update(1.31, **good))
            detector.since = None


class FileTests(unittest.TestCase):
    def test_atomic_final_file_and_aligned_rows(self):
        with tempfile.TemporaryDirectory() as directory:
            writer = Hdf5Episode(directory, {"quaternion_order": "wxyz"})
            for i in range(3):
                writer.append({"time/simulation": np.float64(i / 30),
                               "observations/joint_position": np.arange(43, dtype=np.float32),
                               "observations/images/ego_jpeg": bytes([i, 1, 2])})
            self.assertTrue(writer.path.exists())
            path = writer.finish("success")
            self.assertFalse(writer.path.exists())
            with h5py.File(path) as file:
                self.assertTrue(file.attrs["complete"])
                self.assertTrue(file.attrs["success"])
                self.assertEqual(file["observations/joint_position"].shape, (3, 43))
                np.testing.assert_array_equal(file["observations/images/ego_jpeg"][2], [2, 1, 2])

    def test_interrupt_keeps_partial_and_never_marks_success(self):
        with tempfile.TemporaryDirectory() as directory:
            writer = Hdf5Episode(directory, {})
            writer.append({"time/simulation": 0.})
            writer.interrupt()
            with h5py.File(writer.path) as file:
                self.assertFalse(file.attrs["complete"])
                self.assertFalse(file.attrs["success"])

    def test_timeout_file_is_complete_but_unsuccessful(self):
        with tempfile.TemporaryDirectory() as directory:
            writer = Hdf5Episode(directory, {})
            writer.append({"time/simulation": 0.})
            with h5py.File(writer.finish("timeout")) as file:
                self.assertTrue(file.attrs["complete"])
                self.assertFalse(file.attrs["success"])
                self.assertEqual(file.attrs["outcome"], "timeout")

    def test_discard_removes_only_current_partial_file(self):
        with tempfile.TemporaryDirectory() as directory:
            previous = Hdf5Episode(directory, {})
            previous.append({"time/simulation": 0.})
            saved = previous.finish("success")
            writer = Hdf5Episode(directory, {})
            writer.append({"time/simulation": 1.})
            writer.discard()
            self.assertFalse(writer.file.id.valid)
            self.assertEqual(list(Path(directory).iterdir()), [saved])


class ControlTests(unittest.TestCase):
    def setUp(self):
        self.server = EpisodeControlServer(0)
        port = int(self.server.socket.getsockopt(zmq.LAST_ENDPOINT).decode().rsplit(":", 1)[1])
        self.client = EpisodeControlClient(port)
        self.server.published("POSE")

    def tearDown(self):
        self.client.close()
        self.server.close()

    def exchange(self, operation, token=None, ax=False):
        self.client.request(operation, token)
        self.assertTrue(self.server.socket.poll(1000))
        self.server.poll()
        mode = self.server.constrain(self.server.mode, self.server.mode, ax_rising=ax)
        self.server.published(mode)
        self.assertTrue(self.client.socket.poll(1000))
        return self.client.poll()

    def test_hold_is_idempotent_and_release_requires_manual_edge(self):
        held = self.exchange("hold", "episode-1")
        self.assertEqual(held["mode"], "PLANNER")
        again = self.exchange("hold", "episode-1", ax=True)
        self.assertEqual(again["mode"], "PLANNER")
        self.assertGreaterEqual(again["hold_ticks"], 2)
        self.assertFalse(self.exchange("release", "wrong")["ok"])
        released = self.exchange("release", "episode-1", ax=True)
        self.assertTrue(released["wait_for_ax"])
        self.assertEqual(released["mode"], "PLANNER")
        self.assertEqual(self.server.constrain("PLANNER", "POSE", ax_rising=False), "PLANNER")
        self.assertEqual(self.server.constrain("PLANNER", "POSE", ax_rising=True), "POSE")
        self.server.published("POSE")
        retry = self.exchange("release", "episode-1")
        self.assertTrue(retry["ok"])
        self.assertEqual(retry["mode"], "POSE")
        self.assertEqual(retry["pose_epoch"], 2)

    def test_emergency_has_priority(self):
        self.exchange("hold", "episode-1")
        self.assertEqual(self.server.constrain("PLANNER", "POSE", ax_rising=True, emergency=True), "OFF")
        self.assertEqual(self.server.constrain("OFF", "OFF", ax_rising=False), "OFF")


class CommandBoundaryTests(unittest.TestCase):
    def test_original_controller_commands_work_after_reset_discard(self):
        from gear_sonic.robot_interface.isaac_unitree_bridge import IsaacUnitreeBridge

        articulation = Mock()
        articulation.get_joint_positions.return_value = np.zeros(43)
        bridge = IsaacUnitreeBridge(
            articulation=articulation,
            joint_mapping=SimpleNamespace(sonic_body_to_isaac_dof=list(range(29))),
            verbose=False,
        )
        bridge._apply_isaac_position_targets = Mock()
        motor = SimpleNamespace(q=.1, dq=0., kp=10., kd=1., tau=0., mode=1)
        message = SimpleNamespace(motor_cmd=[motor] * 29, reserve=[0] * 4)
        bridge._low_cmd_handler(message)
        old = bridge.receive_low_cmd()
        bridge.discard_episode_commands()
        bridge.apply_joint_command(old)
        bridge._apply_isaac_position_targets.assert_not_called()
        for _ in range(2):
            bridge._low_cmd_handler(message)
            current = bridge.receive_low_cmd()
            self.assertIsNotNone(current)
            bridge.apply_joint_command(current)
            self.assertIs(bridge.last_applied_command, current)
            hand = SimpleNamespace(motor_cmd=[SimpleNamespace(q=.5)] * 7)
            bridge._left_hand_cmd_handler(hand)
            bridge._right_hand_cmd_handler(hand)
            self.assertIs(bridge.left_hand_cmd, hand)
            self.assertIs(bridge.right_hand_cmd, hand)
            bridge.discard_episode_commands()
        np.testing.assert_allclose(
            bridge._apply_isaac_position_targets.call_args.args[0][:29], .1)

    def test_fresh_body_does_not_resurrect_old_hand_targets(self):
        from gear_sonic.robot_interface.isaac_unitree_bridge import IsaacUnitreeBridge

        bridge = IsaacUnitreeBridge(robot_asset=SimpleNamespace(hand_dof_available=True), verbose=False)
        hand = SimpleNamespace(motor_cmd=[SimpleNamespace(q=.5) for _ in range(7)])
        bridge._left_hand_cmd_handler(hand)
        bridge._right_hand_cmd_handler(hand)
        self.assertEqual(set(bridge._latest_dex3_hand_targets()), {"left", "right"})
        cutoff = bridge.discard_episode_commands()
        motor = SimpleNamespace(q=0., dq=0., kp=10., kd=1., tau=0., mode=0)
        bridge._low_cmd_handler(SimpleNamespace(motor_cmd=[motor] * 29))
        self.assertIsNotNone(bridge.peek_low_cmd())
        self.assertEqual(bridge._latest_dex3_hand_targets(), {})
        with patch("gear_sonic.robot_interface.isaac_unitree_bridge.time.time", return_value=cutoff - 1):
            bridge._left_hand_cmd_handler(hand)
        self.assertEqual(bridge._latest_dex3_hand_targets(), {})
        bridge._right_hand_cmd_handler(hand)
        self.assertEqual(set(bridge._latest_dex3_hand_targets()), {"right"})


class CoordinatorTests(unittest.TestCase):
    def make_collector(self):
        backend = SimpleNamespace(mujoco_backspace_reset=Mock(), dds_bridge=None)
        collector = KitchenEpisodeCollector(backend, "/tmp/not-written-by-test")
        collector.phase = "STOPPING"
        collector.token = "episode-1"
        collector.session = "manager-1"
        collector.writer = Mock()
        collector.writer.finish.return_value = Path("episode.hdf5")
        collector.client = Mock(pending=None)
        return collector, backend

    def reply(self, **fields):
        return dict(ok=True, session="manager-1", mode="PLANNER", hold_token="episode-1",
                    hold_ticks=2, pose_epoch=1, operation="hold") | fields

    def test_manual_backspace_discards_resets_and_restarts_without_manager_hold(self):
        collector, backend = self.make_collector()
        collector.phase = "RECORDING"
        collector.recording_started_at = 1.
        collector.client.poll.return_value = None
        writer = collector.writer
        def begin():
            collector.phase = "RECORDING"
            collector.recording_started_at = 200.
        collector._begin = Mock(side_effect=begin)
        self.assertTrue(collector.request_manual_reset())
        with patch("gear_sonic.simulation_server.kitchen_episodes.time.monotonic", return_value=200.):
            collector.before_step()
        backend.mujoco_backspace_reset.assert_called_once()
        writer.discard.assert_called_once()
        writer.finish.assert_not_called()
        collector._begin.assert_called_once()
        self.assertEqual(collector.phase, "RECORDING")
        self.assertEqual(collector.recording_started_at, 200.)
        self.assertEqual(collector.saved_paths, [])
        self.assertFalse(any(c.args[0] == "hold" for c in collector.client.request.call_args_list))

    def test_manual_reset_waits_for_camera_without_empty_writer_sampling(self):
        collector, backend = self.make_collector()
        collector.phase = "RECORDING"
        collector.client.poll.return_value = None
        collector._begin = Mock()  # Camera not ready, no recording started yet.
        collector.request_manual_reset()
        collector.before_step()
        self.assertEqual(collector.phase, "RESTARTING")
        self.assertTrue(collector.frozen)
        self.assertIsNone(collector.writer)
        collector.after_step(None, None)
        backend.mujoco_backspace_reset.assert_called_once()

    def test_manual_reset_during_pending_automatic_hold_discards_instead_of_saves(self):
        collector, backend = self.make_collector()
        writer = collector.writer
        collector.request_manual_reset()
        collector.client.poll.return_value = self.reply()
        collector.before_step()
        writer.discard.assert_called_once()
        writer.finish.assert_not_called()
        backend.mujoco_backspace_reset.assert_called_once()
        self.assertEqual(collector.phase, "RELEASING")

    def test_manual_reset_while_waiting_does_not_start_recording(self):
        collector, backend = self.make_collector()
        collector.phase = "WAITING"
        collector.writer = None
        collector.client.poll.return_value = None
        collector._begin = Mock()
        collector.request_manual_reset()
        collector.before_step()
        backend.mujoco_backspace_reset.assert_called_once()
        self.assertEqual(collector.phase, "WAITING")
        collector._begin.assert_not_called()

    def test_no_reset_without_confirmed_hold(self):
        collector, backend = self.make_collector()
        collector.client.poll.return_value = None
        collector.before_step()
        backend.mujoco_backspace_reset.assert_not_called()
        collector.client.poll.return_value = self.reply(hold_ticks=1)
        collector.before_step()
        backend.mujoco_backspace_reset.assert_not_called()
        collector.client.poll.return_value = self.reply()
        collector.before_step()
        backend.mujoco_backspace_reset.assert_called_once()
        self.assertEqual(collector.phase, "RELEASING")
        collector.before_step()
        backend.mujoco_backspace_reset.assert_called_once()

    def test_timeout_uses_wall_time_and_existing_hold_reset_wait_flow(self):
        collector, backend = self.make_collector()
        collector.phase = "RECORDING"
        collector.recording_started_at = 100.
        collector.status_at = 100.
        backend._sim_time = 9999.
        collector.client.poll.return_value = None
        writer = collector.writer
        with patch("gear_sonic.simulation_server.kitchen_episodes.time.monotonic", return_value=279.999):
            collector.before_step()
        self.assertEqual(collector.phase, "RECORDING")
        with patch("gear_sonic.simulation_server.kitchen_episodes.time.monotonic", return_value=280.):
            collector.before_step()
        self.assertEqual(collector.phase, "STOPPING")
        collector.client.request.assert_called_with("hold", "episode-1")
        backend.mujoco_backspace_reset.assert_not_called()
        writer.finish.assert_not_called()
        collector.client.poll.return_value = self.reply(hold_ticks=1)
        collector.before_step()
        backend.mujoco_backspace_reset.assert_not_called()
        collector.client.poll.return_value = self.reply()
        collector.before_step()
        writer.finish.assert_called_once_with("timeout")
        backend.mujoco_backspace_reset.assert_called_once()
        collector.client.poll.return_value = self.reply(operation="release", hold_token=None,
                                                       released_token="episode-1")
        collector.before_step()
        self.assertEqual(collector.phase, "WAITING")
        collector._begin = Mock()
        collector.before_step()
        collector._begin.assert_not_called()
        collector.client.poll.return_value = self.reply(mode="POSE", pose_epoch=2, hold_token=None)
        collector.before_step()
        collector._begin.assert_called_once()

    def test_timeout_does_not_override_success_or_inactive_phases(self):
        for phase in ("IDLE", "WAITING", "STOPPING", "ERROR"):
            with self.subTest(phase=phase):
                collector, backend = self.make_collector()
                collector.phase = phase
                collector.recording_started_at = 0.
                collector.client.poll.return_value = None
                with patch("gear_sonic.simulation_server.kitchen_episodes.time.monotonic", return_value=1000.):
                    collector.before_step()
                self.assertEqual(collector.phase, phase)
                self.assertEqual(collector.outcome, "success")
                backend.mujoco_backspace_reset.assert_not_called()

    def test_each_recording_starts_a_fresh_timer_after_camera_ready(self):
        collector, backend = self.make_collector()
        collector.phase = "WAITING"
        collector.camera = Mock(metadata={})
        collector.camera.read.return_value = (b"", False, 0, 0.)
        collector._begin()
        self.assertIsNone(collector.recording_started_at)
        backend.scene_layer_path = __file__
        backend.robot_model = "test"
        backend.dof_names = []
        backend.robot_asset = SimpleNamespace(body_joint_names=[])
        backend.physics_dt = .005
        backend._sim_time = 0.
        collector.status = self.reply()
        collector.camera.read.return_value = (b"image", True, 1, 0.)
        with patch("gear_sonic.simulation_server.kitchen_episodes.Hdf5Episode"), \
                patch("gear_sonic.simulation_server.kitchen_episodes.time.monotonic") as clock:
            for start in (100., 500.):
                clock.return_value = start
                collector._begin()
                self.assertEqual(collector.recording_started_at, start)
                self.assertEqual(collector.phase, "RECORDING")

    def test_save_failure_does_not_reset_or_release(self):
        collector, backend = self.make_collector()
        collector.writer.finish.side_effect = OSError("disk full")
        collector.client.poll.return_value = self.reply()
        collector.before_step()
        backend.mujoco_backspace_reset.assert_not_called()
        self.assertEqual(collector.phase, "ERROR")
        collector.client.request.assert_not_called()

    def test_restart_does_not_authorize_reset(self):
        collector, backend = self.make_collector()
        collector.client.poll.return_value = self.reply(session="new-manager")
        collector.before_step()
        self.assertEqual(collector.phase, "ERROR")
        backend.mujoco_backspace_reset.assert_not_called()

    def test_lost_release_reply_and_open_failure_stays_frozen(self):
        collector, backend = self.make_collector()
        collector.phase = "RELEASING"
        collector.finished_epoch = 1
        collector._begin = Mock(side_effect=OSError("disk full"))
        collector.client.poll.return_value = self.reply(
            operation="release", mode="POSE", hold_token=None, pose_epoch=2,
            released_token="episode-1", wait_for_ax=False,
        )
        collector.before_step()
        self.assertEqual(collector.phase, "ERROR")
        self.assertTrue(collector.frozen)
        backend.mujoco_backspace_reset.assert_not_called()


if __name__ == "__main__":
    unittest.main()
