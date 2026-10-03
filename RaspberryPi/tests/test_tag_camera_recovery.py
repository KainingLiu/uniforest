"""Camera failure injection; no physical cameras, UART or chassis commands."""
import contextlib
import io
import threading
import time
import unittest
from unittest.mock import Mock, patch

import numpy as np

from vision.opencv.field_localizer import FieldLocalizer, FieldPose


class TagCameraRecoveryTests(unittest.TestCase):
    def make_localizer(self):
        loc = FieldLocalizer()
        loc._cap = Mock()
        loc._running = True
        loc._calibrated = False
        loc._detector = Mock()
        loc._detector.detectMarkers.return_value = ([], None, [])
        return loc

    def run_loop(self, loc):
        with contextlib.redirect_stdout(io.StringIO()):
            loc._capture_loop()

    def test_detector_exception_does_not_kill_subsequent_frames(self):
        loc = self.make_localizer()
        frame = np.zeros((10, 10, 3), dtype=np.uint8)
        reads = []

        def read():
            reads.append(1)
            if len(reads) == 3:
                loc._running = False
            return True, frame

        loc._read_frame = read
        loc._detector.detectMarkers.side_effect = [RuntimeError('bad frame'), ([], None, [])]
        self.run_loop(loc)
        self.assertEqual(loc._detector.detectMarkers.call_count, 2)
        self.assertIsNotNone(loc._result)
        self.assertEqual(loc._capture_error, '')
        self.assertFalse(loc.is_running)

    def test_stalled_stream_reopens_and_refreshes_timestamp(self):
        loc = self.make_localizer()
        loc._last_frame_monotonic = time.monotonic() - 2
        loc._result = FieldPose(timestamp=1, valid=True)
        loc.REOPEN_RETRY_S = 0
        old_cap = loc._cap
        new_cap = Mock()
        frame = np.zeros((10, 10, 3), dtype=np.uint8)
        reads = []

        def reopen():
            old_cap.release.assert_called_once()
            loc._cap = new_cap
            return True

        def read():
            reads.append(1)
            if len(reads) == 1:
                return False, None
            if len(reads) == 3:
                loc._running = False
            return True, frame

        loc._open_camera = Mock(side_effect=reopen)
        loc._read_frame = read
        self.run_loop(loc)
        loc._open_camera.assert_called_once()
        new_cap.release.assert_called_once()
        self.assertGreater(loc._result.timestamp, 1)
        self.assertEqual(loc._reopen_count, 1)

    def test_capture_failure_never_refreshes_old_pose(self):
        loc = self.make_localizer()
        loc._thread = Mock(is_alive=lambda: True)
        loc._result = FieldPose(timestamp=1, valid=True, tag_solutions=(Mock(),))
        loc._capture_error = 'read_failed'
        result = loc.result
        self.assertEqual(result.timestamp, 1)
        self.assertFalse(result.valid)
        self.assertEqual(result.tag_solutions, ())
        self.assertEqual(result.capture_error, 'read_failed')

    def test_dead_thread_is_not_running_and_pose_is_invalid(self):
        loc = self.make_localizer()
        loc._thread = Mock(is_alive=lambda: False)
        loc._result = FieldPose(timestamp=time.time(), valid=True)
        self.assertFalse(loc.is_running)
        self.assertFalse(loc.result.valid)
        self.assertEqual(loc.result.capture_error, 'capture_thread_stopped')

    def test_reset_waits_for_inflight_filter_update(self):
        loc = self.make_localizer()
        loc._smoothed = (0, 0, 0)
        entered, proceed, reset_done = threading.Event(), threading.Event(), threading.Event()
        original = loc._smooth_locked

        def update(*args):
            entered.set()
            self.assertTrue(proceed.wait(2))
            return original(*args)

        def reset():
            loc.reset_filter()
            reset_done.set()

        loc._smooth_locked = update
        a = threading.Thread(target=lambda: loc._smooth(1, 1, 1))
        b = threading.Thread(target=reset)
        a.start()
        self.assertTrue(entered.wait(2))
        b.start()
        self.assertFalse(reset_done.wait(.02))
        proceed.set()
        a.join(2)
        b.join(2)
        self.assertTrue(reset_done.is_set())
        self.assertIsNone(loc._smoothed)

    def test_stop_does_not_release_capture_while_read_is_inflight(self):
        loc = self.make_localizer()
        cap = loc._cap
        loc._thread = Mock(is_alive=lambda: True)
        with contextlib.redirect_stdout(io.StringIO()):
            loc.stop()
            self.assertFalse(loc.start())
        cap.release.assert_not_called()
        self.assertTrue(loc._stop_event.is_set())

    def test_poll_timeout_does_not_call_blocking_read_or_retrieve(self):
        loc = self.make_localizer()
        with patch('vision.opencv.field_localizer.cv2.VideoCapture') as factory:
            factory.waitAny.return_value = (False, ())
            self.assertEqual(loc._read_frame(), (False, None))
        loc._cap.read.assert_not_called()
        loc._cap.retrieve.assert_not_called()

    def test_open_re_resolves_role_and_reapplies_exposure(self):
        loc = self.make_localizer()
        cap = Mock()
        cap.get.side_effect = [640, 480]
        with patch('vision.opencv.field_localizer.resolve_camera_source', return_value='/dev/video7') as resolve, \
                patch('vision.opencv.field_localizer.cv2.VideoCapture', return_value=cap) as factory, \
                contextlib.redirect_stdout(io.StringIO()):
            self.assertTrue(loc._open_camera())
        resolve.assert_called_once_with('tag')
        self.assertEqual(factory.call_args.args[0], '/dev/video7')
        import cv2
        cap.set.assert_any_call(cv2.CAP_PROP_EXPOSURE, 150.0)
        cap.set.assert_any_call(cv2.CAP_PROP_GAIN, 32.0)


if __name__ == '__main__':
    unittest.main()
