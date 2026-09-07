import unittest
from unittest.mock import Mock
from control.actions import Actions
from protocol.commands import ACTION_BUILD


class BuildSequenceTests(unittest.TestCase):
    def test_build_delegates_whole_sequence_to_board(self):
        client = Mock()
        Actions.build(client)
        client._run_action.assert_called_once_with(ACTION_BUILD)
