import json
import os
import tempfile
import unittest

from utils.diagnostics import JsonlDiagnostics, classify_failure


class DiagnosticsTests(unittest.TestCase):
    def test_failure_categories(self):
        self.assertEqual(classify_failure(RuntimeError('telemetry lost')), 
                         'hardware_fault')
        self.assertEqual(classify_failure(RuntimeError('tag 6 lost during delivery alignment')),
                         'perception_fault')
        self.assertEqual(classify_failure(RuntimeError('chassis move failed')),
                         'strategy_fault')

    def test_jsonl_writer_is_opt_in(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, 'run.jsonl')
            JsonlDiagnostics(path).write('motion', direction='right', mm=100)
            with open(path, encoding='utf-8') as stream:
                record = json.loads(stream.readline())
            self.assertEqual(record['event'], 'motion')
            self.assertEqual(record['direction'], 'right')
            self.assertEqual(JsonlDiagnostics().path, None)


if __name__ == '__main__':
    unittest.main()
