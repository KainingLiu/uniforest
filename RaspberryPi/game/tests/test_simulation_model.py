"""CAD transform/unit checks and frozen source geometry for the offline viewer."""
import json
from pathlib import Path
import struct
import tempfile
import unittest

import numpy as np
from simulation.robot_model import extract, transform

ROOT = Path(__file__).resolve().parents[1]


class ModelTests(unittest.TestCase):
    def test_gltf_column_major_and_trs_agree(self):
        # A 90-degree z rotation turns +x into +y, then translates by (3,4,5).
        trs = {'rotation': [0, 0, 2**-.5, 2**-.5],
               'translation': [3, 4, 5], 'scale': [2, 1, 1]}
        matrix = transform(trs)
        np.testing.assert_allclose(matrix @ [1, 0, 0, 1], [3, 6, 5, 1], atol=1e-12)
        np.testing.assert_allclose(transform({'matrix': matrix.T.reshape(-1).tolist()}), matrix)

    def test_extractor_excludes_camera_and_applies_parent_units(self):
        document = {'asset': {'version': '2.0'}, 'scenes': [{'nodes': [0, 1]}],
                    'nodes': [{'camera': 0, 'translation': [100, 100, 100]},
                              {'translation': [1, 0, 2], 'children': [2]},
                              {'mesh': 0, 'name': 'RM-wheel', 'scale': [2, 1, 1]}],
                    'meshes': [{'primitives': [{'attributes': {'POSITION': 0}}]}],
                    'accessors': [{'min': [0, 0, 0], 'max': [.1, .2, .3]}]}
        content = json.dumps(document).encode()
        content += b' ' * (-len(content) % 4)
        glb = struct.pack('<IIIII', 0x46546C67, 2, 20+len(content), len(content), 0x4E4F534A)+content
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'fixture.glb'
            path.write_bytes(glb)
            model = extract(path)
        self.assertEqual((model['length_mm'], model['width_mm'], model['height_mm']), (300, 200, 200))
        self.assertAlmostEqual(model['front_extent_mm'], 150)
        self.assertAlmostEqual(model['rear_extent_mm'], 150)
        self.assertEqual(len(model['boxes']), 1)

    def test_recorded_robot_bounds_contain_every_part(self):
        model = json.loads((ROOT/'simulation/robot_model.json').read_text(encoding='utf-8'))
        self.assertEqual(len(model['boxes']), 158)
        for box in model['boxes']:
            self.assertLessEqual(box['x']+box['length']/2, model['front_extent_mm']+.02)
            self.assertGreaterEqual(box['x']-box['length']/2, -model['rear_extent_mm']-.02)
            self.assertLessEqual(abs(box['y'])+box['width']/2, model['collision_width_mm']/2+.02)
            self.assertGreaterEqual(box['z']-box['height']/2, -.02)
            self.assertLessEqual(box['z']+box['height']/2, model['height_mm']+.02)

    def test_field_records_driveable_surfaces_and_source_conflicts(self):
        field = json.loads((ROOT/'simulation/field_model.json').read_text(encoding='utf-8'))
        self.assertEqual((field['width_mm'], field['height_mm']), (4800, 7200))
        by_id = {s['id']: s for s in field['shapes']}
        self.assertTrue(by_id['central_divider']['collision'])
        self.assertFalse(by_id['ramp_blue']['collision'])
        self.assertEqual(by_id['ramp_blue']['height_mm'], 200)
        self.assertIn('low_edge', by_id['ramp_blue'])
        for shape in field['shapes']:
            self.assertTrue(shape['source'])
            for x, y in shape['polygon']:
                self.assertTrue(0 <= x <= 4800 and 0 <= y <= 7200)


if __name__ == '__main__':
    unittest.main()
