import unittest
from pathlib import Path

from agent.client import build_initial_input
from agent.cli import decode_command_line
from agent.direct import build_parser
from agent.config import load_api_config
from agent.tools import RobotToolExecutor, tool_definitions


class AgentToolsTest(unittest.TestCase):
    def test_tool_catalog_contains_robot_capabilities(self):
        names = {item["name"] for item in tool_definitions()}
        self.assertEqual(len(names), 12)
        self.assertTrue({
            "get_robot_state", "get_camera_snapshot", "detect_cubes",
            "detect_tags", "move_chassis", "rotate_chassis",
            "execute_arm_action", "grab_cube", "approach_wall",
            "run_strategy", "emergency_stop",
        } <= names)

    def test_dry_run_does_not_require_hardware(self):
        executor = RobotToolExecutor(dry_run=True)
        self.assertTrue(executor.get_robot_state().value["dry_run"])
        self.assertEqual(executor.detect_cubes().value["blocks"], [])
        self.assertEqual(executor.detect_tags().value["tags"], [])
        self.assertEqual(
            executor.execute_arm_action("grap1").value["would_execute"],
            "grap1",
        )

    def test_motion_limits_are_enforced_by_executor(self):
        executor = RobotToolExecutor(dry_run=True)
        with self.assertRaises(ValueError):
            executor.move_chassis("forward", 2001, 400)
        with self.assertRaises(ValueError):
            executor.move_chassis("forward", 100, 751)
        with self.assertRaises(ValueError):
            executor.rotate_chassis(181, 60)

    def test_initial_input_can_include_fixed_image_context(self):
        image = Path(__file__).parent / "agent_test_image.png"
        image.write_bytes(b"test-image")
        try:
            message = build_initial_input("查看场地", [image])[0]
            self.assertEqual(message["role"], "user")
            self.assertEqual(message["content"][1]["type"], "input_image")
            self.assertTrue(message["content"][1]["image_url"].startswith("data:image/png;base64,"))
        finally:
            image.unlink(missing_ok=True)

    def test_default_context_contains_the_fixed_reference_images(self):
        message = build_initial_input("查看场地") [0]
        image_items = [item for item in message["content"]
                       if item["type"] == "input_image"]
        self.assertEqual(len(image_items), 5)

    def test_api_md_style_config_is_parsed_without_printing_secrets(self):
        config = Path(__file__).parent / "agent_test_API.md"
        config.write_text("## APIFUN gpt\nmodel: test-model\nbase url:https://example.test\nkey: secret-value\n", encoding="utf-8")
        try:
            values = load_api_config(config)
            self.assertEqual(values["model"], "test-model")
            self.assertEqual(values["base_url"], "https://example.test")
            self.assertEqual(values["api_key"], "secret-value")
        finally:
            config.unlink(missing_ok=True)

    def test_named_rightcode_profile_uses_codex_responses_path(self):
        config = Path(__file__).parent / "agent_test_profiles_API.md"
        config.write_text(
            "## APIFUN gpt\nmodel: a\nbase url:https://api.example\nkey: a-key\n\n"
            "## rightcode gpt\nmodel: b\nbase url:https://www.rightapi.ai\nkey: b-key\n",
            encoding="utf-8")
        try:
            values = load_api_config(config, profile="rightcode")
            self.assertEqual(values["model"], "b")
            self.assertEqual(values["base_url"], "https://www.rightapi.ai/codex/v1")
            self.assertEqual(values["profile"], "rightcode gpt")
        finally:
            config.unlink(missing_ok=True)

    def test_relay_module_is_importable(self):
        from agent.relay import RelayHandler, build_parser
        self.assertIsNotNone(RelayHandler)
        args = build_parser().parse_args([])
        self.assertEqual(args.port, 8765)
        self.assertEqual(args.profile, "APIFUN gpt")

    def test_command_line_decodes_utf8_and_gb18030(self):
        text = "抓取一个方块"
        self.assertEqual(decode_command_line(text.encode("utf-8")), text)
        self.assertEqual(decode_command_line(text.encode("gb18030")), text)

    def test_direct_scaffold_parses_one_shot_commands(self):
        args = build_parser().parse_args(["--move", "forward", "100", "400"])
        self.assertEqual(args.move, ["forward", "100", "400"])
        args = build_parser().parse_args(["--action", "grap1"])
        self.assertEqual(args.action, "grap1")
        args = build_parser().parse_args(["--vision", "--grab-right-orange"])
        self.assertTrue(args.grab_right_orange)


if __name__ == "__main__":
    unittest.main()
