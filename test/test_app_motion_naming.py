#!/usr/bin/env python3
"""Static contract tests for APP-layer motion component naming.

These checks prevent a partial rename where CMake, launch files, ROS node names,
or lifecycle service defaults still point to the retired identifiers.
"""

from pathlib import Path
import re
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


class AppMotionNamingContractTest(unittest.TestCase):
    def test_deployment_manifests_do_not_register_test_artifacts(self) -> None:
        cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        package_xml = (PACKAGE_ROOT / "package.xml").read_text(encoding="utf-8")

        for forbidden in (
            "BUILD_TESTING",
            "enable_testing",
            "add_test",
            "test/",
            "sensor_mock_node",
            "Python3",
        ):
            with self.subTest(cmake_forbidden=forbidden):
                self.assertNotIn(forbidden, cmake)
        self.assertNotIn("<test_depend>", package_xml)

    def test_app_layer_files_exist_and_retired_paths_are_removed(self) -> None:
        expected_paths = (
            "src/app_keyboard_control_node.cpp",
            "src/app_motion_target_node.cpp",
            "src/app_motion_mode_manager_node.cpp",
            "src/app_motion_mode_coordinator.cpp",
            "include/app/app_motion_mode_protocol.hpp",
            "include/app/app_motion_mode_coordinator.hpp",
        )
        retired_paths = (
            "src/hal_keyboard_control_node.cpp",
            "src/hal_motion_target_node.cpp",
            "src/motion_mode_manager_node.cpp",
            "src/motion_mode_coordinator.cpp",
            "include/hal/motion_mode_protocol.hpp",
            "include/hal/motion_mode_coordinator.hpp",
            "test/test_motion_mode_core.cpp",
            "launch/keyboard_control.launch.py",
            "launch/motion_target.launch.py",
            "launch/remote_motion_control.launch.py",
            "launch/motion_control.launch.py",
        )

        for relative_path in expected_paths:
            with self.subTest(expected=relative_path):
                self.assertTrue((PACKAGE_ROOT / relative_path).is_file())
        for relative_path in retired_paths:
            with self.subTest(retired=relative_path):
                self.assertFalse((PACKAGE_ROOT / relative_path).exists())

    def test_build_and_runtime_identifiers_use_app_prefix(self) -> None:
        cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        required_cmake_fragments = (
            "add_library(app_motion_mode_core STATIC src/app_motion_mode_coordinator.cpp)",
            "add_executable(app_motion_mode_manager_node src/app_motion_mode_manager_node.cpp)",
            "add_executable(app_keyboard_control_node src/app_keyboard_control_node.cpp)",
            "add_executable(app_motion_target_node src/app_motion_target_node.cpp)",
        )
        for fragment in required_cmake_fragments:
            with self.subTest(fragment=fragment):
                self.assertIn(fragment, cmake)

        source_expectations = {
            "src/app_keyboard_control_node.cpp": (
                'LifecycleNode("app_keyboard_control_node")',
                "class AppKeyboardControlNode",
            ),
            "src/app_motion_target_node.cpp": (
                'LifecycleNode("app_motion_target_node")',
                "class AppMotionTargetNode",
            ),
            "src/app_motion_mode_manager_node.cpp": (
                'rclcpp::Node("app_motion_mode_manager_node")',
                "class AppMotionModeManagerNode",
                '"keyboard_node", "/app_keyboard_control_node"',
                '"target_node", "/app_motion_target_node"',
                '"controller_node", "/bsp_motioncontrol_node"',
            ),
        }
        for relative_path, fragments in source_expectations.items():
            content = (PACKAGE_ROOT / relative_path).read_text(encoding="utf-8")
            for fragment in fragments:
                with self.subTest(file=relative_path, fragment=fragment):
                    self.assertIn(fragment, content)

    def test_each_launch_file_pairs_new_executable_and_node_names(self) -> None:
        launch_dir = PACKAGE_ROOT / "launch"
        launch_expectations = {
            "app_remote_motion_control.launch.py": (
                "app_keyboard_control_node",
                "app_motion_target_node",
                "app_motion_mode_manager_node",
                "bsp_motioncontrol_node",
            ),
            "app_keyboard_control.launch.py": ("app_keyboard_control_node",),
            "app_motion_target.launch.py": ("app_motion_target_node",),
        }
        for launch_name, identifiers in launch_expectations.items():
            launch_text = (launch_dir / launch_name).read_text(encoding="utf-8")
            for identifier in identifiers:
                escaped = re.escape(identifier)
                with self.subTest(file=launch_name, executable=identifier):
                    self.assertRegex(
                        launch_text,
                        rf"executable\s*=\s*['\"]{escaped}['\"]",
                    )
                with self.subTest(file=launch_name, node_name=identifier):
                    self.assertRegex(
                        launch_text,
                        rf"name\s*=\s*['\"]{escaped}['\"]",
                    )

    def test_retired_runtime_identifiers_are_absent_from_build_inputs(self) -> None:
        inspected_paths = [PACKAGE_ROOT / "CMakeLists.txt"]
        inspected_paths.extend(sorted((PACKAGE_ROOT / "launch").glob("*.launch.py")))
        inspected_paths.extend(sorted((PACKAGE_ROOT / "src").glob("app_*.cpp")))
        inspected_paths.extend(sorted((PACKAGE_ROOT / "include/app").glob("*.hpp")))
        build_input_text = "\n".join(
            path.read_text(encoding="utf-8") for path in inspected_paths
        )

        retired_runtime_patterns = (
            r"(?<!app_)hal_keyboard_control_node",
            r"(?<!app_)hal_motion_target_node",
            r"(?<!app_)motion_mode_manager_node",
        )
        for pattern in retired_runtime_patterns:
            with self.subTest(retired_pattern=pattern):
                self.assertIsNone(re.search(pattern, build_input_text))

    def test_protocol_and_data_plane_interfaces_are_unchanged(self) -> None:
        protocol = (
            PACKAGE_ROOT / "include/app/app_motion_mode_protocol.hpp"
        ).read_text(encoding="utf-8")
        keyboard = (
            PACKAGE_ROOT / "src/app_keyboard_control_node.cpp"
        ).read_text(encoding="utf-8")
        target = (
            PACKAGE_ROOT / "src/app_motion_target_node.cpp"
        ).read_text(encoding="utf-8")

        self.assertIn("PROTOCOL_VERSION = 1U", protocol)
        self.assertIn("REQUEST_SIZE = 20U", protocol)
        self.assertIn("RESPONSE_SIZE = 24U", protocol)
        self.assertIn("DATA_PLANE_SIZE = 17U", protocol)
        self.assertIn('"/hal/thruster/cmd"', keyboard)
        self.assertIn('"/app/motioncontrol"', target)


if __name__ == "__main__":
    unittest.main()
