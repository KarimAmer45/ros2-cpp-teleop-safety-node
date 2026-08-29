<!-- Copyright 2026 Karim Amer -->

# Contributing

Any contribution that you make to this repository will
be under the MIT license, as dictated by that
[license](https://opensource.org/licenses/MIT).

Keep changes focused on the safety boundary: raw commands enter through
`/cmd_vel_raw`, and only filtered commands leave through `/cmd_vel`.

Before opening a pull request, source the target ROS 2 distribution and run:

```bash
colcon build --packages-select ros2_cpp_teleop_safety_node
colcon test --packages-select ros2_cpp_teleop_safety_node
colcon test-result --verbose
```

Pull requests should describe the affected guard layer and include reproduction
steps for behavior or configuration changes.
