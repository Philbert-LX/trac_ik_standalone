# Changelog

All notable changes to this standalone version of TRAC-IK will be documented in this file.

## [Standalone Version] - 2024-XX-XX

### Added
- Support for non-ROS builds via `USE_ROS` macro
- `rclcpp_logger_stub.hpp` as ROS Logger replacement for non-ROS environments
- Standalone CMakeLists.txt for non-ROS compilation
- Build scripts for Windows (`build_standalone.bat`) and Linux/Mac (`build_standalone.sh`)
- Comprehensive documentation:
  - `README.md` - Main documentation
  - `API_USAGE_EXAMPLES.md` - API usage guide with examples
  - `CHANGELOG.md` - Version history

### Modified
- `trac_ik_lib/include/trac_ik/trac_ik.hpp`
  - Added conditional compilation support (`USE_ROS` macro)
  - Replaced `rclcpp/rclcpp.hpp` with conditional include
  - Wrapped ROS-specific constructors with `#if USE_ROS`
  
- `trac_ik_lib/include/trac_ik/nlopt_ik.hpp`
  - Added conditional compilation support (`USE_ROS` macro)
  - Replaced `rclcpp/rclcpp.hpp` with conditional include
  - Wrapped ROS-specific constructors with `#if USE_ROS`

- `trac_ik_lib/src/trac_ik.cpp`
  - Added conditional compilation support (`USE_ROS` macro)
  - Wrapped URDF-related constructor with `#if USE_ROS`
  - Wrapped ROS-specific constructors with `#if USE_ROS`
  - Removed hard dependency on `kdl_parser` and `urdf` for non-ROS builds

- `trac_ik_lib/src/nlopt_ik.cpp`
  - Added conditional compilation support (`USE_ROS` macro)
  - Wrapped ROS-specific constructor with `#if USE_ROS`

### Removed
- Hard dependency on ROS (rclcpp, urdf, kdl_parser) for non-ROS builds
- ROS-specific constructors are now optional (compiled only when `USE_ROS=1`)

### Technical Details

#### Default Behavior
- Default `USE_ROS=0` (non-ROS mode)
- Non-ROS constructors remain available and functional
- ROS constructors are conditionally compiled

#### Compatibility
- **Backward Compatible**: Can still compile with ROS support by setting `USE_ROS=1`
- **API Compatible**: Non-ROS API (`TRAC_IK(const KDL::Chain&, ...)`) unchanged
- **Functionality Preserved**: All core IK solving capabilities remain intact

#### Dependencies
- **Required**: KDL, Eigen3, NLopt
- **Optional**: ROS (rclcpp, urdf, kdl_parser) - only needed if `USE_ROS=1`

---

## Original TRAC-IK

This is a modified version of TRAC-IK. For the original changelog, please refer to:
- [Original TRAC-IK Repository](https://bitbucket.org/traclabs/trac_ik)

### Original Copyright
Copyright (c) 2015, TRACLabs, Inc.

### Original License
BSD 3-Clause License

