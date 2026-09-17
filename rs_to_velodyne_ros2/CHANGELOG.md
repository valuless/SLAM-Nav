# Changelog

## [1.9.9]

### Strategy-based conversion

- Added configurable ring source, count, and permutation mapping.
- Added configurable time source, mode, unit, and ordering policy.
- Required scalar input fields to have `count == 1`.
- Rejected `XYZIRT` output when no time source is configured.

### Safety and performance

- Replaced PCL conversion with one bounded `PointCloud2` traversal.
- Added layout, size, timestamp, ring, and endianness validation.
- Added an exception-safe reusable output message pool.
- Preserved Humble-compatible `publish(*guard)` behavior.

### Build and deployment

- Removed PCL dependencies and required standard C++17.
- Added a datatype-, endian-, and row-stride-aware ring probe.
- Preserved package and executable name `rs_to_velodyne_ros2`.

Earlier 1.0-1.9.8 revisions were internal candidates and were not released.
