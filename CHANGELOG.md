# Changelog

All notable changes to this project will be documented in this file.

## [v1.2.1] - 2025-03-06
### Changed
- PID controller now has an auto-tuning capability.


## [v1.2.0] - 2025-03-03
### Added
- Implemented a PID controller with cross-track error correction with respect to each position of car unlike the last PID model with error being calculated with the angle subtendeed by the next waypoint with reference to the car.

## [v1.1.1] - 2025-02-17
- Added CSV file reading for car path simulation
### Changed
- Updated visualization settings for RViz markers
### Fixed
- Resolved issue with markers disappearing in RViz

## [v1.1.0] - 2025-02-16
### Added
- Initial implementation of a PID controller
- Initial implementation of car path simulation using ROS 2

