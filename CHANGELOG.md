# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.1.0] - 2025-01-25

### Added
- Initial release of FIGAROH package
- Dynamic identification algorithms for rigid multi-body systems
- Geometric calibration algorithms for serial and tree-structure robots
- Support for URDF modeling convention
- Optimal trajectory generation for dynamic identification
- Optimal posture generation for geometric calibration
- Integration with Pinocchio for efficient computations
- Support for various optimization algorithms
- Data filtering and pre-processing utilities
- Model parameter update utilities

### Features
- **Dynamic Identification**:
  - Dynamic model including friction, actuator inertia, and joint torque offset
  - Continuous optimal exciting trajectory generation
  - Multiple parameter estimation algorithms
  - Physically consistent standard inertial parameters calculation

- **Geometric Calibration**:
  - Full kinematic parameter calibration
  - Optimal calibration posture generation via combinatorial optimization
  - Support for external sensors (cameras, motion capture)
  - Non-external methods (planar constraints)

### Dependencies
- Core scientific computing: numpy, scipy, matplotlib, pandas
- Robotics: pinocchio (via conda)
- Optimization: cyipopt (via conda), quadprog
- Visualization: meshcat
- Additional: numdifftools, ndcurves, rospkg

### Documentation
- Comprehensive README with installation and usage instructions
- Examples moved to separate repository (figaroh-examples)
- API documentation structure prepared

### Notes
- Examples and URDF models moved to separate repository for clean package distribution
- Package optimized for PyPI distribution
- Supports Python 3.8+
