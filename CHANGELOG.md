# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased] - 2025-11-17

### Added
- **Advanced Linear Solver (`figaroh.tools.solver`)**: Comprehensive multivariate linear solver for robot parameter identification
  - Multiple solving methods: lstsq, QR, SVD, Ridge, Lasso, Elastic Net, Tikhonov, constrained, robust, weighted
  - Regularization support: L1 (Lasso), L2 (Ridge), Elastic Net, custom Tikhonov
  - Constraint handling: Box constraints (bounds), linear equality/inequality constraints
  - Robust regression with iterative reweighting for outlier resistance
  - Comprehensive solution quality metrics (RMSE, R², condition number, residuals)
  - Optimized for dense, large, thin matrices typical in robot dynamics
  - Full unit test coverage (18 tests) with robot identification scenarios

- **Module Reorganization**: Better code organization and separation of concerns
  - **Calibration module restructuring**:
    - `calibration/config.py`: Configuration parsing and YAML handling (624 lines)
    - `calibration/parameter.py`: Parameter management utilities (240 lines)
    - `calibration/data_loader.py`: Data loading and I/O operations (160 lines)
  - **Identification module restructuring**:
    - `identification/config.py`: Configuration parsing for identification (334 lines)
    - `identification/parameter.py`: Parameter management for identification (388 lines)
  - Maintains 100% backward compatibility through re-exports

- **BaseIdentification Enhancement**:
  - `solve_with_custom_solver()`: New method using advanced linear solver with regularization and constraints
  - Flexible solving with multiple methods and custom constraints
  - Support for physical parameter bounds (e.g., positive masses/inertias)

### Improved
- **Parameter Ordering**: Changed to Pinocchio dynamic parameter ordering for consistency
  - New order: [Ixx, Ixy, Ixz, Iyy, Iyz, Izz, mx, my, mz, m]
  - Previous order: [m, mx, my, mz, Ixx, Ixy, Iyy, Ixz, Iyz, Izz]
  
- **Regressor Module**: Cleaned up build_basic_regressor methods
  - Removed unused `tau` parameter for better API clarity
  - Improved method signatures and documentation

- **Code Quality**: Significant reduction in code duplication
  - `calibration_tools.py`: Reduced from ~1500 to ~630 lines (-58%)
  - `identification_tools.py`: Reduced from ~900 to ~295 lines (-67%)
  - Modular design with clear single responsibilities

### Fixed
- Parameter naming: Changed from numbered indices to parent joint names for clarity
- Regressor handling: Better support for additional columns in regressor matrices
- Results manager imports and formatting issues

### Technical Details
- **Files Changed**: 21 files
- **Lines Added**: +3,372
- **Lines Removed**: -1,604
- **Net Change**: +1,768 lines
- **Test Coverage**: All 18 new solver tests passing

## [0.2.0] - 2025-09-05

### Added
- **Unified Configuration System**: Complete overhaul of configuration management
  - New `UnifiedConfigParser` with YAML template inheritance
  - Automatic format detection for seamless legacy compatibility  
  - Advanced parameter mapping between configuration formats
  - Comprehensive configuration validation with helpful error messages

- **Enhanced Base Classes**: Modern object-oriented workflow management
  - `BaseCalibration`: Standardized calibration workflow with unified config support
  - `BaseIdentification`: Standardized identification workflow with unified config support  
  - Automatic configuration format detection and conversion

- **Advanced Regressor Builder**: Complete redesign of regressor computation
  - `RegressorBuilder`: Object-oriented, extensible regressor construction
  - `RegressorConfig`: Configuration dataclass for regressor parameters
  - Enhanced input validation and error handling
  - Support for joint torque and external wrench modes

- **Configuration Format Mapping**: Seamless format conversion utilities
  - `unified_to_legacy_config`: Calibration parameter mapping function
  - `unified_to_legacy_identif_config`: Identification parameter mapping function
  - Perfect compatibility with existing legacy configurations

### Improved  
- **Parameter Processing**: Enhanced parameter handling with better defaults
- **Error Messages**: More informative validation and error reporting
- **Documentation**: Comprehensive updates to README and module documentation
- **Code Organization**: Better structured modules with clear responsibilities
- **Type Safety**: Added type hints throughout the codebase

### Enhanced
- **Cross-Platform Support**: Improved compatibility across operating systems
- **Input Validation**: Robust parameter validation and type checking  
- **Template System**: Flexible configuration template inheritance
- **Backward Compatibility**: Full support for existing legacy configurations

### Removed
- **quadprog dependency**: Removed unused quadprog dependency to reduce package size

### Fixed
- **Configuration Parsing**: Resolved edge cases in YAML parsing
- **Parameter Mapping**: Accurate conversion between configuration formats
- **Validation Logic**: Improved configuration validation accuracy
- **Error Handling**: Better error recovery and user feedback

### Technical Improvements
- Modern Python practices with dataclasses and type hints
- Enhanced error handling with custom exception classes
- Improved testing framework with comprehensive validation
- Better code documentation and examples

### Documentation
- Updated README with modern API examples
- Enhanced configuration system documentation  
- New API usage patterns and best practices
- Comprehensive module documentation updates

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
- Optimization: cyipopt (via conda)
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
