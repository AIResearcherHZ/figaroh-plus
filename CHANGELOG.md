# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.2.4] - 2025-09-08

### Changed
- **Optional Dependencies**: Removed `cyipopt` from required dependencies
  - cyipopt is now truly optional and loaded only when IPOPT optimization is used
  - Users can install without cyipopt and still use all other features
  - Install cyipopt separately when needed: `pip install cyipopt` or via conda environment

### Improved
- **Installation Flexibility**: Package now installs without requiring heavy optimization dependencies
- **Error Handling**: Better error messages when optional dependencies are missing

## [0.2.3] - 2025-09-08

### Added
- **Streamlined Dependencies**: All core dependencies now available via PyPI with automatic installation
- **Lazy Loading**: Optional dependencies (cyipopt) now loaded only when needed to improve startup time
- **Enhanced Installation Notes**: Clear documentation of simplified dependency management

### Improved
- **Dependency Management**: Complete cleanup and optimization of package dependencies
  - Removed redundant `requirements.txt` and `setup.py` files
  - Consolidated all dependencies in `pyproject.toml`
  - Updated to use PyPI versions of robotics libraries (`pin` for Pinocchio)
- **Installation Process**: Significantly simplified installation with better cross-platform compatibility
- **Documentation**: Comprehensive README updates reflecting modern packaging standards
  - Combined development installation methods for clarity
  - Added official Pinocchio repository reference
  - Updated dependency documentation with descriptions
- **Performance**: Faster module loading through localized imports
- **Environment Setup**: Streamlined conda environment with minimal dependencies

### Enhanced
- **Import Strategy**: Localized cyipopt import to specific functions for better error handling
- **Error Messages**: More informative import error messages with installation instructions
- **Package Structure**: Modern Python packaging standards with pyproject.toml-only approach

### Removed
- **Redundant Files**: Eliminated `requirements.txt` and `setup.py` in favor of modern `pyproject.toml`
- **Unnecessary Dependencies**: Cleaned up unused dependencies for leaner installation

### Fixed
- **Package Name**: Corrected dependency references (e.g., proper use of `pin` for Pinocchio PyPI version)
- **Installation Conflicts**: Resolved potential conflicts between conda and pip installations

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
