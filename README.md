# FIGAROH
(Free dynamics Identification and Geometrical cAlibration of RObot and Human)

FIGAROH is a Python toolbox providing efficient and highly flexible frameworks for dynamics identification and geometric calibration of rigid multi-body systems based on the popular modeling convention URDF. The considered systems can be serial (industrial manipulator) or tree-structures (human, humanoid robots).

**📦 Available on PyPI:** `pip install figaroh`

**🚀 Key Features:**
- **Unified Configuration System**: Flexible YAML-based configuration with template inheritance
- **Advanced Parameter Mapping**: Seamless conversion between configuration formats
- **Enhanced Regressor Builder**: Modern, object-oriented regressor computation
- **Comprehensive Examples**: Full working examples in separate repository
- **Cross-Platform Support**: Works on Linux, macOS, and Windows

Note: This repo is a fork from [gitlab repo](https://gitlab.laas.fr/gepetto/figaroh) of which the author is no longer a contributor.

## Installation

### Package Installation

Install the core FIGAROH package:

```bash
# Install from PyPI
pip install figaroh

```bash
# Or install from source
git clone https://github.com/thanhndv212/figaroh-plus.git
cd figaroh
pip install -e .
```
```

### Examples and Tutorials

Examples are maintained in a separate repository to keep the core package lightweight:

```bash
# Clone examples repository
git clone https://github.com/thanhndv212/figaroh-examples.git
cd figaroh-examples

# Install additional dependencies for examples
pip install -r requirements.txt
```

### Development Environment

For development with all dependencies:

```bash
conda env create -f environment.yml
conda activate figaroh-dev
pip install -e .
```
## Prerequisites

FIGAROH has the following core dependencies (automatically installed with pip):
* numpy
* scipy
* matplotlib
* numdifftools
* ndcurves
* meshcat
* rospkg
* pandas

**Note:** Some dependencies should be installed via conda for better compatibility:
```bash
conda install -c conda-forge pinocchio cyipopt
```

## Features

![figaroh_features](figaroh_flowchart.png)

FIGAROH provides a comprehensive suite of tools for robot calibration and identification:

### Dynamic Identification
- **Advanced Model Support**: Dynamic models including friction, actuator inertia, joint torque offsets
- **Optimal Trajectory Generation**: Continuous exciting trajectories with constraint handling
- **Smart Data Processing**: Automated filtering and pre-processing pipelines
- **Multiple Algorithms**: Selection of parameter estimation methods (LS, WLS, TLS)
- **Physical Consistency**: Physically consistent inertial parameters for URDF updates

### Geometric Calibration  
- **Complete Kinematic Models**: Full-set kinematic parameter estimation
- **Optimal Configuration**: Automated calibration posture selection via combinatorial optimization
- **Flexible Sensing**: Support for various sensors (cameras, motion capture, planar constraints)
- **Custom Kinematic Chains**: Adaptable to different robot structures
- **URDF Integration**: Direct model parameter updates

### Configuration Management
- **Unified YAML Format**: Single configuration system for all workflows
- **Template Inheritance**: Reusable configuration templates
- **Automatic Format Detection**: Seamless legacy compatibility
- **Parameter Mapping**: Advanced conversion between configuration formats
- **Validation System**: Comprehensive configuration validation

### Enhanced Tools
- **Modern Regressor Builder**: Object-oriented, extensible regressor computation
- **Advanced Visualization**: Rich plotting and analysis tools  
- **Error Handling**: Robust error management and reporting
- **Results Management**: Structured result storage and analysis
## How to use

**Note:** For complete working examples, see the [figaroh-examples](https://github.com/thanhndv212/figaroh-examples) repository.

Overall, a calibration/identification project folder would look like this:
```
\considered-system
    \config
        considered-system.yaml
    \data
        data.csv
    optimal_config.py
    optimal_trajectory.py
    calibration.py
    identification.py
    update_model.py
```

### Quick Start

1. **Install FIGAROH**:
   ```bash
   pip install figaroh
   ```

2. **Get examples**:
   ```bash
   git clone https://github.com/thanhndv212/figaroh-examples.git
   cd figaroh-examples
   pip install -r requirements.txt
   ```

3. **Run an example**:
   ```bash
   cd examples/tiago
   python identification.py
   ```

### Configuration

FIGAROH now supports a modern unified configuration system with enhanced flexibility and compatibility:

#### Unified Configuration Format

Modern YAML configuration with template inheritance and validation:

```yaml
# modern_config.yaml
inherit_from: "templates/base_robot.yaml"

robot:
  name: "tiago"
  urdf_path: "urdf/tiago.urdf"

calibration:
  method: "full_params"
  sensor_type: "camera"
  optimization:
    algorithm: "least_squares"
    tolerance: 1e-6
  
  markers:
    - ref_joint: "wrist_3_joint"  
      position: [0.1, 0.0, 0.05]
      measure: [true, true, true, true, true, true]

identification:
  mechanics:
    friction_coefficients:
      viscous: [0.01, 0.02, 0.015]
      static: [0.001, 0.002, 0.0015]
    actuator_inertias: [0.1, 0.15, 0.12]
    
  signal_processing:
    sampling_frequency: 5000.0
    cutoff_frequency: 100.0
    filter_type: "butterworth"
    filter_order: 5
```

#### Legacy Format Support

The system maintains full backward compatibility with existing configurations:

```yaml
# legacy_config.yaml  
calibration:
  calib_level: full_params
  base_frame: universe
  tool_frame: wrist_3_link
  markers:
    - ref_joint: wrist_3_joint
      measure: [True, True, True, True, True, True]

identification:
  robot_params:
    - q_lim_def: 1.57
      fv: [0.01, 0.02, 0.015] 
      fs: [0.001, 0.002, 0.0015]
  processing_params:
    - ts: 0.0002
      cut_off_frequency_butterworth: 100.0
```

#### Configuration Features

- **Automatic Format Detection**: System detects and handles both formats seamlessly
- **Template Inheritance**: Reuse common configurations across projects  
- **Parameter Mapping**: Advanced conversion between unified and legacy formats
- **Validation**: Comprehensive validation with helpful error messages
- **Documentation**: Built-in configuration documentation and examples
+ Step 2: Generate sampled exciting postures and trajectories for experimentation.
    - For geometric calibration: Firstly, considering the infinite possibilities of combination of postures can be generated, a finite pool of feasible sampled postures in working space for the considered system needs to be provided thanks to simulator. Then, the pool can be input for a script `optimal_config.py` with a combinatorial optimization algorithm which will calculate and propose an optimal set of calibration postures chosen from the pool with much less number of postures while maximizing the excitation.
    - For dynamic identification: A nonlinear optimization problem needs to formulated and solved thanks to Ipopt solver in a script named `optimal_trajectory.py`. Cost function can be chosen amongst different criteria such as condition number. Joint constraints, self-collision constraints should be obligatory, and other dedicated constraints can be included in constraint functions. Then, the Ipopt solver will iterate and find the best cubic spline that satisfies all constraints and optimize the defined cost function which aims to maximize the excitation for dynamics of the considered system.
+ Step 3: Collect and prepare data in the correct format.  
    To standardize the handling of data, we propose a sample format for collected data in CSV format. These datasets should be stored in a `data` folder for such considered system.
+ Step 4: Create a script implementing identification/calibration algorithms with templates.  
    Dedicated template scripts `calibration.py` and `identification.py` are provided. Users need to fill in essential parts to adapt to their systems. At the end, calibration/identification results will be displayed with visualization and statistical analysis. Then, it is up to users to justify the quality of calibration/identification based on their needs.
+ Step 5: Update model with identified parameters.  
    Once the results are accepted, users can update calibrated/identified parameters to their URDF model by scripts `update_model.py` or simply save to a `xacro` file for later usage.
## Examples

Complete examples and tutorials are available in a separate repository: [figaroh-examples](https://github.com/thanhndv212/figaroh-examples)

The examples include:
- **Human modeling**: Joint center estimation, segment inertial identification
- **Industrial manipulator Staubli TX40**: Dynamic inertial parameters identification
- **Industrial manipulator Universal UR10**: Geometric calibration using RealSense camera and checkerboard
- **3D-printed 3DOF manipulator MATE**: Geometric calibration using ArUco markers
- **Mobile manipulator TIAGo**: Dynamic identification, geometric calibration, mobile base modeling
- **Humanoid TALOS**: Torso-arm geometric calibration, whole-body calibration

Each example includes:
- Configuration files
- Sample data
- Complete workflows
- URDF models (when needed)

## Package Structure

The FIGAROH package is organized into the following modules:

### Core Modules
- **`figaroh.calibration`**: Geometric calibration algorithms and base classes
  - `BaseCalibration`: Modern calibration workflow management
  - `calibration_tools`: Parameter parsing and mapping functions
  
- **`figaroh.identification`**: Dynamic parameter identification methods
  - `BaseIdentification`: Modern identification workflow management  
  - `identification_tools`: Parameter processing and regressor utilities

- **`figaroh.tools`**: Core utilities for robotics computations
  - `regressor`: Enhanced regressor builder with object-oriented design
  - `robot`: Robot model loading and management utilities
  - `robotvisualization`: Advanced visualization tools

### Utility Modules  
- **`figaroh.utils`**: Helper functions and system utilities
  - `config_parser`: Unified configuration parsing system
  - `results_manager`: Structured result storage and analysis
  - `error_handling`: Robust error management framework
  - `cubic_spline`: Spline interpolation utilities

- **`figaroh.measurements`**: Data handling and processing
- **`figaroh.visualisation`**: Plotting and visualization tools  
- **`figaroh.optimal`**: Optimization algorithms for trajectory and configuration generation

### Key Improvements in Latest Version

#### Enhanced Configuration System
- **Unified YAML Format**: Single configuration system supporting both new and legacy formats
- **Template Inheritance**: Reusable configuration templates with parameter overrides
- **Automatic Format Detection**: Seamless backward compatibility 
- **Advanced Parameter Mapping**: Conversion between unified and legacy configuration formats

#### Modern Object-Oriented Design
- **RegressorBuilder**: Flexible, extensible regressor computation
- **Base Classes**: Standardized calibration and identification workflows
- **Configuration Management**: Integrated config parsing and validation

#### Improved Robustness  
- **Error Handling**: Comprehensive error management with informative messages
- **Input Validation**: Robust parameter validation and type checking
- **Cross-Platform Support**: Enhanced compatibility across operating systems

## API Usage

### Modern Object-Oriented Interface

```python
import figaroh
from figaroh.calibration import BaseCalibration
from figaroh.identification import BaseIdentification  
from figaroh.tools.regressor import RegressorBuilder, RegressorConfig
from figaroh.utils.config_parser import UnifiedConfigParser

# Load robot model
robot = figaroh.tools.robot.load_robot("path/to/robot.urdf")

# Modern configuration parsing
parser = UnifiedConfigParser("config/robot_config.yaml")
config = parser.parse()

# Calibration workflow
calibration = BaseCalibration(robot, "config/robot_config.yaml")
calibration.load_data("data/calibration_data.csv")
results = calibration.run_calibration()

# Identification workflow  
identification = BaseIdentification(robot, "config/robot_config.yaml")
identification.load_data("data/identification_data.csv")
params = identification.run_identification()

# Advanced regressor building
regressor_config = RegressorConfig(
    has_friction=True,
    has_actuator_inertia=True,
    is_joint_torques=True
)
builder = RegressorBuilder(robot, regressor_config)
W = builder.build_basic_regressor(q, dq, ddq)
```

### Legacy Function Interface

```python
# Legacy function-based interface (still supported)
from figaroh.calibration import calibration_tools
from figaroh.identification import identification_tools

# Load parameters from YAML
calib_config = calibration_tools.get_param_from_yaml(robot, config_data)
identif_config = identification_tools.get_param_from_yaml(robot, config_data)

# Build regressors
from figaroh.tools.regressor import build_regressor_basic
W = build_regressor_basic(robot, q, dq, ddq, identif_config)
```

### Configuration Format Conversion

```python
from figaroh.calibration.calibration_tools import unified_to_legacy_config
from figaroh.identification.identification_tools import unified_to_legacy_identif_config

# Convert unified format to legacy format
unified_config = parser.create_task_config(robot, parsed_config, "calibration")  
legacy_config = unified_to_legacy_config(robot, unified_config)

# Automatic format detection and conversion in base classes
calibration = BaseCalibration(robot, "any_config_format.yaml")  # Works with both formats
```
## Citations

If you use FIGAROH in your research, please cite the following papers:

### Main Reference
```bibtex
@inproceedings{nguyen2023figaroh,
  title={FIGAROH: a Python toolbox for dynamic identification and geometric calibration of robots and humans},
  author={Nguyen, Dinh Vinh Thanh and Bonnet, Vincent and Maxime, Sabbah and Gautier, Maxime and Fernbach, Pierre and others},
  booktitle={IEEE-RAS International Conference on Humanoid Robots},
  pages={1--8},
  year={2023},
  address={Austin, TX, United States},
  doi={10.1109/Humanoids57100.2023.10375232},
  url={https://hal.science/hal-04234676v2}
}
```

### Related Work
```bibtex
@inproceedings{nguyen2024improving,
  title={Improving Operational Accuracy of a Mobile Manipulator by Modeling Geometric and Non-Geometric Parameters},
  author={Nguyen, Thanh D. V. and Bonnet, V. and Fernbach, P. and Flayols, T. and Lamiraux, F.},
  booktitle={2024 IEEE-RAS 23rd International Conference on Humanoid Robots (Humanoids)},
  pages={965--972},
  year={2024},
  address={Nancy, France},
  doi={10.1109/Humanoids58906.2024.10769790}
}

@techreport{nguyen2025humanoid,
  title={Humanoid Robot Whole-body Geometric Calibration with Embedded Sensors and a Single Plane},
  author={Nguyen, Thanh D V and Bonnet, Vincent and Fernbach, Pierre and Daney, David and Lamiraux, Florent},
  year={2025},
  institution={HAL},
  url={https://hal.science/hal-05169055}
}
```

## License

Please refer to the [LICENSE](LICENSE) file for licensing information.
