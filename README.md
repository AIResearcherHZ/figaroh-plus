# FIGAROH
(Free dynamics Identification and Geometrical cAlibration of RObot and Human)
FIGAROH is a python toolbox aiming at providing efficient and highly flexible frameworks for dynamics identification and geometric calibration of rigid multi-body systems based on the popular modeling convention URDF. The considered systems can be serial (industrial manipulator) or tree-structures (human, humanoid robots).

Note: This repo is a fork from [gitlab repo](https://gitlab.laas.fr/gepetto/figaroh) of which the author is no longer a contributor.

## Installation

### Package Installation

Install the core FIGAROH package:

```bash
# Option 1: Install from PyPI (when available)
pip install figaroh

# Option 2: Install from source
git clone https://github.com/thanhndv212/figaroh.git
cd figaroh
pip install -e .
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
The following packages are required:
* numpy
* scipy
* matplotlib
* pinocchio (conda install)
* cyipopt (conda install)
* numdifftools
* meshcat
* rospkg
* pandas
* quadprog

## Features
![figaroh_features](figaroh_flowchart.png)
As described in the following figure it provides:
+ Dynamic Identification:
    - Dynamic model including effects of frictions, actuator inertia and joint torque offset.
    - Generation of continuous optimal exciting trajectories that can be played onto the robot.
    - Guide line on data filtering/pre-processing.
    - Identification pipeline with a selection of dynamic parameter estimation algorithms.
    - Calculation of physically consistent standard inertial parameters that can be updated in a URDF file.
+ Geometric Calibration:
    - Calibration model with full-set kinematic parameters.
    - Generation of optimal calibration postures based on combinatorial optimization.
    - Calibration pipeline with customized kinematic chains and different selection of external sensoring methods (eye-hand camera, motion capture) or non-external methods (planar constraints).
    - Calculatation of kinematic parameters that can be updated in URDF model.
## How to use
Overall, a calibration/identification project folder would like this:
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
A step-by-step procedure is presented as follow.
+ Step 1: Define a config file with sample template.\
    A .yaml file containing information of the considered system and characteristics of the calibration/identification problem has a structure as follow:
    ```
    calibration:
        calib_level: full_params
        non_geom: False
        base_frame: universe
        tool_frame: wrist_3_link
        markers:
            -   ref_joint: wrist_3_joint
                measure: [True, True, True, True, True, True]
        free_flyer: False
        nb_sample: 29
    ```
    ```
    identification:
        robot_params:
            -   q_lim_def: 1.57
                dq_lim_def : 5.0
                ddq_lim_def : 20.0
                tau_lim_def : 4.0
                fv : None
                fs : None
                Ia : None
                offset : None
                Iam6 : None
                fvm6 : None
                fsm6 : None
                N : None
                ratio_essential : None
        problem_params:
            -   is_external_wrench : False
                is_joint_torques : True
                force_torque : ['All']
                external_wrench_offsets : False
                has_friction : False
                has_joint_offset : False
                has_actuator_inertia : False
                is_static_regressor : True
                is_inertia_regressor : True
                has_coupled_wrist : False
                embedded_forces : False
        processing_params:
            -   cut_off_frequency_butterworth: 100.0
                ts : 0.01
        tls_params:
            -   mass_load : None
                which_body_loaded : None
                sync_joint_motion : False
    ```
+ Step 2: Generate sampled exciting postures and trajectories for experimentation.
    - For geomeotric calibration: Firstly, considering the infinite possibilities of combination of postures can be generated, a finite pool of feasible sampled postures in working space for the considered system needs to be provided thanks to simulator. Then, the pool can be input for a script ```optimal_config.py``` with a combinatorial optimization algorithm which will calculate and propose an optimal set of calibration postures chosen from the pool with much less number of postures while maximizing the excitation.
    - For dynamic identification: A nonlinear optimization problem needs to formulated and solved thanks to Ipopt solver in a script namde ```optimal_trajectory.py```. Cost function can be chosen amongst different criteria such as condition number. Joint constraints, self-collision constraints should be obligatory, and other dedicated constraints can be included in constraint functions. Then, the Ipopt solver will iterate and find the best cubic spline that sastifies all constraints and optimize the defined cost function which aims to maximize the excitation for dynamics of the considered system.
+ Step 3: Collect and prepare data in the correct format.\
    To standardize the handling of data, we propose a sample format for collected data in csv format. These datasets should be stored in a ```data``` folder for such considered system.
+ Step 4: Create a script implementing identification/calibration algorithms with templates.
    Dedicated template scripts ```calibration.py``` and ```identification.py``` are provided. Users needs to fill in essential parts to adapt to their systems. At the end, calibration/identification results will be displayed with visualization and statistical analysis. Then, it is up to users to justify the quality of calibration/identification based on their needs.
+ Step 5: Update model with identified parameters.\
    Once the results are accepted, users can update calibrated/identified parameters to their urdf model by scripts ```update_model.py``` or simply save to a ```xacro``` file for later usage.
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
