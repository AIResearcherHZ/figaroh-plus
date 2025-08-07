# Copyright [2021-2025] Thanh Nguyen
# Copyright [2022-2023] [CNRS, Toward SAS]

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

# http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from sys import argv
import numpy as np
import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.visualize import GepettoVisualizer, MeshcatVisualizer


class Robot(RobotWrapper):
    """Robot class extending Pinocchio's RobotWrapper with additional features."""

    def __init__(
        self,
        robot_urdf,
        package_dirs,
        isFext=False,
        freeflyer_ori=None,
    ):
        """Initialize robot model from URDF.
        
        Args:
            robot_urdf: Path to URDF file
            package_dirs: Package directories for mesh files
            isFext: Whether to add floating base joint
            freeflyer_ori: Optional orientation for floating base
        """
        # Intrinsic dynamic parameter names 
        self.params_name = (
            "Ixx", "Ixy", "Ixz", "Iyy", "Iyz", "Izz",
            "mx", "my", "mz", "m"
        )

        self.isFext = isFext
        self.robot_urdf = robot_urdf

        # Initialize robot model
        if not isFext:
            self.initFromURDF(robot_urdf, package_dirs=package_dirs)
        else:
            self.initFromURDF(
                robot_urdf,
                package_dirs=package_dirs, 
                root_joint=pin.JointModelFreeFlyer()
            )

        # Set floating base parameters if provided
        if freeflyer_ori is not None and isFext:
            joint_id = self.model.getJointId("root_joint")
            self.model.jointPlacements[joint_id].rotation = freeflyer_ori
            
            # Update position limits
            ub = self.model.upperPositionLimit
            lb = self.model.lowerPositionLimit
            ub[:7] = 1
            lb[:7] = -1
            self.model.upperPositionLimit = ub
            self.model.lowerPositionLimit = lb
            self.data = self.model.createData()

        self.geom_model = self.collision_model


def load_robot(robot_urdf, package_dirs=None, isFext=False, load_by_urdf=True, robot_pkg=None):
    """
    Load the robot model from the URDF file.
Args:
        robot_urdf (str): Path to the URDF file.
        package_dirs (str): Package directories for mesh files.
        isFext (bool): Whether to add floating base joint.
        load_by_urdf (bool): Whether to load the robot by URDF.
        robot_pkg (str): Name of the robot package.
    Returns:
        Robot: An instance of the Robot class.
    
    Raises:
        ImportError: If the required packages are not installed.

    Note: For loading by URDF, robot_urdf and package_dirs can be different.
          1/ If package_dirs is not provided directly, robot_pkg is used to
          look up the package directory.
            - If the robot description ROS package is
          installed, the path to this package will be used
          to load the robot model.
            - If not, it will be loaded from /models directory.
          2/ There is no mesh files, package_dirs and robot_pkg are not used.
          3/ If load_by_urdf is False, the robot is loaded from the ROS
          parameter server.
    """
    import os
    import rospkg

    if load_by_urdf:
        if package_dirs is None:
            if robot_pkg is not None:
                try:
                    package_dirs = rospkg.RosPack().get_path(robot_pkg)
                except rospkg.ResourceNotFound:
                    # Resolve relative path to models directory
                    current_dir = os.path.dirname(os.path.abspath(__file__))
                    project_root = os.path.dirname(os.path.dirname(os.path.dirname(current_dir)))
                    package_dirs = os.path.join(project_root, "models")
            else:
                package_dirs = os.path.dirname(os.path.abspath(robot_urdf))
        elif package_dirs == "models":
            current_dir = os.path.dirname(os.path.abspath(__file__))
            project_root = os.path.dirname(os.path.dirname(os.path.dirname(current_dir)))
            package_dirs = os.path.join(project_root, "models")

        if not os.path.exists(robot_urdf):
            raise FileNotFoundError(f"URDF file not found: {robot_urdf}")

        robot = Robot(robot_urdf, package_dirs=package_dirs, isFext=isFext)
    else:
        import rospy
        from pinocchio.robot_wrapper import RobotWrapper

        robot_xml = rospy.get_param("robot_description")
        if isFext:
            robot = RobotWrapper(
                pinocchio.buildModelFromXML(
                    robot_xml, root_joint=pinocchio.JointModelFreeFlyer()
                )
            )
        else:
            robot = RobotWrapper(pinocchio.buildModelFromXML(robot_xml))
    return robot