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

"""Robot loading utilities with support for multiple backends."""

import os
from typing import Optional, Union, Any
import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper

# Import Robot class from the same package
from .robot import Robot


def load_robot(
    robot_urdf: str, 
    package_dirs: Optional[str] = None, 
    isFext: bool = False, 
    load_by_urdf: bool = True, 
    robot_pkg: Optional[str] = None,
    loader: str = "figaroh",
    **kwargs
) -> Union[Robot, RobotWrapper, Any]:
    """Load robot model from various sources with multiple loader options.
    
    Args:
        robot_urdf: Path to URDF file or robot name for robot_description
        package_dirs: Package directories for mesh files
        isFext: Whether to add floating base joint
        load_by_urdf: Whether to load from URDF file (vs ROS param server)
        robot_pkg: Name of robot package for path resolution
        loader: Loader type - "figaroh", "robot_description", "yourdfpy"
        **kwargs: Additional arguments passed to the specific loader
        
    Returns:
        Robot object based on loader type:
        - "figaroh": Robot class instance (default, backward compatible)
        - "robot_description": RobotWrapper from pinocchio 
        - "yourdfpy": URDF object from yourdfpy (suitable for viser)
        
    Raises:
        FileNotFoundError: If URDF file not found
        ImportError: If required packages not available
        ValueError: If the specified loader is not supported

    Note: For loading by URDF, robot_urdf and package_dirs can be different.
          1/ If package_dirs is not provided directly, robot_pkg is used to
          look up the package directory.
          2/ If no mesh files, package_dirs and robot_pkg are not used.
          3/ If load_by_urdf is False, the robot is loaded from the ROS
          parameter server.
          4/ For robot_description loader, robot_urdf should be the robot name.
          5/ For yourdfpy loader, returns URDF object suitable for viser visualization.
    """
    # Handle different loaders
    if loader == "robot_description":
        return _load_robot_description(robot_urdf, isFext, **kwargs)
    elif loader == "yourdfpy":
        return _load_yourdfpy(robot_urdf, package_dirs, robot_pkg, **kwargs)
    elif loader == "figaroh":
        # Original figaroh implementation (backward compatible)
        return _load_figaroh_original(robot_urdf, package_dirs, isFext, load_by_urdf, robot_pkg)
    else:
        raise ValueError(f"Unsupported loader: {loader}. Supported loaders: figaroh, robot_description, yourdfpy")


def _check_package_available(package_name: str) -> bool:
    """Check if a package is available for import."""
    try:
        __import__(package_name)
        return True
    except ImportError:
        return False


def _load_robot_description(robot_name: str, isFext: bool = False, **kwargs) -> RobotWrapper:
    """Load robot from robot_descriptions package."""
    if not _check_package_available("robot_descriptions"):
        raise ImportError("robot_descriptions package is not available")
    
    try:
        from robot_descriptions.loaders.pinocchio import load_robot_description
        
        # Prepare kwargs for load_robot_description
        loader_kwargs = kwargs.copy()
        
        # Handle free-flyer joint if requested
        if isFext:
            loader_kwargs['root_joint'] = pin.JointModelFreeFlyer()
        
        # Try to load robot description, with fallback to "_description" suffix
        try:
            robot = load_robot_description(robot_name, **loader_kwargs)
        except ModuleNotFoundError:
            try:
                robot = load_robot_description(f"{robot_name}_description", **loader_kwargs)
            except ModuleNotFoundError:
                raise ModuleNotFoundError(
                    f"Robot description '{robot_name}' not found. "
                    f"Try specifying the full name like '{robot_name}_description' "
                    f"or check available robot descriptions."
                )
            
        return robot
        
    except ImportError as e:
        raise ImportError(f"Required packages not available for robot_description loader: {e}")
    except Exception as e:
        raise RuntimeError(f"Failed to load robot description '{robot_name}': {e}")


def _load_yourdfpy(robot_urdf: str, package_dirs: Optional[str], robot_pkg: Optional[str], **kwargs) -> Any:
    """Load robot using yourdfpy (suitable for viser)."""
    if not _check_package_available("yourdfpy"):
        raise ImportError("yourdfpy package is not available")
    
    try:
        import yourdfpy
        
        # Prepare package_dirs
        package_dirs = _prepare_package_dirs(robot_urdf, package_dirs, robot_pkg)
        
        # Load with yourdfpy
        robot = yourdfpy.URDF.load(
            robot_urdf,
            mesh_dir=package_dirs,
            build_collision_scene_graph=kwargs.get('build_collision_scene_graph', True),
            load_meshes=kwargs.get('load_meshes', True),
            build_scene_graph=kwargs.get('build_scene_graph', True),
            load_collision_meshes=kwargs.get('load_collision_meshes', False),
            force_collision_mesh=kwargs.get('force_collision_mesh', False),
            force_mesh=kwargs.get('force_mesh', False),
            **{k: v for k, v in kwargs.items() if k not in [
                'build_collision_scene_graph', 'load_meshes', 'build_scene_graph',
                'load_collision_meshes', 'force_collision_mesh', 'force_mesh'
            ]}
        )
        
        return robot
        
    except ImportError as e:
        raise ImportError(f"yourdfpy package not available: {e}")


def _load_figaroh_original(robot_urdf: str, package_dirs: Optional[str], isFext: bool, 
                          load_by_urdf: bool, robot_pkg: Optional[str]) -> Union[Robot, RobotWrapper]:
    """Original figaroh implementation for backward compatibility."""
    if load_by_urdf:
        package_dirs = _prepare_package_dirs(robot_urdf, package_dirs, robot_pkg)
        _validate_urdf_exists(robot_urdf)
        return Robot(robot_urdf, package_dirs=package_dirs, isFext=isFext)
    else:
        return _load_from_ros_param(isFext)


def _prepare_package_dirs(robot_urdf: str, package_dirs: Optional[str], robot_pkg: Optional[str]) -> str:
    """Prepare package directories for robot loading."""
    if package_dirs is None:
        if robot_pkg is not None:
            try:
                import rospkg
                package_dirs = rospkg.RosPack().get_path(robot_pkg)
            except (ImportError, Exception):
                # Resolve relative path to models directory
                package_dirs = _get_models_directory()
        else:
            package_dirs = os.path.dirname(os.path.abspath(robot_urdf))
    elif package_dirs == "models":
        package_dirs = _get_models_directory()
    
    return package_dirs


def _resolve_package_dirs(robot_urdf: str, package_dirs: Optional[str], robot_pkg: Optional[str]) -> str:
    """Resolve package directories for mesh loading (kept for backward compatibility)."""
    return _prepare_package_dirs(robot_urdf, package_dirs, robot_pkg)


def _get_ros_package_path(robot_pkg: str) -> str:
    """Get ROS package path with fallback to models directory."""
    try:
        import rospkg
        return rospkg.RosPack().get_path(robot_pkg)
    except (ImportError, Exception):
        return _get_models_directory()


def _get_models_directory() -> str:
    """Get models directory relative to current file."""
    current_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(os.path.dirname(os.path.dirname(current_dir)))
    return os.path.join(project_root, "models")


def _validate_urdf_exists(robot_urdf: str) -> None:
    """Validate that URDF file exists."""
    if not os.path.exists(robot_urdf):
        raise FileNotFoundError(f"URDF file not found: {robot_urdf}")


def _load_from_ros_param(isFext: bool) -> RobotWrapper:
    """Load robot from ROS parameter server."""
    if not _check_package_available("rospy"):
        raise ImportError("rospy package is not available for ROS parameter server loading")
    
    try:
        import rospy
        from pinocchio.robot_wrapper import RobotWrapper
    except ImportError as e:
        raise ImportError(f"ROS packages not available: {e}")

    robot_xml = rospy.get_param("robot_description")
    root_joint = pin.JointModelFreeFlyer() if isFext else None
    model = pin.buildModelFromXML(robot_xml, root_joint=root_joint)
    
    return RobotWrapper(model)


def get_available_loaders() -> dict:
    """Get information about available robot loaders.
    
    Returns:
        dict: Information about each loader and its availability
    """
    loaders = {
        "figaroh": {
            "description": "Original figaroh Robot class",
            "available": True,
            "returns": "Robot instance",
            "features": ["URDF loading", "ROS param server", "Free-flyer support"]
        },
        "robot_description": {
            "description": "Load from robot_descriptions package",
            "available": _check_package_available("robot_descriptions"),
            "returns": "RobotWrapper instance",
            "features": ["Pre-defined robot models", "Easy robot switching"]
        },
        "yourdfpy": {
            "description": "Load with yourdfpy (for visualization)",
            "available": _check_package_available("yourdfpy"),
            "returns": "URDF object",
            "features": ["Visualization support", "Mesh loading", "Scene graphs"]
        }
    }
    
    return loaders


def list_available_robots(loader: str = "robot_description") -> list:
    """List available robot descriptions for a given loader.
    
    Args:
        loader: Loader type to check for available robots
        
    Returns:
        list: Available robot names
    """
    if loader == "robot_description":
        if not _check_package_available("robot_descriptions"):
            return []
        
        try:
            import robot_descriptions
            # Get all available robot descriptions with URDF format
            available_robots = []
            
            # Check if DESCRIPTIONS attribute exists
            if hasattr(robot_descriptions, 'DESCRIPTIONS'):
                descriptions = robot_descriptions.DESCRIPTIONS
                for robot_name, robot_info in descriptions.items():
                    # Check if the robot has URDF format available
                    if hasattr(robot_info, 'has_urdf') and robot_info.has_urdf:
                        available_robots.append(robot_name)
                    # Fallback: check if robot_info has URDF_PATH attribute
                    elif hasattr(robot_info, 'URDF_PATH'):
                        available_robots.append(robot_name)
            else:
                # Fallback to original method if DESCRIPTIONS not available
                for attr_name in dir(robot_descriptions):
                    if attr_name.startswith('_'):
                        continue
                    attr_obj = getattr(robot_descriptions, attr_name)
                    if hasattr(attr_obj, 'URDF_PATH'):
                        available_robots.append(attr_name)
            
            return sorted(available_robots)
        except Exception:
            return []
    
    return []

