import yaml
import numpy as np
import enum
import os
from urdf_parser_py.urdf import URDF
# from urdfpy import URDF

from ament_index_python.packages import get_package_share_directory

# Full path to the output file
pkg_path = os.path.join(get_package_share_directory('rov_sim'))
xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
print(xacro_file)
robot = URDF.from_xml_file(xacro_file)

