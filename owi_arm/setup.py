## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['owi_arm'],
    package_dir={'': 'src'},
    requires=['rospy',"std_msgs", "sensor_msgs", './scripts/cmd_robot.py','./scripts/owi_joystick_node.py','./scripts/toy_joint_states.py','./scripts/arm_controller.py']
)

setup(**setup_args)
