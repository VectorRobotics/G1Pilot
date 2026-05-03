from setuptools import find_packages, setup
import sys
import os
from glob import glob

package_name = 'humanoid_arm_skills'


def _strip_colcon_args(argv: list[str]) -> list[str]:
    cleaned: list[str] = []
    skip_next = False
    for arg in argv:
        if skip_next:
            skip_next = False
            continue
        if arg == "--editable":
            continue
        if arg == "--build-directory":
            skip_next = True
            continue
        if arg.startswith("--build-directory="):
            continue
        cleaned.append(arg)
    return cleaned


sys.argv = _strip_colcon_args(sys.argv)

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kushal Agarwal',
    maintainer_email='kushalagarwal444@gmail.com',
    description='Visual servoing action server for arms of humanoid robots',
    license='BSD',
    entry_points={
        'console_scripts': [
            'humanoid_arm_vs = humanoid_arm_skills.humanoid_arm_visual_servoing:main',
            'noisy_pose_publisher = humanoid_arm_skills.noisy_pose_publisher:main',
            'pose_filtering = humanoid_arm_skills.pose_filtering:main',
        ],
    },
)
