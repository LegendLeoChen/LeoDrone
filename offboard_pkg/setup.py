from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'offboard_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/**')),
        (os.path.join('share', package_name, 'materials'), glob('materials/**')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/**')),
        (os.path.join('share', package_name, 'weights'), glob('weights/**')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='leo',
    maintainer_email='leo@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'offboard_node = offboard_pkg.offboard_node:main',
            'moveup = offboard_pkg.moveup:main',
            'circle = offboard_pkg.circle:main',
            'image = offboard_pkg.image:main',
            'cam = offboard_pkg.cam:main',
            'cam_pc = offboard_pkg.cam_pc:main',
            'image_detection = offboard_pkg.image_detection:main',
        ],
    },
)
