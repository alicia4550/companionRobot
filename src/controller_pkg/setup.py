from setuptools import setup
import os
from glob import glob

package_name = 'controller_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vivianweli',
    maintainer_email='liv8@mcmaster.ca',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "controller = controller_pkg.main:main",
            "undock = controller_pkg.undock_node:main",
            "dock = controller_pkg.dock_node:main"
        ],
    },
)
