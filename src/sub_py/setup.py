from setuptools import find_packages, setup

package_name = 'sub_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nate',
    maintainer_email='nwmarlin@nmsu.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'controller_input   = sub_py.controller_input:main'
            'depth_publisher    = sub_py.depth_publisher:main',
            'depth_monitor      = sub_py.depth_monitor:main',
            'depth_filterPY     = sub_py.depth_filter:main',
            "edge_detector      = sub_py.edge_detectLAB:main",
            'camera_publisher   = sub_py.camera_publisher:main',
            'lab_filter         = sub_py.lab_filter:main',
            'rail_detector      = sub_py.rail_detector:main',  
            'rail_follower      = sub_py.rail_follower:main',       
        ],
    },
)
