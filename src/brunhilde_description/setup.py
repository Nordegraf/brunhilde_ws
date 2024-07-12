import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'brunhilde_description'

# convert the xacro files to urdf
xacro_path = os.path.join('resource', 'xacro', 'brunhilde.urdf.xacro')
urdf_path = os.path.join('resource', 'urdf', 'brunhilde.urdf')

# os.system(f'ros2 run xacro xacro {xacro_path} > {urdf_path}')

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'resource/xacro/meshes'), glob(os.path.join('resource', 'xacro/meshes/*.stl'))),
        (os.path.join('share', package_name, 'resource/xacro/collisions'), glob(os.path.join('resource', 'xacro/collisions/*.stl'))),
        (os.path.join('share', package_name, 'resource/xacro'), glob(os.path.join('resource', 'xacro/*.xacro'))),
        (os.path.join('share', package_name, 'resource/rviz'), glob(os.path.join('resource', 'rviz/*.rviz'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Niklas Conen',
    maintainer_email='niklas.conen@student.kit.edu',
    description="Description files for the Solo8 based robot of the KIT's HCR Lab.",
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'make_urdf = brunhilde_description.make_urdf:make_urdf',
        ],
    },
)
