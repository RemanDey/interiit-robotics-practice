from setuptools import find_packages, setup

package_name = 'object_mapping'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/semantic_mapping.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='remandey',
    maintainer_email='reman.airport@gmail.com',
    description='Semantic mapping with YOLOv8 segmentation and 3D object localization',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'semantic_mapping = object_mapping.object_mapping_node:main',
        ],
    },
)