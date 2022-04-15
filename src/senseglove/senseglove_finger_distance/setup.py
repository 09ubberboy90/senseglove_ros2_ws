from setuptools import setup

package_name = 'senseglove_finger_distance'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SenseGlove,Florent Audonnet',
    maintainer_email='rogierkrijnen@live.nl,2330834a@student.gla.ac.uk',
    description='A feature package that implements gain scheduling control',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'senseglove_finger_distance_node = senseglove_finger_distance.finger_distance_node:main',
            'senseglove_haptics_node = senseglove_finger_distance.haptics_node:main',
            'haptics_node_interface = senseglove_finger_distance.haptics_node_action_interface:main'
        ],
    },
)
