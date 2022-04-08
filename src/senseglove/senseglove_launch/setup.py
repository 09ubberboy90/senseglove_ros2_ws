from setuptools import setup
from glob import glob
package_name = 'senseglove_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SenseGlove,Florent Audonnet',
    maintainer_email='rogierkrijnen@live.nl,2330834a@student.gla.ac.uk',
    description='provides a clear directory from which one can launch the senseglove',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
