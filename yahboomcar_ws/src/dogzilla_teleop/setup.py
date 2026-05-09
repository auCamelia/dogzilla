import os
from glob import glob
from setuptools import setup

package_name = 'dogzilla_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'web'),    glob('dogzilla_teleop/web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='clp44',
    maintainer_email='clgpoulain@gmail.com',
    description='Web teleoperation interface for Dogzilla',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_server = dogzilla_teleop.web_server:main',
        ],
    },
)
