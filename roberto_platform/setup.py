from setuptools import find_packages, setup

package_name = 'roberto_platform'
submodules = "roberto_platform/submodules"

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
    maintainer='dJPoida',
    maintainer_email='djpoida@gmail.com',
    description='Nodes and Services to be run on the Roberto platform',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts' : [
            'pub = roberto_platform.pub:main',
            'sub = roberto_platform.sub:main',
            'power_monitor = roberto_platform.power_monitor:main',
        ],
    },
)
