from setuptools import find_packages, setup

package_name = 'roberto'

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
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts' : [
            'pub = roberto.pub:main',
            'sub = roberto.sub:main',
        ],
    },
)
