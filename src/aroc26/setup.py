from setuptools import find_packages, setup

package_name = 'aroc26'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/aroc26.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='afifi',
    maintainer_email='adekmail31@gmail.com',
    description='Launch file utama AROC26',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [],
    },
)
