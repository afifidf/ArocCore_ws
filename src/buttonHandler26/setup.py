from setuptools import find_packages, setup

package_name = 'buttonHandler26'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/buttonHandler.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='afifi',
    maintainer_email='adekmail31@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'button_soccer = buttonHandler.buttonHandler:main'
        ],
    },
)
