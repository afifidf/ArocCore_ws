from setuptools import find_packages, setup

package_name = 'KickNRush'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ============================================================
        # [LAUNCH] Daftarkan launch file agar bisa digunakan dengan:
        #   ros2 launch KickNRush KickNRush.launch.py
        # ============================================================
        ('share/' + package_name + '/launch', ['launch/KickNRush.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='afifi',
    maintainer_email='adekmail31@gmail.com',
    description='Package untuk logika orbit, crab walk, dan goal alignment robot OP3 AROC26',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # Entry point utama KickNRush (task control + orbit + goal alignment)
            # Jalankan dengan: ros2 run KickNRush kicknrush
            'kicknrush = KickNRush.main:main',
        ],
    },
)
