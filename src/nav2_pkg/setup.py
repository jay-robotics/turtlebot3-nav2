from setuptools import find_packages, setup

package_name = 'nav2_pkg'

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
    maintainer='jay',
    maintainer_email='jaygajjar890@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "scan=nav2_pkg.scan:main",
            "origin=nav2_pkg.origin:main",
            "command_pub=nav2_pkg.command_pub:main",
            "init=nav2_pkg.initial_pose:main",
            "roam=nav2_pkg.roaming:main",
        ],
    },
)
