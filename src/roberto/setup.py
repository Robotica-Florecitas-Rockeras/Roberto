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
    maintainer='elipson',
    maintainer_email='elipson@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "roberto_teleop = roberto.roberto_teleop:main",
            "roberto_interface = roberto.roberto_interface:main",
            "roberto_player = roberto.roberto_player:main",
            "roberto_motor = roberto.roberto_motor:main"
        ],
    },
)
