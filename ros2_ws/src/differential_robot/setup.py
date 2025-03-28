from setuptools import setup

package_name = 'differential_robot'

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
    maintainer='robotica',
    maintainer_email='c.mezap@uniandes.edu.co',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "turtle_bot_teleop = differential_robot.turtle_bot_teleop:main",
            "turtle_bot_interface = differential_robot.turtle_bot_interface:main",
            "turtle_bot_player = turtlebot_controller.turtle_bot_player:main"
        ],
    },
)
