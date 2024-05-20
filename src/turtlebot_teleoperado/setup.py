from setuptools import find_packages, setup

package_name = 'turtlebot_teleoperado'

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
    maintainer='mario',
    maintainer_email='mario.medeiros@sou.inteli.edu.br',
    description='Entrega da ponderada do turtlebot teleoperado. Consiste em fazer um turtlebot3 burger andar por comandos de CLI.',
    license='CC0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "start_moving = turtlebot_teleoperado.turtlebot_teleoperado:main"
        ],
    },
)
