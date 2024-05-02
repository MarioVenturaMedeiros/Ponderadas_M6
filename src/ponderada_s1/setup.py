from setuptools import find_packages, setup

package_name = 'ponderada-s1'

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
    maintainer='Mário',
    maintainer_email='mario.medeiros@sou.inteli.edu.br',
    description='Entrega da ponderada da primeira semana. Consiste em fazer um desenho no turtlesim.',
    license='CC0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "ponderada_s1 = ponderada_s1.ponderada_s1:main",
        ],
    },
)