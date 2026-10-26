from setuptools import find_packages, setup

package_name = 'argos_mission'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Points nommés (lus par aller_a.py) et paramètres Nav2 (lus par argos.launch.py)
        ('share/' + package_name + '/config',
            ['config/points.yaml', 'config/nav2_params.yaml']),
        ('share/' + package_name + '/launch', ['launch/argos.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='serge',
    maintainer_email='serge@todo.todo',
    description='Missions du robot : déplacements vers des points nommés',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # nom de la commande = module:fonction à appeler
            'aller_a = argos_mission.aller_a:main',
            'repartiteur = argos_mission.repartiteur:main',
        ],
    },
)
