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
        # Installe les points nommés, lus par aller_a.py
        ('share/' + package_name + '/config', ['config/points.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='serge',
    maintainer_email='serge@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # nom de la commande = module:fonction à appeler
            'aller_a = argos_mission.aller_a:main',
        ],
    },
)
