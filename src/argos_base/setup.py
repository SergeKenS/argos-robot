from setuptools import find_packages, setup

package_name = 'argos_base'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Paramètres mesurés du robot réel (lus par pont_moteurs)
        ('share/' + package_name + '/config', ['config/base.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='serge',
    maintainer_email='serge@todo.todo',
    description='Pilotes du robot réel : moteurs (carte Yahboom YB-ESF01)',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'pont_moteurs = argos_base.pont_moteurs:main',
        ],
    },
)
