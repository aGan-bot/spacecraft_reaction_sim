from glob import glob
from setuptools import find_packages, setup

package_name = 'spacecraft_reaction_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='as',
    maintainer_email='agan.simsek@altinay.com',
    description='Free-floating spacecraft reaction simulation.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'reaction_experiment = spacecraft_reaction_sim.reaction_experiment:main',
        ],
    },
)
