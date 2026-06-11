from setuptools import find_packages, setup

package_name = 'vineyard_nav'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/vineyard_row_demo.launch.py']),
        ('share/' + package_name + '/config', ['config/mission.yaml']),
    ],
    install_requires=['setuptools', 'PyYAML', 'numpy'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='Starter package for vineyard row following with Nav2 headland transitions.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'row_follower = vineyard_nav.row_follower:main',
            'mission_manager = vineyard_nav.mission_manager:main',
        ],
    },
)
