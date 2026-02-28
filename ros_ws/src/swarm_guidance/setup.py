from setuptools import find_packages, setup

package_name = 'swarm_guidance'

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
    maintainer='bavin',
    maintainer_email='bavin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "relative_vehicle_sim=swarm_guidance.relative_vehicle_sim:main",
            "swarm_interface=swarm_guidance.swarm_interface:main",
            "guidance=swarm_guidance.guidance:main"
        ],
    },
)
