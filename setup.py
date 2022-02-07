from setuptools import setup

package_name = 'trajectory_plotting'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team Spatzenhirn',
    maintainer_email='team-spatzenhirn@uni-ulm.de',
    description='Visualization for planned state-trajectory and control signal',
    license='MIT',
    entry_points={
        'console_scripts': [
            'plotter = trajectory_plotting.trajectory_plotting:main',
        ],
    },
)
