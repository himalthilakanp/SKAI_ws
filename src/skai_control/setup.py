from setuptools import find_packages, setup

package_name = 'skai_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/skai_control/launch',
    	    ['launch/moveit_controller.launch.py'],
	),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='smallpapaya',
    maintainer_email='smallpapaya@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    	'console_scripts': [
        	'moveit_controller = skai_control.moveit_controller:main',
    	],
    },
)
