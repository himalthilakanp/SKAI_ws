from setuptools import find_packages, setup

package_name = 'skai_dashboard'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/dashboard.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='smallpapaya',
    maintainer_email='smallpapaya@todo.todo',
    description='SKAI live control dashboard',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dashboard = skai_dashboard.dashboard_node:main',
        ],
    },
)
