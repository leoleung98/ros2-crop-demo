from setuptools import setup

package_name = 'row_follower'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/row_follow_launch.py']),
        ('share/' + package_name + '/world', ['world/crops.world']),
        ('share/' + package_name, ['LICENSE']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='leoleungphd',
    maintainer_email='leoleungphd@todo.todo',
    description='Row follower demo',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perception = row_follower.perception:main',
            'controller = row_follower.controller:main',
        ],
    },
)

