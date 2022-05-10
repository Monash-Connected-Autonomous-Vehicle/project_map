from setuptools import setup

package_name = 'icp_py_localiser'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name,'icp_py_localiser/ros2_numpy'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mcav',
    maintainer_email='mcav@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'icp_local = icp_py_localiser.pose_path:main'
        ],
    },
)
