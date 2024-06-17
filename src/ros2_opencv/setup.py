from setuptools import setup

package_name = 'ros2_opencv'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eddy',
    maintainer_email='eddy@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={ 'console_scripts': [ 'publisher_node = ros2_opencv.cameraPublisher:main', 'subscriber_node = ros2_opencv.subscriberImage:main', ], },
)
