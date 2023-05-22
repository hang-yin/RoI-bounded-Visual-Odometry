from setuptools import setup

package_name = 'roi_bounded_visual_odometry'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'launch/main_launch.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yh6917',
    maintainer_email='yinhang0226@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'visual_odometry = roi_bounded_visual_odometry.visual_odometry:main'
        ],
    },
)
