from setuptools import find_packages, setup

package_name = 'turtle_ball_tracking'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ball_tracker.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='elbit18',
    maintainer_email='nir.blagovsky@mapcore.onmicrosoft.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'ball_image_publisher = turtle_ball_tracking.ball_image_publisher:main',
        'ball_tracker = turtle_ball_tracking.ball_tracker:main',
    ],
    },


)
