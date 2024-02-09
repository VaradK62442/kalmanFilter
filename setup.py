from setuptools import find_packages, setup

package_name = 'kalmanplayground'

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
    maintainer='varad',
    maintainer_email='varadkulk123@gmail.com',
    description='Kalman Filter Testing',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'function_publisher = kalmanplayground.function_publisher:main',
            'function_subscriber = kalmanplayground.function_subscriber:main',
            'kalman_filter = kalmanplayground.kalman_filter:main',
            'eval = kalmanplayground.eval:main'
        ],
    },
)
