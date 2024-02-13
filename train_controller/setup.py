from setuptools import find_packages, setup

package_name = 'train_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'launch/train.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zachary',
    maintainer_email='alves.zach26@gmail.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sign_detection = train_controller.sign_detection:main',
            'controller = train_controller.controller:main'
        ],
    },
)
