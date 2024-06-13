from setuptools import find_packages, setup, glob


package_name = 'autopilot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        ('lib/' + package_name, ['./nodes/autopilot_node.py']),
        ('lib/' + package_name, ['./nodes/telemetry_node.py'])
        # ('lib/' + package_name, ['./autopilot/']),
        # ('lib/' + package_name, ['./utils/']),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='animated',
    maintainer_email='73669160+ChrisNassif@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autopilot_node = autopilot_node:main',
            'telemetry_node = telemetry_node:main'
        ],
    },
)
