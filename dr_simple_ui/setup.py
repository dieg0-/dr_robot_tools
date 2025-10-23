from setuptools import find_packages, setup

package_name = 'dr_simple_ui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'nicegui>=3.1.0'],
    zip_safe=True,
    maintainer='Diego Ramos',
    maintainer_email='diego.enrique.ramos@googlemail.com',
    description='A web-based ui for basic teleoperation of a robot using NiceGUI and ROS2',
    license='BSD',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "ui = dr_simple_ui.ui:main",
        ],
    },
)
