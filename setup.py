from setuptools import find_packages, setup

package_name = 'gui_pkg'

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
    maintainer='alperenarda',
    maintainer_email='alperenarda@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "rover_gui_1_0 = gui_pkg.rover_gui_1_0:main",
            "rover_gui_1_1 = gui_pkg.rover_gui_1_1:main",
            "rover_gui_1_2 = gui_pkg.rover_gui_1_2:main",
            "rover_gui_1_3 = gui_pkg.rover_gui_1_3:main",
            "websocketdeneme = gui_pkg.websocketdeneme:main",
            "qrdeneme = gui_pkg.qrdeneme:main"
        ],
    },
)
