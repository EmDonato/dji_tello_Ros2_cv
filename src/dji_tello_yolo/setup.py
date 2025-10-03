from setuptools import setup

package_name = 'dji_tello_yolo'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Emanuele Donato',
    maintainer_email='mcf.donato@email.com',
    description='YOLO person detection node for DJI Tello',
    license='MIT',
    entry_points={
        'console_scripts': [
            'yolo_person_node = dji_tello_yolo.yolo_person_node:main',
        ],
    },
)
