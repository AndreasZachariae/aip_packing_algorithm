from setuptools import setup

package_name = 'r2e_tutorial'

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
    maintainer='robot',
    maintainer_email='gergely.soti@h-ka.de',
    description='Tutorial for ROS2 KUKA ready2educate cells with EKI driver',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'example = r2e_tutorial.example:main',
            'house = r2e_tutorial.house:main'
        ],
    },
)
