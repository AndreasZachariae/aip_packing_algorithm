from setuptools import setup, find_packages

package_name = 'pick_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'load_container_cylinder = pick_package.two_packages_cylinder:load_container_cylinder',
            'pack_container = pick_package.examples:load_container_cylinder'            
            
        ],
    },
)

