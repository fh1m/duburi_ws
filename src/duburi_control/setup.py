from setuptools import setup, find_packages

package_name = 'duburi_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pymavlink'],
    zip_safe=True,
    maintainer='BRACU Duburi',
    description='Pure Python MAVLink API and movement commands for Duburi AUV',
    license='MIT',
    entry_points={
        'console_scripts': [],
    },
)
