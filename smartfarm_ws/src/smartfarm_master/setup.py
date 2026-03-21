from setuptools import find_packages, setup

package_name = 'smartfarm_master'

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
    maintainer='thumb',
    maintainer_email='211345177+thumb0907@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'master_node  = smartfarm_master.master_node:main',
            'monitor_node = smartfarm_master.monitor_node:main',
        ],
    },
)
