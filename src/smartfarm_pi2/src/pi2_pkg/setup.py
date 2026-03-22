from setuptools import find_packages, setup

package_name = 'pi2_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='라파2 노드 패키지',
    license='MIT',
    entry_points={
        'console_scripts': [
            'pi2_node = pi2_pkg.pi2_node:main',
        ],
    },
)
