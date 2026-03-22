from setuptools import setup

package_name = 'smartfarm_pi1'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'pi1_node = smartfarm_pi1.pi1_node:main',
        ],
    },
)
