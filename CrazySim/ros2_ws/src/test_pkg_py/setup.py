from setuptools import setup

package_name = 'test_pkg_py'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mrl-seuk',
    maintainer_email='seosu99@seoultech.ac.kr',
    description='Crazyflie interface package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cf_interface = test_pkg_py.cf_interface:main',
        ],
    },
)

