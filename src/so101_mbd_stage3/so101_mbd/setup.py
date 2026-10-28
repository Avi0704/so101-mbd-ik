from setuptools import setup

package_name = 'so101_mbd'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'fk_check = so101_mbd.fk_check:main',
            'jac_check = so101_mbd.jac_check:main',
        ],
    },
)
