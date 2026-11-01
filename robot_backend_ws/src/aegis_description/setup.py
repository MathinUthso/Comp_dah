import os
from glob import glob

from setuptools import setup

package_name = 'aegis_description'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'urdf'),
            glob('urdf/*')),
        (os.path.join('share',package_name,'launch'),
            glob('launch/*')),
        (os.path.join('share',package_name,'meshes'),
            glob('meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mathinonthemoon',
    maintainer_email='mathinahmeds@gmail.com',
    description='TODO: Package description',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
