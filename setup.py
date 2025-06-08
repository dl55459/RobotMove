from setuptools import find_packages, setup

package_name = 'prarob_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/images', ['images/image.PNG']),
        ('share/' + package_name + '/launch', ['launch/image_process.launch.xml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fpetric',
    maintainer_email='f5r1c.1m0@gmail.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'image_view = prarob_vision.image_view:main',
                'image_process = prarob_vision.image_process:main',
        ],
    },
)
