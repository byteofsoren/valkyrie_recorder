from setuptools import setup

package_name = 'testnode'

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
    maintainer='Magnus Sörensen',
    maintainer_email='byteofsoren@gmail.com',
    description='Test pacage that tests the connection between PC and RPi.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = testnode.publisher_member_function:main',
            'listner = testnode.subscriber_member_function:main'
        ],
    },
)
