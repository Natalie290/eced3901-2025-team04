from setuptools import setup

package_name = 'amcl_localization'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Natalie290',
    maintainer_email='natalie@example.com',
    description='AMCL localization package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'amcl_node = amcl_localization.amcl_node:main',
        ],
    },
)