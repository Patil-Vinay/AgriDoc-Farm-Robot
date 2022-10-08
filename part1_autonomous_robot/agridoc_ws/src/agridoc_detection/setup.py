from setuptools import setup

package_name = 'agridoc_detection'

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
    maintainer='Vinay Patil',
    maintainer_email='patilb.vinay@gmail.com',
    description='Weed spraying and leaf disease logging on the AgriDoc robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'weed_detector = agridoc_detection.weed_detector:main',
            'disease_detector = agridoc_detection.disease_detector:main',
        ],
    },
)
