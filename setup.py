from setuptools import setup, find_packages

from os import path

this_directory = path.abspath(path.dirname(__file__))
with open(path.join(this_directory, 'README.md'), encoding='utf-8') as f:
    readme = f.read()

setup(
    name='vehicle_mrac',
    version='0.0.1',
    description='',
    keywords='autonomous automated vehicles',
    url='',
    author='Kohei Kumazaki, UTokyo',
    author_email='',
    packages=find_packages(exclude=['test']),
    long_description_content_type='text/markdown',
    long_description=readme,
    classifiers=[
        "Programming Language :: Python :: 3.6",
        "License :: OSI Approved :: GNU General Public License v3 (GPLv3)",
        "Operating System :: POSIX :: Linux",
        "Operating System :: MacOS",
    ],
    data_files=[('.', ['LICENSE.txt'])],
    include_package_data=True,
)
