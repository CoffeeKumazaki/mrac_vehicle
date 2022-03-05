import setuptools
import os

readme_path = os.path.abspath(os.path.join(__file__, "..", "..", "README.md"))

with open(readme_path, "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="vehicle_dynamics",
    version="0.0.1",
    author="Kohei Kumazaki",
    author_email="kumazaki98@gmail.com",
    description="vehicle dynamics",
    long_description=long_description,
    long_description_content_type="text/markdown",
#    url="https://github.com/bcollazo/catanatron",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires=">=3.7",
)
