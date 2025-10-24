import os

from setuptools import find_packages, setup

# Read the contents of your README file
with open("README.md", "r", encoding="utf-8") as f:
    long_description = f.read()


def get_data_files():
    data_files = []
    for root, dirs, files in os.walk("assets"):
        root_files = [os.path.join(root, f) for f in files]
        data_files.append((root, root_files))
    return data_files


setup(
    name="kml-mission-planner",
    version="1.0.0",
    author="Vinícius Mattei",
    description="KML Mission Planning Application",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/yourusername/kml-mission-planner",
    packages=find_packages(),
    include_package_data=True,
    package_data={
        "": ["*.ui", "*.qrc", "*.png", "*.icns", "*.md"],
    },
    data_files=get_data_files(),
    install_requires=[
        "PyQt5>=5.15",
        "requests>=2.26",
        "lxml>=4.6",
        "python-dateutil>=2.8",
        "watchdog>=2.1",
    ],
    entry_points={
        "console_scripts": [
            "mission-planner=mission_planner:main",
        ],
    },
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.10",
        "Operating System :: OS Independent",
        "Topic :: Scientific/Engineering :: GIS",
    ],
    python_requires=">=3.10",
    options={
        "build_exe": {
            "include_files": [
                "README.md",
                "assets/",
            ],
        }
    },
)
