"""Setup script for fullstack-manip package."""

from setuptools import setup, find_packages

setup(
    name="fullstack-manip",
    version="0.1.0",
    description="Full-stack robotic manipulation framework",
    author="thanhndv212",
    packages=find_packages(),
    python_requires=">=3.8",
    install_requires=[
        "numpy>=1.20.0",
        "py-trees>=2.2.0",
    ],
    extras_require={
        "dev": [
            "pytest>=7.0.0",
            "black>=22.0.0",
            "flake8>=4.0.0",
        ],
    },
)
