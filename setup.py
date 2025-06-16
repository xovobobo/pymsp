import re
from setuptools import setup, find_packages

requirement_path = "requirements.txt"

with open(requirement_path, 'r', encoding="utf-8") as f:
    install_requires = f.read().splitlines()

with open("msp/__version__.py", encoding="utf-8") as f:
    version = re.findall(r"__version__ = \"(.+)\"", f.read())[0]

setup(
    name="msp",
    version=version,
    packages=find_packages(),
    install_requires=install_requires,
)