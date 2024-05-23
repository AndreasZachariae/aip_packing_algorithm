# setup.py
from setuptools import setup
from Cython.Build import cythonize

setup(
    ext_modules = cythonize("cython_part.pyx")
)

"""
To build the extension module, run the following command in terminal:
python setup.py build_ext --inplace
"""

# Depending on the OS, the extension module will be built as a shared library .so (Linux) or a DLL .pyd (Windows) file. Maybe run and implement again, depending on the OS.