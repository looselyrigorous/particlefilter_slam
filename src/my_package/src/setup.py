from distutils.core import setup
from Cython.Build import cythonize

setup(ext_modules=cythonize('MapBuilder.pyx'))
setup(ext_modules=cythonize('Preprocess.pyx'))