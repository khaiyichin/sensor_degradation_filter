import setuptools

print("pkgs", setuptools.find_packages())


setuptools.setup(
    name="sensor_degradation_filter_py",
    version="1.0.0",
    description= "Python library for the sensor degradation filter",
    author="Khai Yi Chin",
    author_email="khaiyichin@gmail.com",
    url="https://github.com/khaiyichin/sensor-degradation-filter",
    packages=setuptools.find_packages(),
    scripts=[]
)