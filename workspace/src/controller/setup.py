from setuptools import find_packages, setup

package_name = "controller"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="tariq",
    maintainer_email="tariqsoliman2000@gmail.com",
    description="Sends control actions to the Tello drone",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["controller = controller.main:main"],
    },
)
