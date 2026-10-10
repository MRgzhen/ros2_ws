from setuptools import find_packages, setup

package_name = "mycobot_learn"

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
    maintainer="gz",
    maintainer_email="MRgzhen@users.noreply.github.com",
    description="TODO: Package description",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "listener = mycobot_learn.listener:main",
            "talk = mycobot_learn.talk:main",
            "image_sub = mycobot_learn.image_sub:main",
            "image_sub1 = mycobot_learn.image_sub1:main",
            "image_sub11 = mycobot_learn.image_sub11:main",
            "image_sub2 = mycobot_learn.image_sub2:main",
            "image_sub22 = mycobot_learn.image_sub22:main",
            "image_sub3 = mycobot_learn.image_sub3:main",
            "image_sub31 = mycobot_learn.image_sub31:main",
            "arm_hello_moveit = mycobot_learn.arm_hello_moveit:main",
            "arm_move_to_object = mycobot_learn.arm_move_to_object:main",
        ],
    },
)
