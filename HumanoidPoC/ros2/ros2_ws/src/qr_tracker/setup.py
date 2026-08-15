from setuptools import setup

package_name = "qr_tracker"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/qr_tracker.launch.py"]),
        ("share/" + package_name + "/params", ["params/qr_tracker.yaml"]),
    ],
    install_requires=["setuptools", "numpy", "opencv-python"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="you@example.com",
    description="QR-based fine positioning controller gated by Nav2 success flag.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "qr_tracker_node = qr_tracker.qr_tracker_node:main",
        ],
    },
)
