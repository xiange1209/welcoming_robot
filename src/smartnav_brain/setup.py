from setuptools import find_packages, setup

package_name = "smartnav_brain"

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
    maintainer="swient",
    maintainer_email="swient@todo.todo",
    description="SmartNav 業務決策中樞：身分驗證與使用者管理、迎賓劇本與行員通報、導航仲裁（nav_arbiter）與權限規則",
    license="MIT",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "user_auth = smartnav_brain.user_auth_node:main",
            "bank_reception = smartnav_brain.bank_reception_node:main",
            "nav_arbiter = smartnav_brain.nav_arbiter_node:main",
        ],
    },
)
