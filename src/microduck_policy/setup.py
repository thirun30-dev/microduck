from setuptools import setup
setup(
    name="microduck_policy", version="0.1.0", packages=["microduck_policy"],
    data_files=[("share/ament_index/resource_index/packages", ["resource/microduck_policy"]),
                ("share/microduck_policy", ["package.xml"])],
    install_requires=["setuptools"], zip_safe=True,
    entry_points={"console_scripts": [
        "policy_node = microduck_policy.policy_node:main",
        "fetch_policies = microduck_policy.hub:main",
        "teleop = microduck_policy.teleop:main",
    ]},
)
