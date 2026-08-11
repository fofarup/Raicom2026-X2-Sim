#!/usr/bin/env python3
"""Print and validate the selected robot profile without starting ROS motion."""

from robot_profile import ProfileError, load_robot_profile


try:
    profile = load_robot_profile()
except ProfileError as exc:
    print(f"FAIL {exc}")
    raise SystemExit(2)
else:
    print(f"PASS profile={profile['profile_name']} kind={profile['robot_kind']}")
    print(f"file={profile['_path']}")
    print(f"ROS_DOMAIN_ID={profile['runtime']['ros_domain_id']}")
    print(f"RMW_IMPLEMENTATION={profile['runtime']['rmw_implementation']}")
