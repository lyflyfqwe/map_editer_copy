import os

pkgs = [
    "auto_start",
    
    # "debug_tools",
    #  "rm_interface",
    # "rm_serial",
    # "livox_ros_driver2",
    # "rviz_plugin",
     "rm_decision",
    # "rm_decision_system", 
    
    # "altitude_map",
     "path_motion_planning",
    
    # "point_lio",
    # "fast_lio",
    # "ikd_tree_mapping",
    "icp_relocalization",
]

compile_params = [
    "--cmake-args",
    "-DCMAKE_BUILD_TYPE=Release",
    "--packages-select",
    # "--allow-overriding",
]

if(len(pkgs) != 0):
    command = "colcon build " + " ".join(compile_params) + " " + " ".join(pkgs)
    compile_result = os.system(command)
    print("\033[1;31m","command result:", compile_result, "\033[0m")