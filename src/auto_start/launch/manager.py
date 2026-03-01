from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
import os.path as os_path
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import sys
import yaml

class AutoStarter:
    def __init__(self, ld: LaunchDescription):
        self.__share_dir = {}
        self.__ld = ld
        
    def __addShareDir(self, pkg_name:str):
        if pkg_name not in self.__share_dir:
            self.__share_dir[pkg_name] = FindPackageShare(package=pkg_name).find(pkg_name)
        
    def startLaunch(self, pkg_name:str, dir_name:str, launch_name:str):
        self.__addShareDir(pkg_name=pkg_name) 
        launch_file = os_path.join(self.__share_dir[pkg_name], dir_name, launch_name)
        self.__ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_file)
            )
        )
        
    def startNode(self, package:str, executable:str, params:list=None, args:list=None):
        start_node = Node(package=package, executable=executable, parameters=params, arguments=args)
        self.__ld.add_action(start_node)
        
    def loadParamsFromYaml(self, config_file:str)->list:
        with open(config_file, 'r') as file:  
            data = yaml.safe_load(file) 
        
        flattened_params = {}  

        def flatten(obj, prefix=""):  
            if isinstance(obj, dict):  
                for k, v in obj.items():  
                    if prefix: flatten(v, prefix + "." + k)  
                    else: flatten(v, k)  
            else: flattened_params[prefix] = obj  
            
        flatten(data)
        
        params_list = []
        for key, value in flattened_params.items():  
            params_list.append({key: value})  
        return params_list
        
    