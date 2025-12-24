import string
import yaml
import configparser
from ament_index_python.packages import get_package_share_directory


def read_calibration_data(cali_file_path):
    """
    Read calibration data, return dictionary
    """
    with open(cali_file_path, 'r', encoding='utf-8')as f:
        result = yaml.load(f.read(), Loader=yaml.FullLoader)
    return result


def change_yaml_data(yaml_path: string, key: string, value: any):
    """ 
    修改yaml的字段 
    """
    with open(yaml_path, 'r', encoding='utf-8') as f:
        data = f.read()
        f.close()
        doc = yaml.safe_load(data[11:])
    # ros_node_dict = doc['/**']
    # ros_param_dict = ros_node_dict['ros__parameters']
    doc[key] = value
    f = open(yaml_path, "w+")
    f.write("%YAML:1.0\n\n")
    f.close()

    print("set " + key + " " + str(value))
    with open(yaml_path, 'a', encoding='utf-8') as f:
        yaml.safe_dump(doc, f, default_flow_style=False)
        f.close()
    return


def overwrite_calibration_data(cali_file_path, yaml_path):
    data_dict = read_calibration_data(cali_file_path)
    jaka_driver_dict = data_dict['jaka_driver']
    if 'desired_fx' in jaka_driver_dict.keys():
        change_yaml_data(yaml_path, "desired_fx",
                         jaka_driver_dict['desired_fx'])
    if 'desired_fy' in jaka_driver_dict.keys():
        change_yaml_data(yaml_path, "desired_fy",
                         jaka_driver_dict['desired_fy'])
    return
