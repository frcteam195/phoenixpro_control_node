import jinja2
import yaml
import pathlib
import argparse
from typing import List
parser = argparse.ArgumentParser(description='Render script')
parser.add_argument('--output', action="store", dest='output_file', default="")
parser.add_argument('--template', action="store", dest='template_file', default="")
args = parser.parse_args()

node_name = pathlib.Path(__file__).parent.parent.name
robot_name = ""

for item in pathlib.Path.cwd().parent.iterdir():
    if item.is_dir():
        if item.name.count("_Robot") > 0:
            robot_name = str(item.resolve())

if robot_name == "":
    raise Exception("Unable to determine robot project folder")

with open(robot_name + '/config/' + node_name + '.yaml', 'r') as stream:
    try:
        yaml_obj = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        print("Error loading yaml file")
        raise exc


type_map = {
    None : "rclcpp::PARAMETER_NOT_SET",
    bool : "rclcpp::PARAMETER_BOOL",
    int : "rclcpp::PARAMETER_INTEGER",
    float : "rclcpp::PARAMETER_DOUBLE",
    str : "rclcpp::PARAMETER_STRING",
    bytes : "rclcpp::PARAMETER_BYTE_ARRAY",
    6 : "rclcpp::PARAMETER_BOOL_ARRAY",
    7 : "rclcpp::PARAMETER_INTEGER_ARRAY",
    8 : "rclcpp::PARAMETER_DOUBLE_ARRAY",
    9 : "rclcpp::PARAMETER_STRING_ARRAY",
}


# # Generate unit test template
env = jinja2.Environment(loader=jinja2.FileSystemLoader('/'), trim_blocks=True)
template = env.get_template(args.template_file)

keys=[]
values=[]
for (k, v) in yaml_obj[node_name]["ros__parameters"].items():
    keys.append(str(k))
    if type(v) is list:
        if all(isinstance(n, bool) for n in v):
            values.append(type_map[6])
        elif all(isinstance(n, int) for n in v):
            values.append(type_map[7])
        elif all(isinstance(n, float) for n in v):
            values.append(type_map[8])
        elif all(isinstance(n, str) for n in v):
            values.append(type_map[9])
        else:
            values.append(type_map[None])
        pass
    else:
        values.append(type_map[type(v)])

result = template.render(keys=keys, values=values, node_name=node_name)
f = open(args.output_file, "w")
f.write(result)
f.close()