import yaml
import dataclass_wizard.wizard_cli as cli
import json
import os
# from pydict2class import PyDict2Class
# dict2class = PyDict2Class()

with open("parameters.yaml", 'r') as stream:
    params = yaml.safe_load(stream)
    parameters = cli.PyCodeGenerator(file_contents=json.dumps(params), experimental=True).py_code
    # print(dict2class.generate(params, "AutopilotParameters", json=True).sail_lookup_table_wind_angles)
    
if not os.path.exists('autogenerated/'):
    os.mkdir('autogenerated')
with open('autogenerated/parameter_definitions.py', 'x') as f:
    f.write(parameters)