import os
import yaml

def load_config(path=os.path.join(os.path.expanduser("~"), "aJLL/Model/config.yaml")):
  assert os.path.isfile(path), "config.yaml not found."
  with open(path) as fp:
    config = yaml.load(fp, yaml.Loader)
  return config
