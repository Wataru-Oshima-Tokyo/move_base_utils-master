import json

def read_json_file(file_name):
  with open(file_name, 'r') as f:
    json_load = json.load(f)
    return json_load

def write_json_file(file_name, dict_data):
  with open(file_name, 'w') as f:
    json.dump(dict_data, f, indent=2)

if __name__ == '__main__':

  print(json.__version__)

  f_name = "test.json"
  data = read_json_file(f_name)

  new_data = {"a":"10"}
  data.update(new_data)

  new_data = {"f":"6"}
  data.update(new_data)

  f_name = "test2.json"

  write_json_file(f_name, data)
