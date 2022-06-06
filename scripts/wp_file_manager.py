from json_read_write import *

class WpFileManager:
  def __init__(self, config_dir):
    self.config_dir = config_dir
    
    
    self.config_file_name = self.config_dir + "/config.json"
    self.config_data = read_json_file(self.config_file_name)

  def get_last_id(self):
    return self.config_data["last_file_id"]

  def get_wp(self, file_id):
    f_name = self.config_dir + "/" + str(file_id) + ".json"
    data = read_json_file(f_name)
    wp = data["wp"]

    self.config_data["last_file_id"] = file_id
    write_json_file(self.config_file_name, self.config_data)

    return wp

  def write_wp(self, file_id,  wp):
    wp_dic = {"wp":wp}

    last_id = self.config_data["last_file_id"]
    f_name = self.config_dir + "/" + str(file_id) + ".json"
    write_json_file(f_name, wp_dic)

if __name__ == '__main__':

  wm = WpFileManager("wp")

  f_id = wm.get_last_id()

  wp = wm.get_wp(1)

  print(wp)

  wp = [[0,0,0,0]]

  for i in range(10):
    wm.write_wp(i+1, wp)
