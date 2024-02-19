import json
from argparse import ArgumentParser


def get_str_from_value(row, name):
  if (name == "time"):
    return str(row[name])
  if (name == "pos"):
    return ""
  if (name[0:3] == "rod"):
    val = row[name]
    return str(val[0]) + " " + str(val[1]) + " " + str(val[2])
  return ""


def process_file(json_filename, txt_name):
  json_file = open(json_filename)
  json_data = json.load(json_file)

  names=["time","pos","rod_01_end_pt1","rod_01_end_pt2","rod_23_end_pt1","rod_23_end_pt2","rod_45_end_pt1","rod_45_end_pt2"]

  file_out = open(txt_name, "w")
  # file_out.write("# ")
  # for name in names:
  #   file_out.write(name + " ")
  # file_out.write("\n")

  values=[""]*len(names)
  for row in json_data:
    for ni in range(len(names)):
      name = names[ni]
      values[ni] = get_str_from_value(row, name)
      # file_out.write(get_str_from_value(row, name) + " ");
    for v in values:
      file_out.write(v + " ");
    file_out.write("\n")

  file_out.close()

if __name__ == '__main__':
  parser = ArgumentParser()
  parser.add_argument("-o", "--out", dest="out")
  parser.add_argument("-j", "--jason", dest="jason")
  args = parser.parse_args()
  process_file(args.jason, args.out)

