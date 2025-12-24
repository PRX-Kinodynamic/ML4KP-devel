# Export numpy .npy datafile to csv, because we are human beings...
import argparse
import numpy as np

def export_npy(filename_in, filename_out):
  data = np.load(filename_in, 'r')
  file = open(filename_out, 'w');
  for row in data: # assuming [[],[]] ~ matrix in the python way
    line = ""
    for element in row:
      line += str(element) + " "
    line += "\n"
    file.write(line);

  file.close();

if __name__ == '__main__':
  argparse = argparse.ArgumentParser()
  argparse.add_argument('-f', '--file', help='Input file', required=True)
  argparse.add_argument('-o', '--out', help='Output file', required=True)
  args = argparse.parse_args()

  export_npy(args.file, args.out)

