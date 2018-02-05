#!/usr/bin/python

############################################
## bag2csv.py                             ##
##                                        ##
## place rosbag data into a csv file      ##
##                                        ##
## Ian de Vlaming, dvlaming@stanford.edu  ##
## 02/04/2018                             ##
############################################

import rosbag, sys
import pandas as pd

# main (self, string bagFileName, string saveFileName)
# converts a bag file bagFileName into a csv file saveFileName
if __name__ == '__main__':
  # get bag name
  bagFileName = sys.argv[1]
  # unpack bag file
  bagObj = rosbag.Bag(bagFileName)
  # get bag name
  trimmedBagName = bagObj.filename
  # if a save file is specified
  if len(sys.argv) == 3:
    # set the save file
    saveFileName = sys.argv[2]
  # else
  else:
    # save file is the bag name
    saveFileName = trimmedBagName + '.csv'
  # create pandas dataframe from the bag file contents
  df = pd.DataFrame(bagObj.read_messages())
  # save to csv
  df.to_csv(saveFileName)
    
  
