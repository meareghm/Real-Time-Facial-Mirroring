#!/usr/bin/python3
import sys
import rospy
import os

import roslib
#import cv2

from pau2motors.msg import pau
from dlib_puppeteering.msg import lm_points
roslib.load_manifest('dlib_puppeteering')

import random
import numpy
import numpy as np
import pandas
from sklearn.externals import joblib
from sklearn import model_selection
from sklearn.linear_model import Ridge

class dlib_puppeteering:
  
  # declare publishers, subscribers, and other important static variables
  def __init__(self):

    self.dlibX = []
    self.dlibY = []

    self.dlibFaceIndex = []
    self.indexes_brow = [0, 1]  # shapekey indexes to be mapped
    self.indexes_mouth = [30]
    self.indexes_lipupperleft= [24, 26, 28] #[30, 32]
    self.indexes_smileleft= [30]#[24, 25, 30, 31]
    self.indexes_smileright= [31]#[24, 25, 30, 31]
    self.indexes_lowerlipdown = [36, 38, 40]
    self.homedir = os.getenv("HOME")
    
    self.pub_pau = rospy.Publisher('/blender_api/set_pau', pau, queue_size=10)
    self.image_sub = rospy.Subscriber("/dlib_values", lm_points, self.dlib_callback)

    self.blendshape_names = ['brow_center_UP', 'brow_center_DN', 'brow_inner_UP.L', 'brow_inner_DN.L', 'brow_inner_UP.R',
                          'brow_inner_DN.R', 'brow_outer_UP.L', 'brow_outer_DN.L', 'brow_outer_up.R', 'brow_outer_DN.R',
                          'eye-flare.UP.L', 'eye-blink.UP.L', 'eye-flare.UP.R', 'eye-blink.UP.R', 'eye-blink.LO.L',
                          'eye-flare.LO.L', 'eye-blink.LO.R', 'eye-flare.LO.R', 'wince.L', 'wince.R', 'sneer.L',
                          'sneer.R', 'eyes-look.dn', 'eyes-look.up', 'lip-UP.C.UP', 'lip-UP.C.DN', 'lip-UP.L.UP',
                          'lip-UP.L.DN', 'lip-UP.R.UP', 'lip-UP.R.DN', 'lips-smile.L', 'lips-smile.R', 'lips-wide.L',
                          'lips-narrow.L', 'lips-wide.R', 'lips-narrow.R', 'lip-DN.C.DN', 'lip-DN.C.UP', 'lip-DN.L.DN',
                          'lip-DN.L.UP', 'lip-DN.R.DN', 'lip-DN.R.UP', 'lips-frown.L', 'lips-frown.R','lip-JAW.DN']


     
    # load the models from disk
    path = os.getcwd()+'/models'
    mouthleft_path= path + '/mouthleft_model.sav'
    self.mouthleft_model = joblib.load(mouthleft_path)

    lipupperleft_path= path + '/lipupperleft_model.sav'
    self.lipupperleft_model = joblib.load(lipupperleft_path)

    leftsmile_path= path + '/leftsmile_model.sav'
    self.leftsmile_model = joblib.load(leftsmile_path)

    rightsmile_path= path + '/rightsmile_model.sav'
    self.rightsmile_model = joblib.load(rightsmile_path)

    ridgebrow_path= path + '/ridgebrow_model.sav'
    self.ridgebrow_model = joblib.load(ridgebrow_path)

  
  # callback: gets relative distance of DLIB's land marks per their corresponding shapekey
  def dlib_callback(self, data):

      self.dlibX = data.dlib_X
      self.dlibY = data.dlib_Y
      self.dlibFaceIndex = data.dlib_face_index

      head_pau = pau()

      head_pau.m_headRotation.x = 0#0.9
      head_pau.m_headRotation.y = 0#0.5
      head_pau.m_headRotation.z = 0#0.7
      head_pau.m_headRotation.w = 0#0.9

      head_pau.m_headTranslation.x =0#0.9
      head_pau.m_headTranslation.y =0#0.7
      head_pau.m_headTranslation.z =0#0.8

      head_pau.m_neckRotation.x= 1#-0.9
      head_pau.m_neckRotation.y= 1#-0.5
      head_pau.m_neckRotation.z= 1#0.5
      head_pau.m_neckRotation.w= 1#0.9

      head_pau.m_eyeGazeLeftPitch = 0.01
      head_pau.m_eyeGazeLeftYaw = 0.1
      head_pau.m_eyeGazeRightPitch = 0.01
      head_pau.m_eyeGazeRightYaw = 0.1
      head_pau.m_shapekeys = self.blendshape_names
      head_pau.m_coeffs = self.doMapping()

      self.pub_pau.publish(head_pau) # publish to "/blender_api/set_pau"

     
  #def neck_detection(self):

  def doMapping(self):
      xy_normalized = []
      x_new = []
      
      # brows up 256x256 neutral expressions of dlib points from x[17:27] to y[17:27] (points most important during brows up)
      # neutral_brows2  = ([70, 80, 93, 106, 119, 138, 150, 162, 174, 183, 90, 81, 79, 81, 85, 86, 81, 79, 81, 89])
      #neutral_brows_x = ([72, 82, 95, 108, 121, 139, 151, 163, 175, 183])
      #neutral_brows_y = ([87, 79, 76, 78, 83, 83, 78, 76, 79, 87])
      # For Browsup_normalized_new.csv (minimum (0) value at index 89)
      neutral_brows_x = ([67, 75, 87, 99, 110, 129, 140, 151, 162, 169])
      neutral_brows_y = ([84, 76, 74, 77, 81, 81, 77, 75, 78, 86])
      #max_brow_x = ([69, 76, 87, 98, 108, 128, 138, 149, 159, 165])
      #max_brow_y = ([82, 74, 72, 72, 77, 78, 74, 74, 78, 86])
      #neutral_brows_x_scaled  = ([180.0, 205.0, 237.5, 270.0, 302.5, 347.5, 377.5, 407.5, 437.5, 457.5])
      #neutral_brows_y_scaled = ([163.125, 148.125, 142.5, 146.25, 155.625, 155.625, 146.25, 142.5, 148.125, 163.125])
      #neutral_brows_scaled = ([70, 79, 93, 107, 120, 138, 150, 163, 175, 184, 90, 81, 78, 80, 85, 85, 80, 79, 81, 90])
      #neutral_mouthleft_import_landmarks  = ([X48, X49, X50, X51, X57, X58, X59, X60, X61, X62, X66, X67])
      neutral_mouthleft = ([111, 120, 130, 129, 120, 116, 129])#, 130])
      #neutral_jawleft = ([X3, X4, X5, X6, X7, X8, X9, X49, X50, X51, X52, X53, X58, X59, X60, X61])
      neutral_jawleft  = ([68, 75, 85, 97, 113, 132, 152, 116, 126, 134, 141, 149, 125, 115, 112, 125])
      #neutral_mouthleft = ([140, 150, 159, 150, 140, 132, 110, 124, 132, 140, 154, 140, 132, 124])


      x = self.dlibX[0:67]
      y = self.dlibY[0:67]

      x_brow = x[17:27]
      y_brow = y[17:27]

      #facial_center_x = x[30]
      #facial_center_y = y[30]
      #headx_right = x[0]
      #headx_left = x[16]
      #heady_right = y[0]
      #heady_left = y[16]
      combine_x = [x[30],x[0],x[16]]
      #combine_y = [y[30],y[0],y[16]]
      print('combine_x_y')
      print(combine_x)
      #print(combine_y)
      head_direction_ratio =  (x[0] + x[16])/x[30]
      print("head_direction_ratio")
      print(head_direction_ratio)

      if head_direction_ratio > 2:
          #head_pau.m_headRotation.x = 0.9
          print("Sophia's left")
      if head_direction_ratio < 2:
          #head_pau.m_headRotation.x = 0.2
          print("Sophia's right")
      
      # To do (Machine based Head direction estimation) 
      '''head_pau.m_headRotation.x = 0#0.9
      head_pau.m_headRotation.y = 0#0.5

      #Rule based head direction estimation
      head_direction_ratio =  (x[0] + x[16])/x[30]
      print(head_direction_ratio)
      
      if head_direction_ratio > 0.66:
          head_pau.m_headRotation.x = 0.9
      if head_direction_ratio < 0.33:
          head_pau.m_headRotation.x = 0.2'''  

      x_i_jawleft = x[3:10]
      x_ii_jawleft = x[49:54]
      x_iii_jawleft = x[58:62]
      x_jawleft = x_i_jawleft + x_ii_jawleft + x_iii_jawleft
      
      x_jawleft_scaled = []
      for i in range(0, len(x_jawleft)):
          x_jawleft_scaled.append(x_jawleft[i]*256/640)
      
      x_jawleft_new = [neutral_jawleft[0], neutral_jawleft[1]]      
      for i in range(2, len(x_jawleft)):
          x_jawleft_new.append(x_jawleft_new[i-1] + ((x_jawleft_scaled[i] - x_jawleft_scaled[i-1]) * (neutral_jawleft[i-1] - neutral_jawleft[0])/(x_jawleft_scaled[i-1] - x_jawleft_scaled[0])))
      
      x_jawleft_numpy = numpy.array([(x_jawleft_new)])
      ##ridge_jawleft = self.ridge_jawleft()
      ##predicted_jawleft = ridge_jawleft.predict(x_jawleft_numpy)

      x_i_mouthleft = x[48:51]
      x_ii_mouthleft = x[58:62]
      #x_iii_mouthleft = x[67]
      x_mouthleft = x_i_mouthleft + x_ii_mouthleft# + x_iii_mouthleft
   
      x_mouthleft_scaled = []
      for i in range(0, len(x_mouthleft)):
          x_mouthleft_scaled.append(x_mouthleft[i]*256/640)

      x_mouthleft_new = [neutral_mouthleft[0], neutral_mouthleft[1]]
      for i in range(2, len(x_mouthleft)):
          x_mouthleft_new.append(x_mouthleft_new[0] + ((x_mouthleft_scaled[i] - x_mouthleft_scaled[0]) * (neutral_mouthleft[i-1] - neutral_mouthleft[0])/(x_mouthleft_scaled[i-1] - x_mouthleft_scaled[0])))

      x_mouthleft_numpy = numpy.array([(x_mouthleft_new)])
      predicted_mouthleft = self.mouthleft_model.predict(x_mouthleft_numpy)

      x_brow_scaled = []
      y_brow_scaled = []
      for i in range(0, len(x_brow)):
          x_brow_scaled.append(x_brow[i]*256/640)
      for i in range(0, len(y_brow)):
          y_brow_scaled.append(y_brow[i]*256/480)

      x_brow = x_brow_scaled
      y_brow = y_brow_scaled
      x_neutral = neutral_brows_x[0:10]
      y_neutral = neutral_brows_y[0:10]
      #x_neutral = neutral_brows_scaled[0:10]
      #y_neutral = neutral_brows_scaled[10:20]
      y16 = y[16]*256/480
      y0 = y[0]*256/480
      #print(y16-y0)
      x_brow_new = [x_neutral[0]]
      y_brow_new = [y_neutral[0]] 
      x16 = x[16]*256/640
      x0 = x[0]*256/640
      
      factor = ((174-57) / (x16 - x0)) # columns x[16] and x[0] of neutral value (0) browsup of 256 x 256 index 89.
      for i in range(1, len(x_brow)):
          x_brow_new.append(x_brow_new[i-1] + ((x_brow[i] - x_brow[i-1]) * factor))

      for i in range(1, len(y_brow)):
          y_brow_new.append(y_brow_new[i-1] + ((y_brow[i] - y_brow[i-1]) * factor))

      factor_brow_neutral = 101-98 # columns y[16] and y[0] of neutral value (0) browsup of 256 x 256 index 89.
      factor_brow_y = y16 - y0  
      total_factor = factor_brow_neutral - factor_brow_y 
      y_brow_newer = []
      for i in range(0, len(y_brow_new)):
          y_brow_newer.append(y_brow_new[i] - total_factor)
          #total_factor = total_factor-(total_factor/4)


      #to be fed to the ridge model
      xy_brow_new = x_brow_new + y_brow_newer
      brow_numpy = numpy.array([(xy_brow_new)])
      predicted_brow = self.ridgebrow_model.predict(brow_numpy)

      # y_lowerlipdown_open_neutral = [177, 184, 187, 187, 186, 182, 177, 174, 175, 173]
      y_lowerlipdown_open_i = y[55:60]
      y_lowerlipdown_open_ii = y[65:67]
      y_lowerlipdown_open_neutral = [184, 187, 187, 186, 182, 174, 175]
      y_lowerlipdown_open = y_lowerlipdown_open_i + y_lowerlipdown_open_ii 


      y_lowerlipdown_open_scaled = []
      for i in range(0, len(y_lowerlipdown_open)):
          y_lowerlipdown_open_scaled.append(y_lowerlipdown_open[i]*256/480)

      y_lowerlipdown_open = y_lowerlipdown_open_scaled
      y_lowerlipdown_open_new = [y_lowerlipdown_open_neutral[0]] 

      #factor = ((210-73) / (x16 - x0)) # columns x[16] and x[0] of neutral value (0) open.csv of 256 x 256 index 8.
      factor_y_lowerlipdown_open_neutral = 112-104 # columns y[16] and y[0] of neutral value (0) open.csv of 256 x 256 index 8.
      factor_y_lowerlipdown_open = y16 - y0  
      #total_factor = factor_y_lowerlipdown_open_neutral / factor_y_lowerlipdown_open # handle division by zeros

      for i in range(1, len(y_lowerlipdown_open)):
          y_lowerlipdown_open_new.append(y_lowerlipdown_open_new[i-1] + ((y_lowerlipdown_open[i] - y_lowerlipdown_open[i-1])))#* total_factor))

      x_i_leftsmile = x[51:58]
      x_ii_leftsmile = x[62:67]
      y_i_leftsmile = y[51:58]
      y_ii_leftsmile = y[62:67]
      x_leftsmile =  x_i_leftsmile + x_ii_leftsmile 
      y_leftsmile =  y_i_leftsmile + y_ii_leftsmile 

      x_leftsmile_scaled = []
      y_leftsmile_scaled = []

      for i in range(0, len(x_leftsmile)):
          x_leftsmile_scaled.append(x_leftsmile[i]*256/640)
      for i in range(0, len(y_leftsmile)):
          y_leftsmile_scaled.append(y_leftsmile[i]*256/480)

      #x_leftsmile = x_leftsmile_scaled
      #y_leftsmile = y_leftsmile_scaled

      neutral_leftsmile_x = ([126, 133, 141, 150, 142, 134, 126, 126, 133, 147, 133, 126]) 
      neutral_leftsmile_y = ([168, 165, 167, 171, 179, 183, 183, 173, 172, 171, 172, 173])
      x_leftsmile_new = [neutral_leftsmile_x[0]]
      y_leftsmile_new = [neutral_leftsmile_y[0]] 

   
      #y66 = y[66]*256/480
      #y51 = y[51]*256/480
      factor_smileleft_y = ((107-110) / (y[66] - y[51])) # columns y[16] and y[0] of neutral value (0)  left of 256 x 256 smile.csv index 0.'''
      for i in range(1, len(x_leftsmile)):
          x_leftsmile_new.append(x_leftsmile_new[i-1] + ((x_leftsmile[i] - x_leftsmile[i-1])))# * factor))

      for i in range(1, len(y_leftsmile)):
          y_leftsmile_new.append(y_leftsmile_new[i-1] + ((y_leftsmile[i] - y_leftsmile[i-1]) * factor))

      xy_leftsmile_new = x_leftsmile_new + y_leftsmile_new
      x_smile_left_new = x[48:52]
      
      #new leftsmile prediction 
      neutral_smile_new_left = ([102, 111, 119, 126])

      x_smile_new_left_scaled = []
      for i in range(0, len(x_smile_left_new)):
          x_smile_new_left_scaled.append(x_smile_left_new[i]*256/640)
      
      x_smile_left  = x_smile_new_left_scaled
      x_smile_new = [neutral_smile_new_left[0]]

      # factor x16 changed to 230 from 183; for better result (experimental)
      factor_smile_x_new = ((230-62) / (x16 - x0+1)) # 183 and 62 are columns x[16] and x[0] of neutral value (0) smile.csv of 256 x 256 index 2 respectively.
      for i in range(1, len(x_smile_left_new)):
          x_smile_new.append(x_smile_new[i-1] + ((x_smile_left[i] - x_smile_left[i-1]) * factor_smile_x_new))


      #factor_smile = abs(factor_smile_neutral - factor_smile_y)
      '''factor_smile_total_y = factor_smile_neutral_y - factor_smile_y 
      for i in range(1, len(smile_right_y)):
          y_smile_new.append((y_smile_new[i-1] + ((smile_right_y[i] - smile_right_y[i-1]) * factor_smile_x)) - factor_smile_total_y)'''

      #xy_smile_numpy = numpy.array(x_smile_new + y_smile_new)
      x_smile_numpy = numpy.array([(x_smile_new)])
      predicted_smileleft = self.leftsmile_model.predict(x_smile_numpy)

      neutral_lipupperleft_m_x = ([122, 128, 135, 142, 122, 128, 139])
      neutral_lipupperleft_m_y = ([172, 170, 171, 174, 176, 175, 175])

      x_lipupperleft_m_new = [neutral_lipupperleft_m_x[0]]
      y_lipupperleft_m_new = [neutral_lipupperleft_m_y[0]]

      x_i_lipupperleft_m = x[51:55]
      x_ii_lipupperleft_m  = x[62:65]
      y_i_lipupperleft_m = y[51:55]
      y_ii_lipupperleft_m  = y[62:65]

      x_lipupperleft_m =  x_i_lipupperleft_m + x_ii_lipupperleft_m 
      y_lipupperleft_m =  y_i_lipupperleft_m + y_ii_lipupperleft_m

      x_lipupperleft_m_scaled = []
      y_lipupperleft_m_scaled = []

      for i in range(0, len(x_lipupperleft_m)):
          x_lipupperleft_m_scaled.append(x_lipupperleft_m[i]*256/640)
      for i in range(0, len(y_lipupperleft_m)):
          y_lipupperleft_m_scaled.append(y_lipupperleft_m[i]*256/480)

      x_lipupperleft_m  = x_lipupperleft_m_scaled
      y_lipupperleft_m =  y_lipupperleft_m_scaled


      for i in range(1, len(x_lipupperleft_m)):
          x_lipupperleft_m_new.append(x_lipupperleft_m_new[i-1] + ((x_lipupperleft_m[i] - x_lipupperleft_m[i-1]))) #* factor_x))

      for i in range(1, len(y_lipupperleft_m)):
          y_lipupperleft_m_new.append(y_lipupperleft_m_new[i-1] + ((y_lipupperleft_m[i] - y_lipupperleft_m[i-1])))  #* factor_y))

      xy_lipupperleft_m_new = x_lipupperleft_m_new + y_lipupperleft_m_new

      lipupperleft_numpy = numpy.array([(xy_lipupperleft_m_new)])
      predicted_lipupperleft = self.lipupperleft_model.predict(lipupperleft_numpy)

      smile_right_neutral_x  = ([126, 133, 141, 150])
      #smile_right_neutral_x  = ([126, 133, 141, 150, 142, 134, 126, 126, 133, 146, 133, 126])
      smile_right_neutral_y = ([168, 165, 167, 171, 179, 183, 183, 173, 172, 171, 172, 173])


      smile_right_x = x[51:55]
      x_smile_new = [smile_right_neutral_x[0]] 
      x_smile_scaled = []
      for i in range(0, len(smile_right_x)):
          x_smile_scaled.append(smile_right_x[i]*256/640)

      smile_right_x =  x_smile_scaled


      # factor of x is changed from the 183 to 210; just for experiment
      factor_smile_x = ((200-63) / (x16 - x0+1)) # 200 and 63 are columns x[16] and x[0] of neutral value (0) smile_new1.csv of 256 x 256 index 4 respectively.

      factor_smile_y = (106-109) / (y16+1 - y0) # 106 and 109 are columns y[16] and y[0] of neutral value (0) smile_new1.csv of 256 x 256 index 4 respectively.


      for i in range(1, len(smile_right_x)):
          x_smile_new.append(x_smile_new[i-1] + ((smile_right_x[i] - smile_right_x[i-1]) * factor_smile_x))


      #factor_smile = abs(factor_smile_neutral - factor_smile_y)
      '''factor_smile_total_y = factor_smile_neutral_y - factor_smile_y 
      for i in range(1, len(smile_right_y)):
          y_smile_new.append((y_smile_new[i-1] + ((smile_right_y[i] - smile_right_y[i-1]) * factor_smile_x)) - factor_smile_total_y)'''


      #xy_smile_numpy = numpy.array(x_smile_new + y_smile_new)
      x_smile_numpy = numpy.array([(x_smile_new)])
      predicted_smileright = self.rightsmile_model.predict(x_smile_numpy)

      blendshape_values = []
      for i in range(0, 45): # modify this block to include the machine learning approach
          if i in self.indexes_brow:
              #random_val= random.random() # generate some random values between 0 and 1
              #blendshape_values.append(random_val)
              blendshape_values.append(predicted_brow) 

          if i in self.indexes_smileleft:
              blendshape_values.append(predicted_smileright)
              #blendshape_values.append(predicted_smileright)
          if i in self.indexes_smileright:
              blendshape_values.append(predicted_smileright)
          if i in self.indexes_lipupperleft:
              #blendshape_values.append(predicted_jawleft)
              #blendshape_values.append(predicted_lipupperleft)
              blendshape_values.append(predicted_smileright/1.3)
          else:
              blendshape_values.append(0.0)
      return blendshape_values

def main(args):
  rospy.init_node('dlib2blender_mapper', anonymous=True)
  dlib_puppeteering()
  try:
      rospy.spin()
  except KeyboardInterrupt:
      print("Dlib-Puppeteering Exiting...")
  #cv2.destroyAllWindows()

if __name__ == '__main__':
  main(sys.argv)







