import math 
import numpy as np


def dh_transformation(a, d, apha, theta, array):
  ''' This function takes the Denavi-Hartenbery parameters and applies them to the 
      homogenous transformtion matrix '''
  # Set up matrixs with parameters
  trans = np.array([
          [math.cos(theta), -math.sin(theta)*math.cos(apha), math.sin(theta)*math.sin(apha), a*math.cos(theta)],
          [math.sin(theta), math.cos(theta)*math.cos(apha), -math.cos(theta)*math.sin(apha), a*math.sin(theta)],
          [0.0, math.sin(apha), math.cos(apha), d],
          [0.0, 0.0, 0.0, 1.0]
                    ])

  # Right multiple for new transformation 
  new_trans = np.matmul(array, trans)
  return new_trans


def kinematic_chain(dh_parameters):
  ''' This function takes the parameters and applies it to the dh_transformation.
      This will be called for every joint on the robot. '''
  # need an identify matrixs first to have a 4x4 
  transformations = np.identity(4)
  for i in range(len(dh_parameters)):
    transformations = dh_transformation(dh_parameters[i][0], dh_parameters[i][1], dh_parameters[i][2], dh_parameters[i][3], transformations)
    
  return transformations 


def get_pos(transformed_matrix):
  ''' This function takes the homogeneous transformation matrix
      and return the x, y, and z '''
  return transformed_matrix[0][3], transformed_matrix[1][3], transformed_matrix[2][3]


def get_rot(transformed_matrix):
  ''' This function takes the homogeneous transformation matrix
      and return the roll, pitch, and yaw '''
  roll = math.atan(transformed_matrix[1][0]/transformed_matrix[0][0])
  pitch = math.atan(-transformed_matrix[2][0]/(math.sqrt(transformed_matrix[2][1]**2 + transformed_matrix[2][2]**2)))
  yaw = math.atan(transformed_matrix[2][1]/transformed_matrix[2][2])
  return roll, pitch, yaw

if __name__ == '__main__':
  np.set_printoptions(precision=2, suppress=True)
  # Matrix for one joint should be set up let [a, d, apha, theta]
  
  problem_2a = kinematic_chain(np.array([[1.0, 0.0, 0.0, math.pi/2],
                                        [1.0, 0.0, 0.0, math.pi/2]]))
  print('\nProblem 2a: \n',problem_2a)                                        
  x,y,z = get_pos(problem_2a)
  print('x=',x, '\ny=',y, '\nz=',z)
  roll, pitch, yaw = get_rot(problem_2a)
  print('roll=', roll, '\npitch=', pitch, '\nyaw=', yaw)

  problem_2b = kinematic_chain(np.array([[0.0, 0.0, 0.0, 0],
                                        [0.0, 0.0, 0.0, 0],
                                        [0.0, 0.0, 0.0, 0],
                                        [0.0, 0.0, 0.0, 0],
                                        [0.0, 0.0, 0.0, 0],
                                        [0.0, 0.0, 0.0, 0]]))
  print('\nProblem 2b: \n',problem_2b)                                        
  x,y,z = get_pos(problem_2b)
  print('x=',x, '\ny=',y, '\nz=',z)
  roll, pitch, yaw = get_rot(problem_2b)
  print('roll=', roll, '\npitch=', pitch, '\nyaw=', yaw)

  problem_2c = kinematic_chain(np.array([[0.0, 0.0, 0.0, 0],
                                        [0.0, 0.0, 0.0, -math.pi/2],
                                        [0.0, 0.0, 0.0, 0],
                                        [0.0, 0.0, 0.0, 0],
                                        [0.0, 0.0, 0.0, 0],
                                        [0.0, 0.0, 0.0, 0]]))
  print('\nProblem 2c: \n',problem_2c)                                        
  x,y,z = get_pos(problem_2c)
  print('x=',x, '\ny=',y, '\nz=',z)
  roll, pitch, yaw = get_rot(problem_2c)
  print('roll=', roll, '\npitch=', pitch, '\nyaw=', yaw)