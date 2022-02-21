import math 
import numpy as np


def dh_transformation(a, d, alpha, theta, array):
  ''' This function takes the Denavi-Hartenbery parameters and applies them to the 
      homogenous transformtion matrix '''
  # Set up matrixs with parameters
  trans = np.array([
          [math.cos(theta), -math.sin(theta)*math.cos(alpha), math.sin(theta)*math.sin(alpha), a*math.cos(theta)],
          [math.sin(theta), math.cos(theta)*math.cos(alpha), -math.cos(theta)*math.sin(alpha), a*math.sin(theta)],
          [0.0, math.sin(alpha), math.cos(alpha), d],
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
  roll = math.atan2(transformed_matrix[2][1], transformed_matrix[2][2])
  pitch = math.atan2(-transformed_matrix[2][0], (math.sqrt(transformed_matrix[2][1]**2 + transformed_matrix[2][2]**2)))
  yaw = math.atan2(transformed_matrix[1][0], transformed_matrix[0][0])
  
  return roll, pitch, yaw


def print_helper(pos, rot):
  print('\nx= ', round(pos[0],2), '\ny=', round(pos[1],2), '\nz=', round(pos[2],2))
  print('\nroll=',  round(rot[0],2), '\npitch=',  round(rot[1],2), '\nyaw=',  round(rot[2],2)) 
  print('__________________________________________________')


if __name__ == '__main__':
  np.set_printoptions(precision=2, suppress=True)
  # Matrix for one joint should be set up let [a, d, alpha, theta]
  problem_2a = kinematic_chain(np.array([[1.0, 0.0, 0.0, math.pi/2],
                                        [1.0, 0.0, 0.0, math.pi/2]]))
  print('\nProblem 2a: \n',problem_2a)                                        
  pos = get_pos(problem_2a)
  rot = get_rot(problem_2a)
  print_helper(pos, rot)

  problem_2_case_1 = kinematic_chain(np.array([[0.0, 0.1625,  math.pi/2, 0.0],
                                                [-0.425, 0.0, 0.0, 0.0],
                                                [-0.3922, 0.0, 0.0, 0.0],
                                                [0.0, 0.1333, math.pi/2, 0.0],
                                                [0.0, 0.0997, -math.pi/2, 0.0],
                                                [0.0, 0.0996, 0.0, 0.0]]))
  print('\nProblem 2b Case 1: \n',problem_2_case_1)                                                                           
  pos = get_pos(problem_2_case_1)
  rot = get_rot(problem_2_case_1)
  print_helper(pos, rot)

  problem_2_case_2 = kinematic_chain(np.array([[0.0, 0.1625,  math.pi/2, 0.0],
                                                [-0.425, 0.0, 0.0, -math.pi/2],
                                                [-0.3922, 0.0, 0.0, 0.0],
                                                [0.0, 0.1333, math.pi/2, 0.0],
                                                [0.0, 0.0997, -math.pi/2, 0.0],
                                                [0.0, 0.0996, 0.0, 0.0]]))
  print('\nProblem 2b Case 2: \n',problem_2_case_2)                                        
  pos = get_pos(problem_2_case_2)
  rot = get_rot(problem_2_case_2)
  print_helper(pos, rot)