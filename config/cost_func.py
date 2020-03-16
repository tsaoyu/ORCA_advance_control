import numpy as np

Q = np.array(  [[1,0,0,0,0,0,0,0,0,0,0,0],          
                [0,1,0,0,0,0,0,0,0,0,0,0],
                [0,0,1,0,0,0,0,0,0,0,0,0],
                [0,0,0,0,0,0,0,0,0,0,0,0], 
                [0,0,0,0,0,0,0,0,0,0,0,0], 
                [0,0,0,0,0,1,0,0,0,0,0,0],
                [0,0,0,0,0,0,0.1,0,0,0,0,0], 
                [0,0,0,0,0,0,0,0.1,0,0,0,0], 
                [0,0,0,0,0,0,0,0,0.1,0,0,0],
                [0,0,0,0,0,0,0,0,0,0,0,0], 
                [0,0,0,0,0,0,0,0,0,0,0.1,0],
                [0,0,0,0,0,0,0,0,0,0,0,0.1]])

           # x y z phi theta psi u v w p q r 


R = np.array([[1,0,0,0],
              [0,1,0,0],
              [0,0,1,0],
              [0,0,0,1]])

x = np.array([1.5, 1, 1,  1,   1,  1,  1,  1 , 1  , 1,   1,   1 ])

def matrix_to_tuple_val(matrix):
    col, row = matrix.shape
    for c in range(col):
        for r in range(row):
            val = matrix[c,r]
            if val != 0:
                print('({},{}) {}'.format(c,r, val))

    print('\n')

def vector_to_tuple_val(vector):
    col = len(vector)
    for c in range(col):
        val = vector[c]
        if val != 0:
            print('({},{}) {}'.format(c, 0, val))

    print('\n')

matrix_to_tuple_val(Q)
matrix_to_tuple_val(R)
vector_to_tuple_val(x)