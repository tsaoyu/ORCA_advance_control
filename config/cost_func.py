import numpy as np

Q = np.array(  [[1,   0,   0,   0,   0,   0,     0,     0,     0,   0,     0,   0],             
                [0,   1,   0,   0,   0,   0,     0,     0,     0,   0,     0,   0],   
                [0,   0,   1,   0,   0,   0,     0,     0,     0,   0,     0,   0],   
                [0,   0,   0,   1,   0,    0,     0,     0,     0,   0,     0,   0],    
                [0,   0,   0,   0,   1,    0,     0,     0,     0,   0,     0,   0],    
                [0,   0,   0,   0,   0,    1,      0,     0,     0,   0,     0,   0],   
                [0,   0,   0,   0,   0,    0,    10,     0,     0,   0,     0,   0],    
                [0,   0,   0,   0,   0,    0,     0,    10,     0,   0,     0,   0],    
                [0,   0,   0,   0,   0,    0,     0,     0,    10,   0,     0,   0],   
                [0,   0,   0,   0,   0,    0,     0,     0,     0,  0.1,     0,   0],    
                [0,   0,   0,   0,   0,    0,     0,     0,     0,   0,     0.1,   0],   
                [0,   0,   0,   0,   0,    0,     0,     0,     0,   0,     0,  10]])

           # u v w p q r x y z phi theta psi 


R = np.array([[1,   0,   0,   0],   
              [0,   1,   0,   0],   
              [0,   0,   1,   0],   
              [0,   0,   0,   1]])

x_des = np.array([0,  0,   0,  0,   0,  0,   0,  0,  5 ,  0,   0,   0])
u_des = np.array([0,  0,   0,  0])

Q_final = np.array(  [[1,   0,   0,   0,   0,   0,     0,     0,     0,   0,     0,   0],             
                [0,   1,   0,   0,   0,   0,     0,     0,     0,   0,     0,   0],   
                [0,   0,   1,   0,   0,   0,     0,     0,     0,   0,     0,   0],   
                [0,   0,   0,   1,   0,    0,     0,     0,     0,   0,     0,   0],    
                [0,   0,   0,   0,   1,    0,     0,     0,     0,   0,     0,   0],    
                [0,   0,   0,   0,   0,    1,     0,     0,     0,   0,     0,   0],   
                [0,   0,   0,   0,   0,    0,     1,     0,     0,   0,     0,   0],    
                [0,   0,   0,   0,   0,    0,     0,     1,     0,   0,     0,   0],    
                [0,   0,   0,   0,   0,    0,     0,     0,     1,   0,     0,   0],   
                [0,   0,   0,   0,   0,    0,     0,     0,     0,   1,     0,   0],    
                [0,   0,   0,   0,   0,    0,     0,     0,     0,   0,     1,   0],   
                [0,   0,   0,   0,   0,    0,     0,     0,     0,   0,     0,   1]])

           # u v w p q r x y z phi theta psi 


R_final = np.array([[0,   0,   0,   0],   
              [0,   0,   0,   0],   
              [0,   0,   0,   0],   
              [0,   0,   0,   0]])

x_des_final = np.array([0,  0,   0,  0,   0,  0,  0,  0 , 5  , 0,   0,   0 ])
u_des_final = np.array([0,  0,   0,  0])



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
vector_to_tuple_val(x_des)
vector_to_tuple_val(u_des)


matrix_to_tuple_val(Q_final)
matrix_to_tuple_val(R_final)
vector_to_tuple_val(x_des_final)
vector_to_tuple_val(u_des_final)