import numpy as np




def linearPrediction(x, P, A, Q):
    #LINEARPREDICTION calculates mean and covariance of predicted state
    #   density using a liear Gaussian model.

    x = np.matmul(A,x)
    P = np.matmul(np.matmul(A,P),A.T)+Q
    return x,P

def linear_update(x, P, y, H, R):
    #   %LINEARPREDICTION calculates mean and covariance of predicted state
    #   density using a linear Gaussian model.

    # S = H*P*H.transpose()+R # innovation cov
    S = np.matmul(np.matmul(H,P),H.T) + R
    K_k = np.matmul(np.matmul(P,H.T),np.linalg.inv(S)) # Kalman gain
    vk = y - np.matmul(H,x); # innovation 

    x = x+np.matmul(K_k,vk)
    P = P-np.matmul(np.matmul(K_k,S),K_k.T) # numerical stability?
    return x, P

def kalman_filter(y, x, P, A, Q, H, R):

    # prediction step
    x,P = linearPrediction(x, P, A, Q)

    # update step
    x, P = linear_update(x, P, y, H, R)


    return x, P


