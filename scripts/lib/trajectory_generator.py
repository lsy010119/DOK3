import time
import numpy as np
import scipy.sparse as ssp
import scipy.sparse.linalg as sla



class TrajectoryGenerator:

    def __init__(self, delt):
        self.delt = delt

    def generate(self, x_0, x_des, wp, v_mean):


        dist = np.array([])

        if len(wp) == 0:

            dist = np.append( dist, np.array([np.linalg.norm(x_0[:3]-x_des[:3])]))

            target_time = np.round(dist/v_mean, 1)

            T = target_time[0] # time will vary from 0 to T with step delt

            n = int(target_time[0] / self.delt) # number of timesteps

            # gamma = .05 # damping, 0 is no damping
            gamma = .05 # damping, 0 is no damping

            A = np.zeros((6,6))
            B = np.zeros((6,3))

            A[0,0] = 1
            A[1,1] = 1
            A[2,2] = 1
            A[0,3] = (1-gamma*self.delt/2)*self.delt
            A[1,4] = (1-gamma*self.delt/2)*self.delt
            A[2,5] = (1-gamma*self.delt/2)*self.delt
            A[3,3] = 1 - gamma*self.delt
            A[4,4] = 1 - gamma*self.delt
            A[5,5] = 1 - gamma*self.delt

            B[0,0] = self.delt**2/2
            B[1,1] = self.delt**2/2
            B[2,2] = self.delt**2/2
            B[3,0] = self.delt
            B[4,1] = self.delt   
            B[5,2] = self.delt   

            G = np.zeros((6,3*n))

            for i in range(n):

                G[:, 3*i:3*(i+1)] = np.linalg.matrix_power(A,max(0,n-i-1))@B

            u_hat = sla.lsqr(G,x_des - np.linalg.matrix_power(A,n)@x_0)[0]

            u_vec = u_hat

            u_opt = u_vec.reshape(n,3).T

            x = np.zeros((6,n+1))
            x[:,0] = x_0

            for t in range(n):
                x[:,t+1] = A.dot(x[:,t]) + B.dot(u_opt[:,t])


            return x, np.array([])

        else:

            dist = np.append( dist, np.array([np.linalg.norm(x_0[:3]-wp[:,0])]))

            target_time = np.round(dist/v_mean, 1)

            for i in range(len(wp[0])-1):
                dist = np.append( dist, np.array([np.linalg.norm(wp[:,i]-wp[:,i+1])]))

                target_time = np.append( target_time, 
                                         np.array([target_time[i]+round(dist[i+1]/v_mean, 1)]))
            
            dist = np.append( dist, np.array([np.linalg.norm(x_des[:3]-wp[:,-1])]))

            target_time = np.append( target_time,
                                     np.array([target_time[-1]+round(dist[-1]/v_mean, 1)]))

            K = len(wp[0])
            # target_time : [t_1, t_2, t_3, ... , t_n, t_des]

            T = target_time[-1] # time will vary from 0 to T with step delt

            # delt = 0.1

            n = int(target_time[-1] / self.delt) # number of timesteps


            tk = (target_time / self.delt).astype(int)+1



            gamma = .05 # damping, 0 is no damping

            A = np.zeros((6,6))
            B = np.zeros((6,3))

            A[0,0] = 1
            A[1,1] = 1
            A[2,2] = 1
            A[0,3] = (1-gamma*self.delt/2)*self.delt
            A[1,4] = (1-gamma*self.delt/2)*self.delt
            A[2,5] = (1-gamma*self.delt/2)*self.delt
            A[3,3] = 1 - gamma*self.delt
            A[4,4] = 1 - gamma*self.delt
            A[5,5] = 1 - gamma*self.delt

            B[0,0] = self.delt**2/2
            B[1,1] = self.delt**2/2
            B[2,2] = self.delt**2/2
            B[3,0] = self.delt
            B[4,1] = self.delt   
            B[5,2] = self.delt   


            C = np.array([[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,1,0,0,0]])


            G = np.zeros( (6*n,6) )
            for i in range(n):
                G[6*i:6*(i+1),:] = np.linalg.matrix_power(A,i+1)
            
            H = np.zeros( (6*n,3*n) )
            H_first = np.zeros( (6*n,3) )
            for i in range(n):
                H_first[6*i:6*(i+1),:] = np.linalg.matrix_power(A,i)@B
            for i in range(n):
                H[6*i:,3*i:3*(i+1)] = H_first[:6*(n-i),:]

            S = np.zeros( (3*K+6,6*n))
            for k in range(K):
                S[3*k:3*k+3,6*tk[k]:6*tk[k]+6] = C

            S[-6:,-6:] = np.eye(6)


            wp_and_des = np.hstack((wp.T.flatten(),x_des))


            u_hat = sla.lsqr(S@H,wp_and_des - S@G@x_0)[0]

            u_vec = u_hat   

            u_opt = u_vec.reshape(n,3).T

            x = np.zeros((6,n+1))
            x[:,0] = x_0

            for t in range(n):
                x[:,t+1] = A.dot(x[:,t]) + B.dot(u_opt[:,t])

            return x,tk
