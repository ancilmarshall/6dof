classdef KF < handle
   
   properties
      Phi  % state transition matrix
      H    % linear measurement/model matrix
      Q    % process noise
      R    % measurement noise
      
      Xest % state estimate
      P    % state covariance matrix
      n    % state size

   end
   
   
   methods
      
      function self = KF(Phi,H,Q,R,Xest,P)
         self.Phi = Phi;
         self.H = H;
         self.Q = Q;
         self.R = R;
         self.Xest = Xest;
         self.P = P;  
         self.n = length(Xest);
      end
      
      % step the filter given a measurement or not
      function [stateEst, covarianceMatrix] = step(self,obs)
         
         newObs = false;
         if nargin==2
            newObs = true;
         end
            
         % propagation step
         Xbar = self.Phi*self.Xest;
         Pbar = self.Phi*self.P*self.Phi' + self.Q;
         
         % update step
         if (newObs)
            S = obs - self.H*Xbar; % innovation
            K = Pbar*self.H'*pinv(self.H*Pbar*self.H' + self.R);
            self.Xest = Xbar + K*S;
            self.P = (eye(self.n) - K*self.H)*Pbar;
         else
            self.Xest = Xbar;
            self.P = Pbar;
         end
         
         % FIXME - should output as a column. Let client take care
         stateEst = self.Xest;
         covarianceMatrix = self.P;
         
      end
   end
   
   
end
