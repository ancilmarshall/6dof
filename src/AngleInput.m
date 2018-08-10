classdef AngleInput < handle & IWriter

   properties
      
      dt = 0;
      time = 0;
      
      name = 'angleInput';
      outputVars = {
         'thetaCmd'
         'phiCmd'
         };
      
      thetaCmd = 0;
      phiCmd = 0;
      vx = 0;
      vy = 0; 
      ax = 0;
      ay = 0;
      
      vwx = 0;
      vwy = 0;
      tau = 0.15; % estimate given the drone theta dynamics
      Cx = 0.47;
      G = 9.81;
      
   end
   
   properties(Access = private)
      d2r = pi/180;
      r2d = 180/pi;      
   end
   
   methods
      
      function self = AngleInput(dt)
         self.dt = dt;
         self.time = 0;
         
         self.writer = Writer(self.name, self.outputVars, ...
            @() [ self.time ...
                  self.thetaCmd ...
                  self.phiCmd]');         
                  
      end
      
      function step(self)
         
         self.vx = getappdata(0,'data_rbody_vx');
         self.vy = getappdata(0,'data_rbody_vy'); 
         self.ax = getappdata(0,'data_rbody_ax');
         self.ay = getappdata(0,'data_rbody_ay');
         self.vwx = getappdata(0,'env_wind_vwx');
         self.vwy = getappdata(0,'env_wind_vwy');
         
         % angle profile goes here
         
         % -ve for -ve wind
         thetaCmdEq = atan2(self.Cx*self.vwx,self.G);
         transitionVx = abs(self.tau*self.ax); %approximation. tau is also an estimate
         if self.vx < transitionVx
            self.thetaCmd = thetaCmdEq;
         end
                  
         % only for postive wind, need this cap
         if self.thetaCmd < thetaCmdEq;
            self.thetaCmd = thetaCmdEq;
         end
                  
         self.phiCmd = 0 * self.d2r;
         
         self.time = self.time + self.dt;
         self.writer.step;
      end
      
      
   end

end