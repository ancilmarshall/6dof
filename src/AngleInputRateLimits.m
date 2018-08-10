classdef AngleInputRateLimits < handle & IWriter

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
      
      rateLimitNominal = 50*pi/180; 
      rateLimitCorrection = 50*pi/180;
      thetaCorrection =  20*pi/180;
      vxTransition = 4.5;
      
   end
   
   properties(Access = private)
      d2r = pi/180;
      r2d = 180/pi;      
   end
   
   methods
      
      function self = AngleInputRateLimits(dt)
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
         nextCmd = self.thetaCmd;
         if self.vx < self.vxTransition
            nextCmd = 0;
         end
                  
         % limit the rate
        
         if (self.thetaCmd < self.thetaCorrection)
            ratelimit = self.rateLimitCorrection;
         else
            ratelimit = self.rateLimitNominal;
         end
         
         if ( abs(nextCmd - self.thetaCmd) > ratelimit*self.dt)
            self.thetaCmd = self.thetaCmd - ratelimit*self.dt;
         else
            self.thetaCmd = nextCmd;
         end
         
         self.phiCmd = 0 * self.d2r;
         
         self.time = self.time + self.dt;
         self.writer.step;
      end
      
      
   end

end