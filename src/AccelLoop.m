classdef AccelLoop < handle & IWriter
   
   properties
      
      accelXController;
      accelYController;
      
      dt;
      time;
      
      name = 'accelLoop';
      outputVars = {
         'axRef'
         'axRef'
         'vx'
         'vy'
         'ax'
         'ay'
         'thetaCmd'
         'phiCmd'
         'errorAx'
         'errorAy'
         };
      
      axRef = 0;
      ayRef = 0;
      vx = 0;
      vy = 0;
      ax = 0;
      ay = 0;
      theta = 0;
      phi = 0;
      thetaCmd = 0;
      phiCmd = 0;
      errorAx = 0;
      errorAy = 0;
      
      % gains without feed forward gain
      kp = 0.025;
      ki = 0.3;
      kd = 0;
      
      kff = 0.3; % feed-forward scale factor
      dcgain = 0.89371;
                    
      Cx;
      Cy;
      G;
      
      % producer
      guidance;
      
   end
   
   
   methods
      
      function self = AccelLoop(dt)
         
         self.accelXController = ControllerPID('axLoopPID',dt);
         self.accelYController = ControllerPID('ayLoopPID',dt);
         
         self.accelXController.controlMax =  30*pi/180;
         self.accelXController.controlMin = -30*pi/180;
         self.accelYController.controlMax =  30*pi/180;
         self.accelYController.controlMin = -30*pi/180;
        
         self.accelXController.kp = self.kp;
         self.accelXController.ki = self.ki;
         self.accelXController.kd = self.kd;
         
         self.accelYController.kp = self.kp;
         self.accelYController.ki = self.ki;
         self.accelYController.kd = self.kd;
                  
         self.Cx = getappdata(0,'config_aero_Cx');
         self.Cy = getappdata(0,'config_aero_Cy');
         self.G = getappdata(0,'config_env_G');
                  
         self.dt = 0;
         self.time = 0;
         
         self.writer = Writer(self.name, self.outputVars, ...
            @() [ self.time ...
                  self.axRef ...
                  self.ayRef ...
                  self.vx ...
                  self.vy ...                  
                  self.ax ...
                  self.ay ...
                  self.thetaCmd ...
                  self.phiCmd ...
                  self.errorAx ...
                  self.errorAy]');
                  
      end
      
      
      function step(self)
         
         % input
         self.vx = getappdata(0,'data_rbody_vx');
         self.vy = getappdata(0,'data_rbody_vy');
         self.theta = getappdata(0,'data_rbody_theta');
         self.phi = getappdata(0,'data_rbody_phi');
         self.ax = getappdata(0,'data_rbody_ax');
         self.ay = getappdata(0,'data_rbody_ay');
         
         userThetaCmd = getappdata(0,'data_guidance_userThetaCmd');
         
         % guidance object is an AccelGuidanceLoop object for this consumer
         self.axRef = 1/self.dcgain * self.guidance.axCmd;
         self.ayRef = self.guidance.ayCmd;
         
         % step composants

         self.errorAx = self.axRef - self.ax;
         self.errorAy = self.ayRef - self.ay;
   
         self.accelXController.error = - self.errorAx; % gain negative for pitch axis
         self.accelYController.error = self.errorAy; 
         
         % feedfoward calculation
         thetaFF = - atan2(self.axRef + self.Cx*self.vx, self.G);
         phiFF = atan2(self.ayRef + self.Cy*self.vy, self.G);
          
         % feedfward from colibry (to compare). Works out to same
%          accel = [self.axRef self.ayRef]';
%          C = [self.Cx self.Cy]';
%          v = [self.vx self.vy]';
%          F = accel + C.*v;
%          Fnorm = norm(F);
%          beta = atan2(Fnorm,self.G);
%          thetaFF = -beta*F(1)/Fnorm;
%          phiFF = beta*F(2)/Fnorm;
         
         self.accelXController.controlFF = 0.3*thetaFF;
         self.accelYController.controlFF = 0.3*phiFF;
   
         guidance_state = getappdata(0,'logic_guidance_state');
         
         if (guidance_state == 1)
            self.accelXController.step;
            self.thetaCmd  = self.accelXController.getCommand;
         else
            self.thetaCmd = userThetaCmd;
         end
         
         self.accelYController.step;
         self.phiCmd = self.accelYController.getCommand;         
         
         self.time = self.time + self.dt;
         % output
         
         self.writer.step;
         
      end
      
      % override default implementation of the protocol (interface) class
      function write(self)
         self.accelXController.write;
         self.accelYController.write;
         self.writer.write;
      end
      
      
   end
   
   
   
   
end