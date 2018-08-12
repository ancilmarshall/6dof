classdef AccelLoop < handle & IWriter
   
   properties
      
      accelXController;
      accelYController;
      
      dt;
      time;
      
      name = 'accelLoop';
      outputVars = {
         'axRef'
         'ayRef'
         'axCmd'
         'ayCmd'
         'vx'
         'vy'
         'ax'
         'ay'
         'thetaCmd'
         'phiCmd'
         'errorAx'
         'errorAy'
         };
      
      axRef = 0; % open loop guidance reference
      ayRef = 0;
      axCmd = 0; % from outer position loop compensation
      ayCmd = 0;
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
      
      % gains 
%       kp = 0.025;
%       ki = 0.3;
%       kd = 0;
      
      % zero gains so that I only have ff control in the accel loop
      kp = 0;
      ki = 0;
      kd = 0;      
      
                    
      Cx;
      Cy;
      G;
      
      % producer
      guidance;
      
      phase = 0;
      
   end
   
   
   methods
      
      function self = AccelLoop(dt)
         
         self.accelXController = ControllerPID('axLoopPID',dt);
         self.accelYController = ControllerPID('ayLoopPID',dt);
         
         self.accelXController.controlMax =  55*pi/180;
         self.accelXController.controlMin = -55*pi/180;
         self.accelYController.controlMax =  55*pi/180;
         self.accelYController.controlMin = -55*pi/180;
        
         self.accelXController.kp = self.kp;
         self.accelXController.ki = self.ki;
         self.accelXController.kd = self.kd;
         
         self.accelYController.kp = self.kp;
         self.accelYController.ki = self.ki;
         self.accelYController.kd = self.kd;
                  
         self.Cx = getappdata(0,'config_aero_Cx');
         self.Cy = getappdata(0,'config_aero_Cy');
         self.G = getappdata(0,'config_env_G');
                  
         self.dt = dt;
         
         self.time = 0;
         
         self.writer = Writer(self.name, self.outputVars, ...
            @() [ self.time ...
                  self.axRef ...
                  self.ayRef ...
                  self.axCmd ...
                  self.ayCmd ...
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
         self.axCmd = self.guidance.axCmd;
         self.ayCmd = self.guidance.ayCmd;
         self.axRef = self.guidance.axRef;
         self.ayRef = self.guidance.ayRef;
         
         % step composants
         totalAxCmd = self.axRef + self.axCmd;
         totalAyCmd = self.ayRef + self.ayCmd;
         self.errorAx = totalAxCmd - self.ax;
         self.errorAy = totalAyCmd - self.ay;
   
         self.accelXController.error = - self.errorAx; % gain negative for pitch axis
         self.accelYController.error = self.errorAy; 
         
         % feedfoward calculation
         thetaFF = - atan2(totalAxCmd + self.Cx*self.vx, self.G);
         phiFF = atan2(totalAyCmd + self.Cy*self.vy, self.G);
         
         self.accelXController.controlFF = thetaFF;
         self.accelYController.controlFF = phiFF;
   
         guidance_state = getappdata(0,'logic_guidance_state');
         % 0 - user pilot command, 1 - use outer position/accel loop
         
         if (guidance_state >= 1) % TODO: get rid of this 
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