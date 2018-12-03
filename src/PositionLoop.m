classdef PositionLoop < handle & IWriter
   
   % This version of the PositionLoop contains the acceleration control
   % TODO - break up into position and accel loop
   % Write Interface classes to define the inputs and the outputs! 
   properties
      
      dt = 0;
      time = 0;
      
      name = 'positionLoop';
      
      % *Cmd are from the successive outer loop for feedback
      % *Ref are the congruent external commands that should minimize need
      % for feedback. Open loop
      xRef = 0;    % Since position is the outermost loop, there is no cmd
      yRef = 0;
      vxCmd = 0;
      vyCmd = 0;
      vxRef = 0;
      vyRef = 0;
      axCmd = 0;
      ayCmd = 0;
      axRef = 0;
      ayRef = 0
               
      x = 0;
      y = 0;
      vx = 0;
      vy = 0;
      ax = 0;
      ay = 0;
      
      thetaCmd = 0;
      phiCmd = 0;
      
      % internal PID controllers
      posXController;
      posYController;
      
      % config (guess)
%       kp = 2;
%       ki = 0;
%       kd = 4;
      
      % how can this be done in using sisotools (Design)
      kp = 1.6;
      ki = 0;
      kd = 3.2;
      
      Cx = 0;
      Cy = 0;
      G = 0;

      % consumer
      accelLoop;
      
      % do this quickly here for now. TODO replace with real accel loop
      accelXController;
      
      %producer
      positionRef;
      
      outputVars = {
         'xRef'
         'yRef'
         'vxRef'
         'vyRef'
         'vxCmd'
         'vyCmd'
         'axRef'
         'ayRef'
         'axCmd'
         'ayCmd'
         'thetaCmd'
         'phiCmd'
      };
                  
   end
   
   
   methods
      
      function self = PositionLoop(dt)
         self.dt = dt;
         self.time = 0;

         self.posXController = ControllerPI_DerivFeedback('posLoopXPID',dt);
         self.posYController = ControllerPI_DerivFeedback('posLoopYPID',dt);
         self.posXController.kp = self.kp;
         self.posYController.kp = self.kp;
         self.posXController.ki = self.ki;
         self.posYController.ki = self.ki;
         self.posXController.kd = self.kd;
         self.posYController.kd = self.kd;  
         
         self.accelXController = ControllerPID('axLoopPID',dt);
         self.accelXController.kp = -0.2;
         self.accelXController.ki = 0;% 0.0;
         self.accelXController.kd = 0;% -0.02;
         
         
         %config
         self.Cx = getappdata(0,'config_aero_Cx');
         self.Cy = getappdata(0,'config_aero_Cy');
         self.G = getappdata(0,'config_env_G');
         
         self.writer = Writer(self.name, self.outputVars, ...
            @() [ self.time ...
                  self.xRef ...
                  self.yRef ...
                  self.vxRef ...
                  self.vyRef ...
                  self.vxCmd ...
                  self.vyCmd ...
                  self.axRef ...
                  self.ayRef ...
                  self.axCmd ...
                  self.ayCmd ...
                  self.thetaCmd ...
                  self.phiCmd]');
      end
      
      
      function step(self)
         
         %inputs
         self.x = getappdata(0,'data_rbody_x');
         self.y = getappdata(0,'data_rbody_y');
         self.vx = getappdata(0,'data_rbody_vx');
         self.vy = getappdata(0,'data_rbody_vy');
         self.ax = getappdata(0,'data_rbody_ax');
         self.ay = getappdata(0,'data_rbody_ay');
         
         self.xRef = self.positionRef.x;
         self.yRef = self.positionRef.y;
         self.vxRef = self.positionRef.vx;
         self.vyRef = self.positionRef.vy;
         self.axRef = self.positionRef.ax;
         self.ayRef = self.positionRef.ay;

         % calculate position loop controller inputs
         % Note. This is PD control with use of the
         % velocity state feedback as the derivative part
         xError = self.xRef - self.x;
         yError = self.yRef - self.y;
         
         xRateError = self.vxRef - self.vx;
         yRateError = self.vyRef - self.vy;
         
         self.posXController.error = xError;
         self.posXController.deriv_error = xRateError;
         
         self.posYController.error = yError;
         self.posYController.deriv_error = yRateError;
            
         self.posXController.step;
         self.posYController.step;
         
         self.axCmd = self.posXController.getCommand;
         self.ayCmd = self.posYController.getCommand;
         
         % the acceleration loop, which is just a FF transform
         % TODO - Use the AccelLoop class instead. Break this up into two
         % Good example here. Any accel loop should take totalAxCmd as
         % input. The feedback portion and the external reference should
         % be handled in the guidance blocks
         totalAxCmd = self.axCmd + self.axRef;
         totalAyCmd = self.ayCmd + self.ayRef;
         
         thetaFF = - atan2(totalAxCmd + self.Cx*self.vx, self.G);
         phiFF = atan2(totalAyCmd + self.Cy*self.vy, self.G); 
         
         self.accelXController.error = totalAxCmd - self.ax; % ax should be changed to horiz frame        
         self.accelXController.controlFF = thetaFF;
         self.accelXController.step;
         
         self.thetaCmd = self.accelXController.getCommand;
         self.phiCmd = phiFF;
         
         self.time = self.time + self.dt;
         self.writer.step;
         
      end
         
      % override default implementation of the protocol (interface) class
      function write(self)
         %self.posXController.write;
         %self.posYController.write;
         self.writer.write;
      end
      
      % event handling protocol. TODO. Make this sim time
      % add an activate flag to the protocol also
      % add a deactivate function also
      function activate(self)
         self.time = getappdata(0,'data_rbody_time');
      end      
      
   end
      
   
end