classdef VelocityLoop < handle & IWriter
   
   
   properties
      
      velocityXController;
      velocityYController;
      
      dt;
      time;
      
      name = 'velocityLoop';
      outputVars = {
         'vxRef'
         'vyRef'
         'vx'
         'vy'
         'thetaCmd'
         'phiCmd'
         'errorVx'
         'errorVy'
         };
      
      vxRef = 0;
      vyRef = 0;
      vx = 0;
      vy = 0;
      thetaCmd = 0;
      phiCmd = 0;
      errorVx = 0;
      errorVy = 0;
      
      kp = 0.2612;
      ki = 0.1606;
      kd = 0;
      
      % producer
      guidance;
      
   end
   
   
   methods
      
      function self = VelocityLoop(dt)
         
         self.velocityXController = ControllerPID('vxLoopPID',dt);
         self.velocityYController = ControllerPID('vyLoopPID',dt);
         
         self.velocityXController.kp = self.kp;
         self.velocityXController.ki = self.ki;
         self.velocityXController.kd = self.kd;
         
         self.velocityYController.kp = self.kp;
         self.velocityYController.ki = self.ki;
         self.velocityYController.kd = self.kd;
                  
         self.dt = 0;
         self.time = 0;
         
         self.writer = Writer(self.name, self.outputVars, ...
            @() [ self.time ...
                  self.vxRef ...
                  self.vyRef ...
                  self.vx ...
                  self.vy ...
                  self.thetaCmd ...
                  self.phiCmd ...
                  self.errorVx ...
                  self.errorVy]');
                  
      end
      
      
      function step(self)
         
         % input
         self.vx = getappdata(0,'data_rbody_vx');
         self.vy = getappdata(0,'data_rbody_vy');
         self.vxRef = self.guidance.vxCmd;
         self.vyRef = self.guidance.vyCmd;
         
         % step composants

         self.errorVx = self.vxRef - self.vx;
         self.errorVy = self.vyRef - self.vy;
   
         % for pitch plane, gains are negative due to +theta convention
         self.velocityXController.error = -self.errorVx; 
         self.velocityYController.error = self.errorVy; 
   
         self.velocityXController.step;
         self.velocityYController.step;
         
         self.thetaCmd  = self.velocityXController.getCommand;
         self.phiCmd = self.velocityYController.getCommand;
         
         self.time = self.time + self.dt;
         % output
         
         self.writer.step;
         
      end
      
      % override default implementation of the protocol (interface) class
      function write(self)
         self.velocityXController.write;
         self.velocityYController.write;
         self.writer.write;
      end
      
      
   end
   
   
   
   
end