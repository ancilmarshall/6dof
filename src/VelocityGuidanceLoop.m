classdef VelocityGuidanceLoop < handle & IWriter
   
   
   properties
      
      dt = 0;
      time = 0;
      
      name = 'guidance';
      nextPath = [0 0 0]'; 
      nextWpt = [0 0 0]';
      fromWpt = [0 0 0]';
      waypointManager;
      
      userVxCmd = 2; % not really VxCmd, but thetaCmd
      userVyCmd = 0;

      vxCmd = 0;
      vyCmd = 0;
      
      crossTrackError = 0;
      crossTrackController;
      
      x = 0;
      y = 0;
      
      % config (guess)
      kp = 3;
      ki = 0.5;
      kd = 2;
      
%       kp = 1;
%       ki = 0;
%       kd = 2;     
      
      % consumer
      velocityLoop;
      
      outputVars = {
         'vxCmd'
         'vyCmd'
         'crossTrackError'
      };
                  
   end
   
   methods
      
      function self = VelocityGuidanceLoop(dt)
         self.dt = dt;
         self.time = 0;

         self.crossTrackController = ControllerPID('ctLoopPID',dt);
         self.crossTrackController.kp = self.kp;
         self.crossTrackController.ki = self.ki;
         self.crossTrackController.kd = self.kd;
         
         self.writer = Writer(self.name, self.outputVars, ...
            @() [ self.time ...
                  self.vxCmd ...
                  self.vyCmd ...
                  self.crossTrackError]');
      end
      
      
      function setWaypointManager(self,manager)
         self.waypointManager = manager;
         self.nextWpt = self.waypointManager.next;
         self.updateNextPath(self.nextWpt);
      end
      
      function step(self)
         
         %inputs
         self.x = getappdata(0,'data_rbody_x');
         self.y = getappdata(0,'data_rbody_y');
         
         posDiff = [self.x, self.y, 0]' - self.fromWpt;
         crossTrackErrorVector = cross(self.nextPath,posDiff);
         self.crossTrackError = crossTrackErrorVector(3);
         

         if isempty(self.nextWpt)
            self.vyCmd = 0;
            self.crossTrackError = 0;
         else          
            if ( self.x < self.nextWpt(1))
               self.crossTrackController.error = - self.crossTrackError;
               self.crossTrackController.step;
               self.vyCmd = self.crossTrackController.getCommand;
            else
               self.nextWpt = self.waypointManager.next;
               
               if isempty(self.nextWpt)
                  self.vyCmd = 0;
                  self.crossTrackError = 0;
               else
                  self.updateNextPath(self.nextWpt);
               end
            end
         end
         self.vxCmd = self.userVxCmd;
         self.time = self.time + self.dt;
         self.writer.step;
      end
         
      function updateNextPath(self,nextWpt)
         currentPos = [self.x, self.y, 0]';
         self.fromWpt = currentPos; %or can use the prev wpt in the list
         temp = nextWpt - currentPos;
         self.nextPath = temp / norm(temp);         
      end   
      
      
      % override default implementation of the protocol (interface) class
      function write(self)
         self.crossTrackController.write;
         self.writer.write;
      end
      
   end
      
   
end