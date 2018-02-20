classdef AccelGuidanceLoop < handle & IWriter
   
   
   properties
      
      dt = 0;
      time = 0;
      
      name = 'guidance';
      nextPath = [0 0 0]';
      nextWpt = [0 0 0]';
      fromWpt = [0 0 0]';
      waypointManager;

      userAxCmd = 1; % not really VxCmd, but thetaCmd
      userAyCmd = 0;
      
      axCmd = 0;
      ayCmd = 0;
      
      crossTrackError = 0;
      crossTrackController;
      
      distToGoError = 0;
      distToGoErrorRate = 0;
      distCritical = 0;
      
      isBraking = false;
      
      distToGoController;
      
      x = 0;
      y = 0;
      vx = 0;
      vy = 0;
      
      % config (guess)
%       kp = 2;
%       ki = 0;
%       kd = 4;
      
      % how can this be done in using sisotools (Design)
      kp = 1;
      ki = 0;
      kd = 2;

      % consumer
      accelLoop;
      
      outputVars = {
         'axCmd'
         'ayCmd'
         'crossTrackError'
         'distToGoError'
         'distToGoErrorRate'
         'distCritical'
      };
                  
   end
   
   
   methods
      
      function self = AccelGuidanceLoop(dt)
         self.dt = dt;
         self.time = 0;

         self.crossTrackController = ControllerPID('ctLoopPID',dt);
         self.distToGoController = ControllerPI_DerivFeedback('distToGoPID',dt);
         
         self.crossTrackController.kp = self.kp;
         self.crossTrackController.ki = self.ki;
         self.crossTrackController.kd = self.kd;
         
         self.distToGoController.kp = 1;
         self.distToGoController.ki = 0;
         self.distToGoController.kd = 1.5;
         
         self.crossTrackController.controlMax = 5;
         self.crossTrackController.controlMin = -5;
         
         self.distToGoController.controlMax = 1;
         self.distToGoController.controlMin = -5;
         
         self.writer = Writer(self.name, self.outputVars, ...
            @() [ self.time ...
                  self.axCmd ...
                  self.ayCmd ...
                  self.crossTrackError ...
                  self.distToGoError ...
                  self.distToGoErrorRate ...
                  self.distCritical]');
      end
      
      % put this in a protocol
      function setWaypointManager(self,manager)
         self.waypointManager = manager;
         self.nextWpt = self.waypointManager.next;
         self.updateNextPath(self.nextWpt);
      end
      
      
      function step(self)
         
         if self.time > 2
            self.userAxCmd = 0;
         end
         
         %inputs
         self.x = getappdata(0,'data_rbody_x');
         self.y = getappdata(0,'data_rbody_y');
         self.vx = getappdata(0,'data_rbody_vx');
         self.vy = getappdata(0,'data_rbody_vy');
         
         posDiff = [self.x, self.y, 0]' - self.fromWpt;
         crossTrackErrorVector = cross(self.nextPath,posDiff);
         self.crossTrackError = crossTrackErrorVector(3);
         
         if isempty(self.nextWpt)
            self.ayCmd = 0;
            self.crossTrackError = 0;
         else          
            if ( self.x < self.nextWpt(1))
               self.crossTrackController.error = - self.crossTrackError;
               self.crossTrackController.step;
               self.ayCmd = self.crossTrackController.getCommand;
            else
%                self.nextWpt = self.waypointManager.next;
%                
%                if isempty(self.nextWpt)
%                   self.ayCmd = 0;
%                   self.crossTrackError = 0;
%                else
%                   self.updateNextPath(self.nextWpt);
%                end
                 self.ayCmd = 0;
                 self.crossTrackError = 0;
            end
         end
         
         % activate the x loop for testing a braking command
         % guide along the path
         
       
         distAlongPath = dot(self.nextPath,posDiff);
         pathLength = norm(self.nextWpt-self.fromWpt);
         self.distToGoError = pathLength - distAlongPath;

         vel = [self.vx self.vy 0];
         velocityAlongPath = dot(self.nextPath,vel);

         vCmd = 0;
         
         amax = 2;
         self.distCritical = 2*velocityAlongPath^2/(2*amax);
         
         if abs(self.distToGoError) < self.distCritical
            self.isBraking = true;
         end
         
         if (self.isBraking)
                        
            self.distToGoErrorRate = vCmd - velocityAlongPath;
            
            self.distToGoController.error = self.distToGoError;
            self.distToGoController.deriv_error = self.distToGoErrorRate;
            
            self.distToGoController.step;
            self.axCmd = self.distToGoController.getCommand;
         else
            self.axCmd = self.userAxCmd;
         end
         
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