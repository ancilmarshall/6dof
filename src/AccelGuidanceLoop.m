classdef AccelGuidanceLoop < handle & IWriter
   
   
   properties
      
      dt = 0;
      time = 0;
      
      name = 'guidance';
      nextPath = [0 0 0]';
      nextWpt = Waypoint(0,0,0);
      fromWpt = Waypoint(0,0,0);
      waypointManager;

      userAxCmd = 3;
      userAyCmd = 0;
      
      accelDuration = 1;
      
      axCmd = 0;
      ayCmd = 0;
      axRef = 0;
      ayRef = 0;
      
      crossTrackError = 0;
      crossTrackController;
      
      distToGoError = 0;
      distToGoErrorRate = 0;
      distCritical = 0;
      
      isBraking = false;
      brakingStartTime = 0.0;
      
      distToGoController;
      
      state = 0;
      
      x = 0;
      y = 0;
      vx = 0;
      vy = 0;
      
      % config (guess)
%       kp = 2;
%       ki = 0;
%       kd = 4;
      
      % how can this be done in using sisotools (Design)
      kp = 2;
      ki = 0;
      kd = 3;

      % consumer
      accelLoop;
      
      outputVars = {
         'axCmd'
         'ayCmd'
         'crossTrackError'
         'distToGoError'
         'distToGoErrorRate'
         'distCritical'
         'state'
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
         
         self.distToGoController.kp = 3;
         self.distToGoController.ki = 0;
         self.distToGoController.kd = 2;
         
         self.crossTrackController.controlMax = 5;
         self.crossTrackController.controlMin = -5;
         
         self.distToGoController.controlMax = 5;
         self.distToGoController.controlMin = -5;
         
         self.writer = Writer(self.name, self.outputVars, ...
            @() [ self.time ...
                  self.axCmd ...
                  self.ayCmd ...
                  self.crossTrackError ...
                  self.distToGoError ...
                  self.distToGoErrorRate ...
                  self.distCritical ...
                  self.state]');
      end
      
      % put this in a protocol
      function setWaypointManager(self,manager)
         self.waypointManager = manager;
         self.nextWpt = self.waypointManager.next;
         self.updateNextPath(self.nextWpt);
      end
      
      
      function step(self)
         
         % Fixme - Why is this here?
         if self.time > self.accelDuration
            self.userAxCmd = 0;
         end
         
         %inputs
         self.x = getappdata(0,'data_rbody_x');
         self.y = getappdata(0,'data_rbody_y');
         self.vx = getappdata(0,'data_rbody_vx');
         self.vy = getappdata(0,'data_rbody_vy');
         
         % cross track
         posDiff = [self.x, self.y, 0]' - self.fromWpt.position;
         crossTrackErrorVector = cross(self.nextPath,posDiff);
         self.crossTrackError = crossTrackErrorVector(3);
         
         % distance to go
         distAlongPath = dot(self.nextPath,posDiff);
         pathLength = norm(self.nextWpt-self.fromWpt);
         self.distToGoError = pathLength - distAlongPath;

         vel = [self.vx self.vy 0];
         velocityAlongPath = dot(self.nextPath,vel);
         vCmd = 0;
         amax = 4;
         self.distCritical = velocityAlongPath^2/(2*amax);
         
         self.distCritical = self.distCritical+1; % add a buffer
         
         if (self.nextWpt.safe == false) &&  (abs(self.distToGoError) < self.distCritical)
            self.isBraking = true; % this can do back to false for safe user command
            self.brakingStartTime = self.time;
         end
         
         if isempty(self.nextWpt)
            self.ayCmd = 0;
            self.crossTrackError = 0;
         else
            if self.distToGoError > 0
               self.crossTrackController.error = - self.crossTrackError;
               self.crossTrackController.step;
               self.ayCmd = self.crossTrackController.getCommand;
            else
               
               % only go to next waypoint if the current one is safe
               if self.nextWpt.safe == true
                  
                  self.nextWpt = self.waypointManager.next;
                  
                  if isempty(self.nextWpt)
                     self.ayCmd = 0;
                     self.crossTrackError = 0;
                  else
                     self.updateNextPath(self.nextWpt);
                  end
               end
               
               self.ayCmd = 0;
               self.crossTrackError = 0;
            end
         end
         
         if (self.isBraking)
            self.state = 1;
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
         
         % output
         setappdata(0,'logic_guidance_state',self.state);
      end
         
      function updateNextPath(self,nextWpt)
         currentPos = [self.x, self.y, 0]';
         self.fromWpt.setPosition(currentPos); %or can use the prev wpt in the list
         temp = nextWpt.position - currentPos;
         self.nextPath = temp / norm(temp);         
      end
         
      % override default implementation of the protocol (interface) class
      function write(self)
         self.crossTrackController.write;
%         self.distToGoController.write;
         self.writer.write;
      end
      
   end
      
end