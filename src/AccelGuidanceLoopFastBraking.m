classdef AccelGuidanceLoopFastBraking < handle & IWriter
   
   
   properties
      
      dt = 0;
      time = 0;
      
      name = 'guidance';
      nextPath = [0 0 0]';
      nextWpt;
      fromWpt = Waypoint(0,0,0);
      waypointManager;

      userAxCmd = 1; % not really VxCmd, but thetaCmd
      userAyCmd = 0;
      
      accelDuration = 7;
      
      axCmd = 0;
      ayCmd = 0;
      
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
         'state'
      };
                  
   end
   
   
   methods
      
      function self = AccelGuidanceLoopFastBraking(dt)
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
         
         ax = getappdata(0,'data_rbody_ax');
         self.distCritical = self.calcCriticalDistance(ax,-4,velocityAlongPath,0.15);
                 
         
         self.distCritical = self.distCritical+1; % add a buffer
         
         if (self.nextWpt.safe == false) &&  (abs(self.distToGoError) < self.distCritical)
            self.isBraking = true; % this can do back to false for safe user command
            self.brakingStartTime = self.time;
         end
         
         if isempty(self.nextWpt)
            self.ayCmd = 0;
            self.crossTrackError = 0;
         else
            if ( self.distToGoError > 0 )
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
            
            % do not allow positive acceleration commands during first few
            % seconds of braking. Dont want vehicle speeding up
%             if self.time < (self.brakingStartTime + 2)
%                self.distToGoController.controlMax = 0;
%             else
%                self.distToGoController.controlMax = 3;
%             end
            
            self.distToGoController.step;
            self.axCmd = -4;%self.distToGoController.getCommand;
            if (velocityAlongPath < 0.8)
               self.axCmd = 0;
            end

%             if self.time < (self.brakingStartTime + 2)
%                self.axCmd = min(self.axCmd,0);
%             end
            
            
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
         self.distToGoController.write;
         self.writer.write;
      end
      
      
      function dist = calcCriticalDistance(~,a1,a0,v1,tau)
         
         % Step 1. Find v0
         v0 = -tau*a0;

         % Step 2. Solve for t0, the minimum time to do the constant braking
         % Implement Newton Method (below)

         veleq = @(t) (v0 - (-tau*(a1-a0)*exp(-t/tau) + a0*t + v1 + tau*(a1-a0)))^2;
         texact = fminsearch(veleq,1.5);

         % Step 3. Calculate total distance travelled during and after braking
         disteq = @(t) tau*tau*(a1-a0)*exp(-t/tau) + 0.5*a0*t.^2 + v1*t + tau*(a1-a0)*t - tau^2*(a1-a0);
         x0 = disteq(texact);
         xf = x0 - tau*tau*a0;

         dist = xf;
         
      end
      
      
   end
      
   
end