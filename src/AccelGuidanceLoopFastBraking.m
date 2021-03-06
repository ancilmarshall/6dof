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
      axRef = 0;
      ayRef = 0;
      
      vxCmd = 0.0;
      vyCmd = 0.0;
      
      crossTrackError = 0;
      crossTrackController;
      
      distToGoError = 0;
      distToGoErrorRate = 0;
      distCritical = 0;
      
      isBraking = false;
      brakingStartTime = 0.0;
      tau = 0.2;
      brakingDecelerationTime = 0.0;
      distToGoController;
      
      state = 0;
      
      x = 0;
      y = 0;
      vx = 0;
      vy = 0;
      
      config = AvoidanceConfigClass(); % TODO: put in the git repo

      
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
         
         self.distToGoController.controlMax = 10;
         self.distToGoController.controlMin = -10;
         
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
         if (self.isBraking == false)
            self.distCritical = self.calcCriticalDistance(ax,velocityAlongPath);
         end
                          
         if (self.nextWpt.safe == false) &&  ...
            (abs(self.distToGoError) < self.distCritical) && ...
            (self.state ~= 2)
            self.isBraking = true; % this can go back to false for safe user command
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
            
            if (velocityAlongPath >= 0.6 )
                % open position loop, closed accel loop
                self.distToGoController.step;
                self.axCmd = self.distToGoController.getCommand;
                self.axRef = -4;
                self.axCmd = 0;
            else
               % activate the outer position loop, keep accel loop closed               
               %self.distToGoController.step;
               self.axCmd = 0; % self.distToGoController.getCommand;
               self.axRef = 0; % zero the open loop accel cmd            
            end
            
            totalBrakingTime = self.tau + self.brakingDecelerationTime;
            if ((self.time - self.brakingStartTime) > totalBrakingTime)
               self.state = 2;
               self.isBraking = 0;
            end
                        
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
      
      
      function dist = calcCriticalDistance(self,a1,v1)
                  
         a0max = self.config.a0max;
         a0min = self.config.a0min;
         v1min = self.config.v1min;
         v1max = self.config.v1max;
         slope = (a0max-a0min)/(v1max-v1min);
         a0eq = @(v1) slope*(v1-v1min) + a0min; % only if v1 is between v1min, v1max
         
         a0 = a0max;
         % Step 0. Find a0
         if v1<v1min
            a0 = a0min;
         elseif v1>v1max
            a0 = a0max;
         else
            a0 = a0eq(v1);
         end
         
         % Step 1. Find v0
         v0 = - self.tau*a0;     
         
         % Step 2. Solve for t0, the minimum time to do the constant braking
         % Implement Newton Method (below)

         veleq = @(t) (v0 - (-self.tau*(a1-a0)*exp(-t/self.tau) + a0*t + v1 + self.tau*(a1-a0)))^2;
         self.brakingDecelerationTime = fminsearch(veleq,1.5);
        

         % Step 3. Calculate total distance travelled during and after braking
         
         totalDistance = @(t,a0,v1) self.tau*self.tau*(a1-a0)*exp(-t/self.tau) + 0.5*a0*t*t + v1*t + ...
                    self.tau*(a1-a0)*t - self.tau*self.tau*(a1-a0) - self.tau*self.tau*a0;
         
         disteq = @(t) self.tau*self.tau*(a1-a0)*exp(-t/self.tau) + 0.5*a0*t.^2 + v1*t + ...
             self.tau*(a1-a0)*t - self.tau^2*(a1-a0);
         x0 = disteq(self.brakingDecelerationTime);
         xf = x0 - self.tau*self.tau*a0;

         dist = totalDistance(self.brakingDecelerationTime,a0,v1);
         
      end
      
      
   end
      
   
end