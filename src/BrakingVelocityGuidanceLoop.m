classdef BrakingVelocityGuidanceLoop < handle & IWriter
   
   
   properties
      
      dt = 0;
      time = 0;
      
      name = 'guidance';

      % taken from the quick Colibry implementation
      dmax = 4.0; 
      amax = 4.0;
      
      uref_filt = 0.0;
      dist = 0.0;
      distanceToGo = 0.0;
      brakingDistance = 0.0;
      brakingVelocity = 0.0;
      brakingVelocityFilt = 0.0;
      
      %flags
      isVelocityCmdZero = false;
      filter_initialized = false;
      
      %output
      vxCmd = 0.0;
      vyCmd = 0.0;
      
      outputVars = {
         'vxCmd'
         'vyCmd'
         'distanceToGo'
         'brakingDistance'
         'brakingVelocity'
         'brakingVelocityFilt'
      };
                  
   end
   
   methods
      
      function self = BrakingVelocityGuidanceLoop(dt)
         self.dt = dt;
         
         self.writer = Writer(self.name, self.outputVars, ...
            @() [ self.time ...
                  self.vxCmd ...
                  self.vyCmd ...
                  self.distanceToGo ...
                  self.brakingDistance ...
                  self.brakingVelocity ...
                  self.brakingVelocityFilt ]');
      end
      
      
      function setWaypointManager(self,manager)
         self.waypointManager = manager;
         self.nextWpt = self.waypointManager.next;
         self.updateNextPath(self.nextWpt);
      end
      
      function step(self)
         
         %inputs
         currentState = getappdata(0,'data_logic_currentState');
         speedHoriz_x = getappdata(0,'data_rbody_vx');
         speedHoriz_y = getappdata(0,'data_rbody_vy');
         speedHoriz = [speedHoriz_x speedHoriz_y];
         
         dist_to_go = max(self.dmax-self.dist,0.0);
         uref = 0.0;
         
         tau = 0.3;
         a = 1/tau;
         
         if (currentState == ControllerState.GOTOFIX_STATE)
            
            uref = sqrt(max(2*self.amax*(dist_to_go-tau*self.uref_filt),0.0));
            if (norm(speedHoriz) < uref)
               uref = norm(speedHoriz);
            end
            
            if ( abs(uref-0.0) < 1e-9 )
               self.isVelocityCmdZero = true;
            end
            
            if (~self.filter_initialized)
               self.uref_filt = norm(speedHoriz);
               self.filter_initialized = true;
            end
            
            self.uref_filt = (1-a*self.dt)*self.uref_filt + a*self.dt*uref;
            
            self.dist = self.dist + norm(speedHoriz) * self.dt;
            
         else
            self.isVelocityCmdZero = false;
            self.dist = 0.0;
            self.filter_initialized = false;
         end
         
         self.vxCmd = uref;

         self.brakingVelocity = uref;
         self.distanceToGo = dist_to_go;
         self.brakingDistance = self.dist;
         self.brakingVelocityFilt = self.uref_filt;
         
         self.time = self.time + self.dt;
         self.writer.step;
      end
         
      function updateNextPath(self,nextWpt)
         currentPos = [self.x, self.y, 0]';
         self.fromWpt = currentPos; %or can use the prev wpt in the list
         temp = nextWpt - currentPos;
         self.nextPath = temp / norm(temp);         
      end   
      
      
   end
      
   
end