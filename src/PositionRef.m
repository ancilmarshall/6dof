classdef PositionRef < handle & IWriter
   
   properties
      
      dt;
      time;
      name = 'pref';
      outputVars = {
         'x'
         'v'
         'u'
         };
      
      % state/output along with initial conditions
      x = 0;
      v = 0;
      u = -1;
      
      wn = 1.5;
      zeta = 0.8;
      
      ulim = .05;
      
      Kp;
      Kd;
      
      %producers
      positionInput;
      
   end
   
   methods
      
      % constructor
      function self = PositionRef(dt)
        
         self.dt = dt;
         self.time = 0;
         
         self.Kd = 2*self.zeta*self.wn;
         self.Kp = (self.wn)^2/self.Kd;
         
         self.writer = Writer(self.name,self.outputVars, ...
            @() [ self.time ...
                  self.x ...
                  self.v ...
                  self.u
            ]');
      end
      
      function step(self)
         %inputs
         r = 1;

         %calculate the control
         w = self.Kp*(r-self.x);
         self.u = self.Kd*(w-self.v);
                  
         %integrate and update
         xdot = self.v;
         vdot = self.u;
         self.x = xdot * self.dt + self.x;
         self.v = vdot * self.dt + self.v;
         
         self.time = self.time + self.dt;
         
         %step writer
         self.writer.step
      
      end
      
   end

end