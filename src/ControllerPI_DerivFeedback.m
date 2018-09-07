classdef ControllerPI_DerivFeedback < handle & IWriter
   
   % n order digital filter implementation
   
   properties

      kp = 0;
      ki = 0;
      kd = 0;
      error = 0;
      deriv_error = 0;
      dt = 0;
      time = 0;
      controlFF = 0;
      controlProp = 0;
      controlInt = 0;
      controlDeriv = 0;
      control = 0;
      
      controlMax = Inf;
      controlMin = -Inf;
      
   end
   
   properties (Access = private)
      
      name = 'controller';
      outputVars = {
         'error'
         'controlProp'
         'controlInt'
         'controlDeriv'
         'controlFF'
         'control'
         };
   end
   
   methods
      
      function self = ControllerPI_DerivFeedback(name,dt)
                 
         self.dt = dt;
         self.time = 0;
         self.name = name;
         
         self.writer = Writer(self.name,self.outputVars, ...
            @() [ self.time ...
                  self.error ...
                  self.controlProp ...
                  self.controlInt ...
                  self.controlDeriv ...
                  self.controlFF ...
                  self.control]');
                  
      end
            
      function step(self)
                  
         % PID
         self.controlProp = self.kp * self.error;
         self.controlInt = self.controlInt + self.ki * self.error * self.dt;
         self.controlDeriv = self.kd * self.deriv_error;
         
         self.control = self.controlProp + self.controlDeriv + ... 
            self.controlInt + self.controlFF;
         self.time = self.time + self.dt;
         
         %saturate output
         self.control = min(self.control,self.controlMax);
         self.control = max(self.control,self.controlMin);
                  
         %self.writer.step;
         
      end
      
      function cmd = getCommand(self)
         cmd = self.control;
      end
      
   end
   
   
end