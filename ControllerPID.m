classdef ControllerPID < handle & IWriter
   
   % n order digital filter implementation
   
   properties

      kp = 0;
      ki = 0;
      kd = 0;
      error = 0;
      dt = 0;
      time = 0;
     
   end
   
   properties (Access = private)
      controlProp = 0;
      controlInt = 0;
      controlDeriv = 0;
      controlFF = 0;
      control = 0;
      
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
      
      function self = ControllerPID(dt)
         
         %config
         self.kp = getappdata(0,'config_control_kp');
         self.ki = getappdata(0,'config_control_ki');
         self.kd = getappdata(0,'config_control_kd');
                 
         %try
         self.dt = dt;
         %catch
            %disp('error');
         %end
         self.time = 0;
         
         self.writer = Writer(self.name,self.outputVars, ...
            @() [ self.time ...
                  self.error ...
                  self.controlProp ...
                  self.controlInt ...
                  self.controlDeriv ...
                  self.controlFF ...
                  self.control]');
                  
      end
            
      function out = step(self)
         
         % input
%          self.error = getappdata(0,'data_control_error');
         self.controlFF = getappdata(0,'data_control_controlFF');
         
         % PID
         self.controlProp = self.kp * self.error;
         self.controlInt = self.controlInt + self.ki * self.error * self.dt;
         self.controlDeriv = 0;
         
         self.control = self.controlProp + self.controlDeriv + ... 
            self.controlInt + self.controlFF;
         self.time = self.time + self.dt;
         
         out = self.control;
         
         %self.writer.step;
         
      end
      
      function cmd = getCommand(self)
         cmd = self.control;
      end
      
   end
   
   
end