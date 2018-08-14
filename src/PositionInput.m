classdef PositionInput < handle & IWriter
   
   properties
      
      dt;
      time;
      name = 'positionInput';
      outputVars = {
         'xInput'
         'yInput'
         };
      
      xInput = 0;
      yInput = 0;
      
   end
      
   methods
      
      % constructor
      function self = PositionInput(dt)
        
         self.dt = dt;
         self.time = 0;
         self.writer = Writer(self.name,self.outputVars, ...
            @() [ self.time ...
                  self.xInput ...
                  self.yInput ...
            ]');
      end
      
      function step(self)
         
         %inputs
         
         %step
         
         %update
         self.time = self.time + self.dt;
         
         %step writer
         self.writer.step
      
      end
      
      % event handling protocol
      function activate(self)
         self.time = getappdata(0,'data_rbody_time');
         %self.time = 0;
      end
      
   end
   
   
   
end