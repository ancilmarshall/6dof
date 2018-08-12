classdef ClassTemplate < handle & IWriter
   
   properties
      
      dt;
      time;
      name = 'template';
      outputVars = {
         };
      
      %producers
      
   end
      
   methods
      
      % constructor
      function self = ClassTemplate(dt)
        
         self.dt = dt;
         self.time = 0;
         self.writer = Writer(self.name,self.outputVars, ...
            @() [ self.time ...
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
      
   end
   
   
   
end