classdef IWriter < handle
   
   properties
        writer; % writer saves time history of states in arrays so that
                % this object does not have to. The writer class can be 
                % generic and used by several objects
   end
   
   methods
       function write(self)
           self.writer.write;
       end
      
   end
   
   
   
end