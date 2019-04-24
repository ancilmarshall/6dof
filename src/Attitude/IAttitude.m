classdef (Abstract) IAttitude < handle & IWriter
% Interface class for attitude coordinate systems

   properties
        integrator
        dt
        time
        states = [];
   end
   
   properties (Abstract)
      name
      outputVars
   end
   
   methods (Abstract)
      step(self)
      attitude2euler(self)
      attitude2dcm(self)
      euler2attitude(self)
      dcm2attitude(self)
      
      % provide support for the following
      % add two attitudes
      % subtract two attitudes
   end

end