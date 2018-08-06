classdef AvoidanceConfigClass < Singleton
   
   properties
      tau = 0.2;
      tau0 = 0.2;
      tau1 = 0.2;
      a0max = -4.5;
      a0min = -2;
      v1min = 0.5;
      v1max = 4.0;
      
      %bepop
      Cx = 0.35;
      G = 9.81;
      wn  = 11;
      zeta = 0.9;
      
      
      %anafi
%       Cx = 0.47;
%       G = 9.81;
%       wn  = 11;
%       zeta = 0.9;      
      
      
      Cy;
      
      is_a0max_limited = false;
      a0max_limit = -6.0;
      
   end
   
   methods
      function set.a0max(self,val)
         
         if val > 0
            error('a0max must be negative');
         end
         
         if self.is_a0max_limited %#ok<*MCSUP>
            if val < self.a0max_limit
               self.a0max = self.a0max_limit;
            else
               self.a0max = val;
            end
         else
            self.a0max = val;
         end
      end
   end
   

   
   methods(Access=private)
      % Guard the constructor against external invocation.  We only want
      % to allow a single instance of this class.  See description in
      % Singleton superclass.
      function newObj = AvoidanceConfigClass()
         % Initialise your custom properties.
         newObj.Cy = newObj.Cx;
      end
   end
   
   
   methods(Static)
      % Concrete implementation.  See Singleton superclass.
      function obj = instance()
         persistent uniqueInstance
         if isempty(uniqueInstance)
            obj = AvoidanceConfigClass();
            uniqueInstance = obj;
         else
            obj = uniqueInstance;
         end
      end
   end
   
   
   
   
end