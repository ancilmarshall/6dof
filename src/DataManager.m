classdef DataManager < Singleton & Dictionary
   
 
   methods(Access=private)
      % Guard the constructor against external invocation.  We only want
      % to allow a single instance of this class.  See description in
      % Singleton superclass.
      function newObj = DataManager()
         % Initialise your custom properties.
      end
   end
   
   
   methods(Static)
      % Concrete implementation.  See Singleton superclass.
      function obj = instance()
         persistent uniqueInstance
         if isempty(uniqueInstance)
            obj = DataManager();
            uniqueInstance = obj;
         else
            obj = uniqueInstance;
         end
      end
   end
   
end