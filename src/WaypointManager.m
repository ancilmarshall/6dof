classdef WaypointManager < handle
   

   properties
      waypoints = {};
      head = 1;
      
   end
   
   methods
      
      function add(self,wpt)
         self.waypoints{end+1} = wpt;
      end
      
      function wpt = next(self)
         if self.head > length(self.waypoints)
            wpt = [];
         else
            wpt = self.waypoints{self.head};
            self.head = self.head + 1;
         end
      end
      
      function array = wptArray(self)
         array = zeros(length(self.waypoints)+1,2);
         for i=1:length(array)-1
            array(i+1,1) = self.waypoints{i}.x;
            array(i+1,2) = self.waypoints{i}.y;
         end
         
      end
      
   end
   
   
   
   
end

