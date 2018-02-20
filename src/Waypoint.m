classdef Waypoint 
   
   properties
      % ned values
      x = 0;
      y = 0;
      z = 0;
      safe = false;
      index = 0;
      time = 0; % time above which wpt is active
   end
   
   methods
      
      function self = Waypoint(varargin)
         if nargin==3
            self.x = varargin{1};
            self.y = varargin{2};
            self.z = varargin{3};
         elseif nargin == 4
            self.x = varargin{1};
            self.y = varargin{2};
            self.z = varargin{3};   
            self.safe = varargin{4};
         elseif nargin == 5
            self.x = varargin{1};
            self.y = varargin{2};
            self.z = varargin{3};
            self.safe = varargin{4};
            self.index = varargin{5};
         end
      end
      
      function pos = getPositionNED(self)
         pos = [self.x self.y self.z];
      end
      
   end
   
end