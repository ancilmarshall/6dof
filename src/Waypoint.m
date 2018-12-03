classdef Waypoint < handle
   
   properties
      % ned values
      x = 0;
      y = 0;
      z = 0;
      safe = true;
      index = 0;
      time = 0; % time above which wpt is active
      
      position;
      
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
         elseif nargin == 6
            self.x = varargin{1};
            self.y = varargin{2};
            self.z = varargin{3};
            self.safe = varargin{4};
            self.index = varargin{5};
            self.time = varargin{6};
         end
      end
      
      function setPosition(self,value)
         self.position = [value(1) value(2) value(3)]';
         self.x = value(1);
         self.y = value(2);
         self.z = value(3);
      end
            
      function value = get.position(self)
         value = [self.x self.y self.z]';
      end
      
      % operator overloading
      function value = plus(obj1,obj2)
         value = obj1.position + obj2.position;
      end
      
      % operator overloadin
      function value = minus(obj1,obj2)
         value = obj1.position - obj2.position;
      end
      
      
   end
   
end