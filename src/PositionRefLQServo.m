classdef PositionRefLQServo < handle & IWriter
   
   properties
      
      dt;
      time;
      name = 'positionRef';
      outputVars = {
         'x'
         'y'
         'vx'
         'vy'
         'ax'
         'ay'
         'jx'
         'jy'
         };
      
      % state/output along with initial conditions
      x = 0;
      vx = 0;
      ax = 0;
      jx = 0;
      
      y = 0;
      vy = 0;
      ay = 0;
      jy = 0;
      
      % error
      ex = 0;
      ey = 0;
      
      errIntx = 0;
      errInty = 0;
      
      wn = 0.5;
      zeta = 0.8;
      
      ulim = .05;
      
      Kp = 17;
      Kd = 14.3;
      Ka = 6.2;
      Ki = -10;
      
      alpha = 0.2857;
      
      %producers
      positionInput;
      
      isActive = false;
      
   end
   
   methods
      
      % constructor
      function self = PositionRefLQServo(dt)
        
         self.dt = dt;
         self.time = 0;
         
        
%          self.writer = Writer(self.name,self.outputVars, ...
%             @() [ self.time ...
%                   self.x ...
%                   self.y ...
%                   self.vx ...
%                   self.vy ...
%                   self.ax ...
%                   self.ay
%             ]');
      end
      
      function step(self)
         %inputs
         xin = self.positionInput.xInput;
         yin = self.positionInput.yInput;

         %TODO Add external velocity and acceleration inputs
         
         % error
         self.ex = xin - self.x;
         self.ey = yin - self.y;
         
         %calculate the control/acceleration
         
         self.jx = self.alpha*self.Kp*xin ...
                   - self.Kp*self.x - self.Kd*self.vx - self.Ka*self.ax ...
                   - self.Ki*self.errIntx;
         
         self.jy = yin - self.Kp*self.y - self.Kd*self.vy - self.Ka*self.ay;
         
         %integrate and update
         xdot = self.vx;
         vxdot = self.ax;
         axdot = self.jx; % the control input
         self.x = xdot * self.dt + self.x;
         self.vx = vxdot * self.dt + self.vx;
         self.ax = axdot * self.dt + self.ax;
         self.errIntx = self.errIntx + self.ex * self.dt;
         
         ydot = self.vy;
         vydot = self.ay;
         aydot = self.jy; % the control input
         self.y = ydot * self.dt + self.y;
         self.vy = vydot * self.dt + self.vy;
         self.ay = aydot * self.dt + self.ay;
         self.errInty = self.errInty + self.ey * self.dt;
         
         self.time = self.time + self.dt;
         
         %step writer
         self.writer.step
      
      end
      
      function activate(self)
         
         if self.isActive == false
            self.isActive = true;
            self.time = getappdata(0,'data_rbody_time');
            %self.time = 0;
         self.writer = Writer(self.name,self.outputVars, ...
            @() [ self.time ...
                  self.x ...
                  self.y ...
                  self.vx ...
                  self.vy ...
                  self.ax ...
                  self.ay ...
                  self.jx ...
                  self.jy
            ]');            
            
            
            % initialize the position ref state
            self.x = getappdata(0,'data_rbody_x');
            self.y = getappdata(0,'data_rbody_y');
            self.vx = getappdata(0,'data_rbody_vx');
            self.vy = getappdata(0,'data_rbody_vy');
            self.ax = getappdata(0,'data_rbody_ax');
            self.ay = getappdata(0,'data_rbody_ay');
         end
         
      end         
      
   end

end