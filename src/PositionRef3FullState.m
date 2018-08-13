classdef PositionRef3FullState < handle & IWriter
   
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
      
      wn = 0.5;
      zeta = 0.8;
      
      ulim = .05;
      
      Ka = 15;
      Kd = 30;
      Kp = 1;
      
      %producers
      positionInput;
      
      isActive = false;
      
   end
   
   methods
      
      % constructor
      function self = PositionRef3FullState(dt)
        
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
         
         %calculate the control/acceleration
         self.jx = xin - self.Kp*self.x - self.Kd*self.vx - self.Ka*self.ax;
         self.jy = yin - self.Kp*self.x - self.Kd*self.vy - self.Ka*self.ay;
         
         %integrate and update
         xdot = self.vx;
         vxdot = self.ax;
         axdot = self.jx; % the control input
         self.x = xdot * self.dt + self.x;
         self.vx = vxdot * self.dt + self.vx;
         self.ax = axdot * self.dt + self.ax;
         
         ydot = self.vy;
         vydot = self.ay;
         aydot = self.jy; % the control input
         self.y = ydot * self.dt + self.y;
         self.vy = vydot * self.dt + self.vy;
         self.ay = aydot * self.dt + self.ay;
         
         self.time = self.time + self.dt;
         
         %step writer
         self.writer.step
      
      end
      
      function activate(self)
         
         if self.isActive == false
            self.isActive = true;
            self.time = getappdata(0,'data_rbody_time');
            
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