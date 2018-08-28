classdef PositionRef < handle & IWriter
   
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
         };
      
      % state/output along with initial conditions
      x = 0;
      vx = 0;
      ax = 0;
      
      y = 0;
      vy = 0;
      ay = 0;
      
      wn = 1.5;
      zeta = 0.8;
      
      %ulim = .05;
      ulim = 1000;
      
      Kp;
      Kd;
      
      %producers
      positionInput;
      
      isActive = false;
      
   end
   
   methods
      
      % constructor
      function self = PositionRef(dt)
        
         self.dt = dt;
         self.time = 0;
         
%          self.Kd = 2*self.zeta*self.wn;
%          self.Kp = (self.wn)^2/self.Kd;
         
         self.Kd = 2.6;
         self.Kp = 0;
         
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
         w = self.Kp*(xin-self.x);
         axUnlimited = self.Kd*(w-self.vx);
         axUnlimited = saturate(axUnlimited,-10,10);
         
         w = self.Kp*(yin-self.y);
         self.ay = self.Kd*(w-self.vy);
         
         %integrate and update
         xdot = self.vx;
         vxdot = self.ax;
         self.x = xdot * self.dt + self.x;
         self.vx = vxdot * self.dt + self.vx;
         
         axLimit = self.ulim;
         dax = axUnlimited - self.ax;
         dax = saturate(dax,-axLimit,axLimit);
         
         self.ax = dax + self.ax;
         
         ydot = self.vy;
         vydot = self.ay;
         self.y = ydot * self.dt + self.y;
         self.vy = vydot * self.dt + self.vy;
         
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
                  self.ay
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