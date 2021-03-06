classdef PositionRefLQServo < handle & IWriter
   % use the horizontal equations with Cx
   % xdot = v
   % vdot = -Cx * v - g * theta
   % where theta is the input
   % according to MIT Feedback control notes chapter 13
   % 
   % gains based on LQR design
   
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
      
      % error
      ex = 0;
      ey = 0;
      
      errIntx = 0; % additional state
      errInty = 0; % 
         
      Kp = -0.54;
      Kd = -0.4131;
      Ki = 0.3162;
      
      Cx;
      Cy;
      G;
          
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

         self.Cx = getappdata(0,'config_aero_Cx');
         self.Cy = getappdata(0,'config_aero_Cy');
         self.G = getappdata(0,'config_env_G');           

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
         % u = -K x 
         thetaCmd = - self.Kp*self.x - self.Kd*self.vx - self.Ki*self.errIntx;
         phiCmd   = - self.Kp*self.y - self.Kd*self.vy - self.Ki*self.errInty;
         
         %integrate and update
         xdot = self.vx;
         vxdot = -self.Cx*self.vx - self.G*thetaCmd;
         
         self.x = xdot * self.dt + self.x;
         self.vx = vxdot * self.dt + self.vx;
         self.ax = vxdot;
         self.errIntx = self.errIntx + self.ex * self.dt;
         
         ydot = self.vy;
         vydot = -self.Cy*self.vy + self.G*phiCmd;
         
         self.y = ydot * self.dt + self.y;
         self.vy = vydot * self.dt + self.vy;
         self.ay = vydot;
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
                  self.ay
            ]');            
            
            
            % initialize the position ref state
            self.x = getappdata(0,'data_rbody_x');
            self.y = getappdata(0,'data_rbody_y');
            self.vx = getappdata(0,'data_rbody_vx');
            self.vy = getappdata(0,'data_rbody_vy');
%             self.ax = getappdata(0,'data_rbody_ax');
%             self.ay = getappdata(0,'data_rbody_ay');
         end
         
      end         
      
   end

end