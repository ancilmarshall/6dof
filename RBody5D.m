classdef RBody5D < handle & IWriter
   
    properties

        integrator;
        dt;
        time;
        states = [];      

    end
    
    properties (Access = private )
        name = 'rbody';
        
        wn;
        zeta;
        Cx;
        Cy
        G;
        
        outputVars = {
            'x'
            'vx'
            'theta'
            'q'
            'y'
            'vy'
            'phi'
            'p'
            };
    end

    methods
        
        function self = RBody5D(states,dt)
            self.time = 0;
            self.states = states;
            self.dt = dt;
            
            self.integrator = Integrator(self.states,self.dt);
            self.writer = Writer(self.name,self.outputVars, ...
               @() [self.time;self.states]);
            
            % get config data
            self.wn = getappdata(0,'config_act_wn');
            self.zeta = getappdata(0,'config_act_zeta');
            self.Cx = getappdata(0,'config_aero_Cx');
            self.Cy = getappdata(0,'config_aero_Cy');
            self.G = getappdata(0,'config_env_G');
            
        end
        
        function registerIntegrator(self,integrator)
            self.integrator = integrator;
        end
        
        function step(self)

            x = self.states(1);
            vx = self.states(2);
            theta = self.states(3);
            q = self.states(4);
            
            y = self.states(5);
            vy = self.states(6);
            phi = self.states(7);
            p = self.states(8);

            % control input
            theta_cmd = getappdata(0,'data_control_thetaCmd');
            phi_cmd = getappdata(0,'data_control_phiCmd');
            
            % X-dir nonlinear equations
            xdot(1,1) = vx;
            xdot(2,1) = -self.Cx*vx - self.G*tan(theta);
            xdot(3,1) = q;
            xdot(4,1) = -self.wn^2*theta - ...
               2*self.zeta*self.wn*q + self.wn^2*theta_cmd;
            
            % Y-dir nonlinear equations
            xdot(5,1) = vy;
            xdot(6,1) = - self.Cy*vy - self.G*tan(phi);
            xdot(7,1) = p;
            xdot(8,1) = -self.wn^2*phi - ...
               2*self.zeta*self.wn*p + self.wn^2*phi_cmd;

            % integration
            self.integrator.updateDerivatives(xdot);
            [self.time,self.states] = self.integrator.step;
            
            %update the writer
            self.writer.step;
            
%             for i=1:length(self.consumers)
%                 consumer = self.consumers(i);
%                 consumer.updateStates(self.states)
%             end
            
            %%% output
            
        end
        
    end
    
end
