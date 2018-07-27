classdef RBody5D < handle & IWriter
   
    properties

        integrator;
        dt;
        time;
        states = [];     
        
        %producer
        angleCommandProducer;
        windProducer;
        
        vwx = 0;
        vwy = 0;
    end
    
    properties (Access = private )
        name = 'rbody';
        
        wn;
        zeta;
        Cx;
        Cy
        G;
        
        %output
        ax = 0;
        ay = 0;
        
        outputVars = {
            'x'
            'vx'
            'theta'
            'q'
            'y'
            'vy'
            'phi'
            'p'
            'ax'
            'ay'
            };
    end

    methods
        
        function self = RBody5D(states,dt)
            % get config data ( look like our config manager/data manager
            self.wn = getappdata(0,'config_act_wn');
            self.zeta = getappdata(0,'config_act_zeta');
            self.Cx = getappdata(0,'config_aero_Cx');
            self.Cy = getappdata(0,'config_aero_Cy');
            self.G = getappdata(0,'config_env_G');           
                      
            self.time = 0;
            self.states = states;
            self.dt = dt;
            self.updateOutput;
            
            self.integrator = Integrator(self.states,self.dt);
            self.writer = Writer(self.name,self.outputVars, ...
               @() [self.time;self.states;self.ax;self.ay]);
            
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
            theta_cmd = self.angleCommandProducer.thetaCmd;
            phi_cmd = self.angleCommandProducer.phiCmd;
            
            % typically what we program in ode45
            % X-dir nonlinear equations
            xdot(1,1) = vx;
            xdot(2,1) = -self.Cx*(vx-self.vwx) - self.G*tan(theta);
            xdot(3,1) = q;
            xdot(4,1) = -self.wn^2*theta - ...
               2*self.zeta*self.wn*q + self.wn^2*theta_cmd;

            % Y-dir nonlinear equations
            xdot(5,1) = vy;
            xdot(6,1) = - self.Cy*(vy-self.vwy) + self.G*tan(phi);
            xdot(7,1) = p;
            xdot(8,1) = -self.wn^2*phi - ...
               2*self.zeta*self.wn*p + self.wn^2*phi_cmd;

            % integration
            self.integrator.updateDerivatives(xdot);
            [self.time,self.states] = self.integrator.step;
            
            %update calculated variables
            self.updateOutput;
            
            % update the writer
            self.writer.step;
            
%             for i=1:length(self.consumers)
%                 consumer = self.consumers(i);
%                 consumer.updateStates(self.states)
%             end
            
            % output
            self.setOutputData;
            
        end
        
        function updateOutput(self)
           
            vx = self.states(2);
            theta = self.states(3);
            self.ax = -self.Cx*(vx-self.vwx) - self.G*tan(theta);
            
            vy = self.states(6);
            phi = self.states(7);
            self.ay = -self.Cy*(vy-self.vwy) + self.G*tan(phi);           
           
        end
        
        function setOutputData(self)
           
            x = self.states(1);
            vx = self.states(2);
            theta = self.states(3);
            q = self.states(4);
            y = self.states(5);
            vy = self.states(6);
            phi = self.states(7);
            p = self.states(8);

            setappdata(0,'data_rbody_x',x);
            setappdata(0,'data_rbody_y',y);
            setappdata(0,'data_rbody_vx',vx);
            setappdata(0,'data_rbody_vy',vy);
            setappdata(0,'data_rbody_ax',self.ax);
            setappdata(0,'data_rbody_ay',self.ay);
            setappdata(0,'data_rbody_q',q);
            setappdata(0,'data_rbody_p',p);
            setappdata(0,'data_rbody_theta',theta);
            setappdata(0,'data_rbody_phi',phi);           
        end
        
    end
    
end
