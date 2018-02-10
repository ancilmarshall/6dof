classdef Nav < IConsumer
    
    properties
        alpha = 0;
        beta = 0;
        psi = 0;
        theta = 0;
        phi = 0;
        C_toBFromN = zeros(9,1);
        
        time; %temporary. TODO: way to write for object with no integrator
              % so a static (not dynamic model) % need simtime
    end
    
    properties
        dt;
        writer;
        prefix = 'nav';
        outputLabels = {
            'alpha'
            'beta'
            'psi'
            'theta'
            'phi'
            %'C_toBFromN'
            };
       
        output = [];
    end
    
    methods
        
        % Constructor
        function self = Nav(dt)
            self.time = 0;
            self.dt = dt;
            self.writer = Writer(self.prefix,self.outputLabels,...
               @()[self.time;self.output]);
        end
        
        function updateOutput(self)
            self.output = [
                self.alpha
                self.beta
                self.psi
                self.theta
                self.phi
                %self.C_toBFromN
                ];
        end
        
        function updateTime(self,time)
            self.time = time;
        end
        
        % or maybe update data or maybe a binding function
        function updateStates(self,states)
            self.states = states;
        end
        
        function step(self)
            v = self.states(1:3);
            %omega = self.states(4:6);
            q = self.states(7:10);
            
            q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);
            
            self.alpha = atan2(v(3),v(1));
            self.beta = asin( v(2) / norm(v));
            
            self.psi = atan2(2*(q1*q2 + q0*q3),(q0^2 + q1^2 - q2^2 - q3^2));
            self.theta = asin(-2*(q1*q3 - q0*q2));
            self.phi = atan2(2*(q2*q3 + q0*q1),(q0^2 - q1^2 - q2^2 + q3^2));
            
            E = eye(3);
            qvec = [q1;q2;q3];
            Q = [0 -q3 q2 ; q3 0 -q1 ; -q2 q1 0 ];
            C = q0^2*E - qvec'*qvec*E + 2*(qvec*qvec') + 2*q0*Q;
            self.C_toBFromN = [
                C(1,1)
                C(1,2)
                C(1,3)
                C(2,1)
                C(2,2)
                C(2,3)
                C(3,1)
                C(3,2)
                C(3,3)
                ];
                
            %update
            self.time = self.time + self.dt;
            self.updateOutput();
            
            %update the writer
            self.writer.step;
                        
        end
        
        function write(self)
            self.writer.write();
        end
        
    end
    
end