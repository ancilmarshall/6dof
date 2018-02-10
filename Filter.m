classdef Filter < handle
    
   % n order digital filter implementation
    
   properties
       a
       b
       
       dt
       time
       
       input; 
       output;
       
   end
   
   properties (Access = private)
       order;
       writer;
       name = 'filter';
       outputVars = {...
           'input'
           'output'
       };
   end
   
   methods
      
       function self = Filter(dt)
           
           self.a = getappdata(0,'config_control_a');
           self.b = getappdata(0,'config_control_b');
           
           self.order = length(self.a)-1;
           
           %normalize
           if self.a(1) ~= 1 
               factor = self.a(1);
               self.a = self.a/factor;
               self.b = self.b/factor;
           end
           
           self.dt = dt;
           
           self.input = zeros(self.order+1,1);
           self.output = zeros(self.order+1,1);
           
           self.time = 0;
           
           self.writer = Writer(self.name,self.outputVars);
            
            % write the first set of data. Could argue ghat this can be
            % done upon construction. 
            self.writer.updateTime(self.time);
            outputData = [self.input(self.order+1) self.output(self.order+1) ];
            self.writer.updateData(outputData);
           
       end
       
       function out = step(self)
           
           % input 
           in = getappdata(0,'data_filter_input');
           
           k = self.order+1;
           u = self.input;
           y = self.output;
           
           u(k) = in;
           
           tempb = 0;
           tempa = 0;
           
           for i=1:k
               tempb = tempb + self.b(i)*u(k-i+1);
           end
           
           for i=2:k
               tempa = tempa + self.a(i)*y(k-i+1);
           end
           y(k) = tempb - tempa;
           
           % case for order = 2 -> k=3
           %y(k) = self.b(1)*u(k) + self.b(2)*u(k-1) + self.b(3)*u(k-2) ...
           %                      - self.a(2)*y(k-1) - self.a(3)*y(k-2);
                           
           out = y(k);
           
           % update memory. Update oder important. update oldest first.
           for i=1:k-1
               y(i)=y(i+1);
               u(i)=u(i+1);
           end
           
           %update internal data
           self.input = u;
           self.output = y;           
           self.time = self.time + self.dt;
           
           outputData = [self.input(self.order+1) self.output(self.order+1) ]; 
           self.writer.updateTime(self.time);
           self.writer.updateData(outputData);
           
       end
       
       % always forgetting to add this. Need to make it a default by adding
       % a protocol
       function []=write(self)
           self.writer.write;
       end
       
       function cmd = getCommand(self)
           cmd = self.output(self.order+1);
       end
       
   end
    
    
end