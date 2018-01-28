classdef Writer < handle
    
    % TODO: Update to write out vectors, 3x1 or matricies as cells or nx1 
    properties
       time;
       data;
       dataLabels;
       prefix;
    end
    
    properties (Access = private)
        numdata
    end
    
    methods
        
        % TODO: initial with initial time,data
        function self = Writer(prefix,dataLabels)
            self.prefix = prefix;
            self.dataLabels = dataLabels;
            self.numdata = length(dataLabels);
        end
        
        function updateTime(self,time)
            if isempty(time)
                self.time = time;
            else
                self.time(end+1,1) = time;
            end
        end
        
        function updateData(self,data)
            data = data'; % transpose to form row vector
            if isempty(data)
                self.data = data;
            else
                self.data(end+1,:) = data;
            end
            
        end
        
        function write(self)
            assignin('base',[self.prefix '_time'],self.time);
            for i=1:self.numdata
                label = [self.prefix '_' self.dataLabels{i}];
                assignin('base',label,self.data(:,i));
            end
        end
        
    end
    
end