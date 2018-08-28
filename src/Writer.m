classdef Writer < handle
    
    % TODO: Update to write out vectors, 3x1 or matricies as cells or nx1 
    properties
       time;
       data;
       dataLabels;
       prefix;
    end
    
    properties (Access = private)
        numdata;
        getDataCallback;
    end
    
    methods
        
        function self = Writer(prefix,dataLabels,getDataCallback)
            self.prefix = prefix;
            self.dataLabels = dataLabels;
            self.numdata = length(dataLabels);
            self.getDataCallback = getDataCallback;
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

            if isempty(self.data)
                self.data = data;
            else
                self.data(end+1,:) = data;
            end
        end
        
        function step(self)
           stepData = self.getDataCallback();
           self.updateTime(stepData(1));
           self.updateData(stepData(2:end));
        end       
        
        function write(self)
            assignin('base',[self.prefix '_time'],self.time);
            for i=1:self.numdata
                label = [self.prefix '_' self.dataLabels{i}];
                try
                  assignin('base',label,self.data(:,i));
                catch
                   error(['Error writing ' label]);
                end
            end
        end
        
    end
    
end