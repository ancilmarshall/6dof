close all;
%clear all;

t0 = 0;
tf = 3;
dt = 0.005;

% objects
positionRef = PositionRefLQServo(dt);
positionInput = PositionInput(dt);


% producer registration
positionRef.positionInput = positionInput;

% set the guidance command
positionInput.xInput = 1;

% sim
positionInput.activate;
positionRef.activate;

while positionRef.time < tf
   positionInput.step;
   positionRef.step;
end

% write
positionRef.write;
positionInput.write;

figure;
plotg(positionInput_time,positionInput_xInput,...
     positionRef_time,positionRef_x,'r');
  