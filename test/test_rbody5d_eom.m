function xdot = test_rbody5d_eom(~,state)

wn = getappdata(0,'config_act_wn');
zeta = getappdata(0,'config_act_zeta');
Cx = getappdata(0,'config_aero_Cx');
Cy = getappdata(0,'config_aero_Cy');
G = getappdata(0,'config_env_G');
thetaCmd = getappdata(0,'data_guidance_userThetaCmd');
phiCmd = getappdata(0,'data_guidance_userPhiCmd');

            
vx = state(2);
theta = state(3);
q = state(4);

vy = state(6);
phi = state(7);
p = state(8);

xdot(1,1) = vx;
xdot(2,1) = -Cx*vx - G*tan(theta);
xdot(3,1) = q;
xdot(4,1) = -wn^2*theta - 2*zeta*wn*q + wn^2*thetaCmd;

xdot(5,1) = vy;
xdot(6,1) = -Cy*vy - G*tan(phi);
xdot(7,1) = p;
xdot(8,1) = -wn^2*phi - 2*zeta*wn*p + wn^2*phiCmd;


