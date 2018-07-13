function xdot = tau_eom(~,x)

   global tau input
   xdot = (-x + input)/tau;

end