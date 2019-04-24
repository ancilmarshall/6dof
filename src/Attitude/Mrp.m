classdef Mrp < IAttitude
% Interface class for attitude coordinate systems
   
   properties
      
      sigma
      
      name = 'mrp';
      outputVars = {
         'sigma1'
         'sigma2'
         'sigma3'
         };
   end
   
   methods
      function step(self)
         
      end
      
      function angles = attitude2euler(self)
      end
      
      function dcm = attitude2dcm(self)
         s2 = norm(self.sigma)^2;
         den = (1+s2)^2;
         sigma_tilde = skewmat(self.sigma);
         dcm = eye(3) + (8*sigma_tilde^2 - 4*(1-s2)*sigma_tilde)/den;
      end
      
      function out = euler2attitude(self, angles)
         
      end
      
      function out = dcm2attitude(self, C)
      
      end
   end

end