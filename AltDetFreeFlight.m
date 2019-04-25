function [altitude] = AltDetFreeFlight(position, objective,layermat)
%Method 1 of determining a new altitude is based on the distance the UAV 
%has to its destination. Longer distances return a higher altitude.
%A simple even distribution from distances 0 to 10 km is assumed

blanks=ones(size(position,2));
altitude=blanks*layermat(1,1);


end