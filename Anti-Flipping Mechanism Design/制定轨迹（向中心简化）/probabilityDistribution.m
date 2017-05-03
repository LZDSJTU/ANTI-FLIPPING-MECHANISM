function [ f, Xi ] = probabilityDistribution( d )


[f,Xi] = ksdensity(d','Support','positive');
% f probability distribution
% f * Xi = Probability
f = [0,f];
% Xi interval
Xi = [0,Xi];


end

