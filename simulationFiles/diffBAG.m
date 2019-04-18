function [ dx ] = diffBAG( x )
% Diff operates by taking xk+1-tx for an entire sequence, but returns one
% less value in the sequence, and so its use for integrating another
% parameter over it requires averaging that other parameter over the same
% time steps that diff creates. diffBAG uses diff, but then averages the
% points with the end points having half of the values. In effect, x is
% averaged instead requiring the other parameter being integrated over x to
% be averaged.
% July 17th, 2013 BAG. Intellectual content contributed by Brian Buss.
% October 25th, 2013. BAG. Modified simply to handle the transpose form of
% incoming data as well.

xdiff=diff(x);

[n,m]=size(x);

if(n>m)
    dx = [0;0.5*xdiff]+[0.5*xdiff;0];
else
    dx = [0 0.5*xdiff]+[0.5*xdiff 0];
end

end

