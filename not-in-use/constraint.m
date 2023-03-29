% Creates linear inequality in the form of A*x < b
% The constraint corresponds to all points to the left of the line traced
% FROM p1 TO p2
function [A, b] = constraint(p1, p2)

d = p1 - p2;
A = [-d(2) d(1)];
b = d(1)*p2(2) - d(2)*p2(1);

end