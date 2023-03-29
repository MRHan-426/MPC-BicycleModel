% Returns the clearance between obst and line and the point o on obst
% which is closest to the line
function [d, os] = clearance(line, obst)

closest_inds = [
    nearest_points(line, obst(1, :)');
    nearest_points(line, obst(2, :)');
    nearest_points(line, obst(3, :)');
    nearest_points(line, obst(4, :)');
];

diffs = line(:, closest_inds) - obst';
dists_2 = diffs(1,:).^2 + diffs(2,:).^2;

[ds, is] = mink(dists_2, 2);
d = min(ds).^0.5;
%o = obst(i,:)';
os = [];
for p = [-0.05, 0.01, 0.15]
    os = [os (1-p)*obst(is,:)' + p*line(:, closest_inds(is))];
end

end

