function [i, j] = nearest_points(line, p)

diffs = line - p;
dists_2 = diffs(1,:).^2 + diffs(2,:).^2;
[~, inds] = mink(dists_2, 2);

i = inds(1);
j = inds(2);

if j < i
    [i, j] = deal(j, i);
end

end