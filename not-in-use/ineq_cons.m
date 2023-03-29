function [Aineq, bineq] = ineq_cons(A,b,npred,nstates,ninputs)

ncons = size(b,1);
zsize = (npred+1)*nstates+npred*ninputs;
xsize = (npred+1)*ncons;

Aineq = zeros(xsize,zsize);
bineq = ones(xsize,1);

state_idxs=nstates+1:nstates:zsize;
con_idxs=(ncons+1):ncons:xsize;

if npred <= 1
    return
end

for i=npred-1:npred
    Aineq(con_idxs(i):con_idxs(i)+ncons-1, state_idxs(i)) = A(:,1);
    Aineq(con_idxs(i):con_idxs(i)+ncons-1, state_idxs(i)+2) = A(:,2);
    bineq(con_idxs(i):con_idxs(i)+ncons-1) = b;
end

end