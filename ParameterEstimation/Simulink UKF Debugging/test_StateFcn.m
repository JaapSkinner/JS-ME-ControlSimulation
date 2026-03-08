function x_next = test_StateFcn(x, u)
% dummy_StateFcn - A bombproof state transition function for the full 17-state vector.
%
% Its only purpose is to pass the state through without error to isolate
% the measurement function for debugging.

    % We don't need to do any physics here.
    % We just need to return a vector of the correct size.
    x_next = x;

end
