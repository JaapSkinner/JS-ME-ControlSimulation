NUMLOOPS = 300;
clc;

pool = gcp('nocreate');
if isempty(pool)
    pool = parpool; % start pool
end

q = parallel.pool.DataQueue;
afterEach(q, @(x) fprintf('... Progress: %.0f%% (%d / %d Simulations Run)\n', ...
    (x/NUMLOOPS)*100, x, NUMLOOPS));

futures = parallel.FevalFuture.empty(NUMLOOPS,0);

for i = 1:NUMLOOPS
    futures(i) = parfeval(pool, @simulateWrapper, 0, q, i);
end

wait(futures); % block until all complete
disp('All simulations done.');

function simulateWrapper(q, i)
    warning('off','all');
    evalc('[~] = runSimulationVariedFcn();');
    warning('on','all');
    send(q, i);
end
