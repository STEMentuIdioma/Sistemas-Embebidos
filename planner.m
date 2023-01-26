function [Dt, idx] = planner(mode, h, C, N, t_min, H)
    %%Planificador

    Id = 0;
    IdNext = [];

    ready = logical(zeros(N,1));
    terminated = logical(zeros(N,1));
    standby = logical(zeros(N,1));
    running = logical(zeros(N,1));


    c = zeros(N, 1);
    tau = zeros(N, 1);

    dt = []; %-1*ones(L, 1);
    id = []; %-1*ones(L, 1);

    i = 0;      j = 0;
    for t = 0:t_min:H
%         
        arrived = mod(t, h) == 0;      
        ready = or(arrived, ready); 
        terminated = and(~arrived,terminated); 

        switch mode
            case 'RM'
                Sel = min(h(ready));   %P = [4,5,8];    P(ready) = P([1,1,0]) = [5,4]   Per = min(P(ready)) = [4, 5]
                if(isempty(Sel))
                    IdNext = 0;
                else
                    [Aux, IdNext] = max((h - Sel) == 0);
                end 
            case 'EDF'
                Sel = max(mod(t, h(ready))./h(ready));
                if isempty(Sel)
                    IdNext = 0;
                else
                    [Aux, IdNext] = max((mod(t, h)./h - Sel) == 0); 
                end    
        end

        if t == 0
            standby = [true, true, true]';
        end
        
        j = j+1
        tau(j,1) = 1 + 0.25*standby(1) + 0.5*running(1);
        tau(j,2) = 2 + 0.25*standby(2) + 0.5*running(2);
        tau(j,3) = 3 + 0.25*standby(3) + 0.5*running(3);
      
        if (isempty(IdNext))
            continue;
        elseif (Id(1) ~= IdNext(1))
            if (Id(1) ~= 0)    %
                running(Id(1)) = false;          
                if (terminated(Id(1)))
                    standby(Id(1)) = false;
                else
                    standby(Id(1)) = true;
                end
            end

            if(IdNext(1) ~= 0)
                running(IdNext(1)) = true;        
                standby(IdNext(1)) = false;
                i = i + 1;
                dt(i) = t;
                id(i) = IdNext;
            end            
        end
               
        Id(1) = IdNext(1);

        if(Id(1) ~= 0)
            c(Id(1)) = c(Id(1)) + t_min;



            if (c(Id(1)) == C(Id(1)))
                terminated(Id(1)) = true;
                standby(Id(1)) = false;
                ready(Id(1)) = false;
                running(Id(1)) = false;
                c(Id(1)) = 0;
            end
        end
    end
    
    Dt = dt(2:i)' - dt(1:i-1)';
    idx = id(1:i-1)';
        
    figure(2);
    stairs(dt, id);         hold on;
    stairs(0:t_min:H, tau);  hold off;
end