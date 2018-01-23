function x = InverseFunction(f, y, xmin, xmax)
    epsilon = 0.1;
    if xmax < xmin
        x = 0;
    else
        xmid = (xmin + xmax)/2;
        if f(xmid) > y + epsilon
            x = InverseFunction(f, y, xmin, xmid);
        elseif f(xmid) < y - epsilon
            x = InverseFunction(f, y, xmid, xmax);
        else
            x = xmid;
        end
    end
end