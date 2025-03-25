function V = forceToVolt(a, b, F)
    c = -abs(F);

    V = (-b + sqrt(b^2 - 4*a*c)) / (2*a);

    if F < 0
        V = -V;
    end
end