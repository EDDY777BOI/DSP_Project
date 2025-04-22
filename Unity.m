function u = Unity(v)
    % Normaliseer vector v tot eenheidsvector u
    % Controleer op nulvector om deling door nul te voorkomen
    n = norm(v);
    if n > 0
        u = v / n;
    else
        u = v; 
    end
end