function angles_deg = computeEulerFromAxes(R_rel, a1, a2, a3)
% Algemene functie om Euler-hoeken te berekenen uit een rotatiematrix en 3 rotatie-assen
% INPUT:
%   R_rel : 3x3 rotatiematrix (bijv. R_rel_UT Upper arm t.o.v. Thorax)
%   a1    : 3x1 vector, eerste rotatie-as (bv. Yt)
%   a2    : 3x1 vector, tweede rotatie-as (bv. Xh)
%   a3    : 3x1 vector, derde rotatie-as (bv. Yh)
% OUTPUT:
%   angles_deg : 1x3 vector [angle1, angle2, angle3] in graden

% Normaliseer de assen voor de zekerheid as  = as/||as||
a1 = Unity(a1);
a2 = Unity(a2);
a3 = Unity(a3);

% Transformeer standaard basisassen door rotatiematrix
X_rot = R_rel(:,1);
Y_rot = R_rel(:,2);
Z_rot = R_rel(:,3);

% Hoek 1: rotatie rond as a1 (bijv. Yt / plane of elevation)
proj_a3 = a3 - dot(a3, a1) * a1;
proj_Y = Y_rot - dot(Y_rot, a1) * a1;
proj_a3 = proj_a3 / norm(proj_a3);
proj_Y  = proj_Y / norm(proj_Y);

cross1 = cross(proj_a3, proj_Y);
sign1 = sign(dot(cross1, a1));
angle1 = sign1 * acosd(dot(proj_a3, proj_Y)); % cosd = inverse cos in graden

% Hoek 2: rotatie rond as a2 (bijv. Xh / elevation volgens ISB negatief)
angle2 = asind(dot(cross(a1, Y_rot), a2)); 

% Hoek 3: rotatie rond as a3 (bijv. Yh / axial rotation)
proj_a2 = a2 - dot(a2, a1) * a1;
proj_X  = X_rot - dot(X_rot, a1) * a1;
proj_a2 = proj_a2 / norm(proj_a2);
proj_X  = proj_X / norm(proj_X);

cross3 = cross(proj_a2, proj_X);
sign3 = sign(dot(cross3, a1));
angle3 = sign3 * acosd(dot(proj_a2, proj_X));

% Combineer in 1 rijvector zoals gebruikt in de main code
angles_deg = [angle1, angle2, angle3];

end
