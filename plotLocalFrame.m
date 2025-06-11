function plotLocalFrame(origin, R, scale, colors, parentAxes)
% PLOTLOCALFRAME Draw a local coordinate system with quiver3.
%
% origin : 1×3 vector [x y z] of the origin
% R : 3×3 rotation matrix (columns = X, Y, Z unit axes)
% scale : (scalar) length factor for the arrows
% colors : 3×3 matrix with RGB colors per axis, e.g. [1 0 0; 0 1 0; 0 0 1]
% parentAxes: (optional) handle to an existing axes.
% If not given, uses gca.

if nargin < 5
        ax = gca;
    else
        ax = parentAxes;
    end

    % Kolommen uit R: X_as, Y_as, Z_as
    X = R(:,1);
    Y = R(:,2);
    Z = R(:,3);

    hold(ax, 'on');
    % Teken X-as
    quiver3(ax, origin(1), origin(2), origin(3), ...
            scale*X(1), scale*X(2), scale*X(3), ...
            'Color', colors(1,:), 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
    % Teken Y-as
    quiver3(ax, origin(1), origin(2), origin(3), ...
            scale*Y(1), scale*Y(2), scale*Y(3), ...
            'Color', colors(2,:), 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
    % Teken Z-as
    quiver3(ax, origin(1), origin(2), origin(3), ...
            scale*Z(1), scale*Z(2), scale*Z(3), ...
            'Color', colors(3,:), 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
    hold(ax, 'off');
end

