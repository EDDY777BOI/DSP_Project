function A_corrected = fixAttitudeContinuity(A)
% Ensure continuity in a series of attitude matrices.
% For each frame i (starting at 2) and for each column (axis), if the dot product
% with the corresponding column in the previous frame is negative, the column in
% frame i is multiplied by –1.

    [N, ~, ~] = size(A);
    A_corrected = A;  % initialize

    % Loop over frames starting with the second frame.
    for i = 2:N
        for col = 1:3
            % Extract the column vector for frame i and frame i-1.
            vec_current = squeeze(A_corrected(i,:,col));   % 1x3 vector
            vec_previous = squeeze(A_corrected(i-1,:,col));  % 1x3 vector

            % If the dot product is negative, flip the current vector.
            if dot(vec_current, vec_previous) < 0
                A_corrected(i,:,col) = -A_corrected(i,:,col);
            end
        end
    end
end
