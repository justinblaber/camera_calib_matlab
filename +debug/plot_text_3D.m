function plot_text_3D(s, x_1, y_1, z_1, xform_12, color, font_size, font_weight, a)
    % This will plot text in 3D given an affine transform

    % Matlab's 3D plot is not very good; to get it in the orientation I want,
    % I've just switched the x, y, and z components with:
    %   x => y
    %   y => z
    %   z => x

    if ~exist('a', 'var')
        f = figure();
        a = axes(f);
    end

    p_2 = xform_12 * [x_1 y_1 z_1 1]';
    text(a, p_2(3), p_2(1), p_2(2), s, ...
         'Color', color, 'FontSize', font_size, 'FontWeight', font_weight, ...
         'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
end
