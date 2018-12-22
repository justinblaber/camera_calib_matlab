function test_stereo_four_points_detect_LoG
    % Get tests path
    tests_path = fileparts(fileparts(fileparts(mfilename('fullpath'))));

    % Load calibration config
    calib_config = util.load_calib_config(fullfile(tests_path, 'data', 'stereo2', 'stereo2.conf'));

    % Set images
    path_cbs.L = {fullfile(tests_path, 'data', 'stereo2', 'IMG_1_L.JPG')};
    path_cbs.R = {fullfile(tests_path, 'data', 'stereo2', 'IMG_1_R.JPG')};

    % Validate all calibration board images
    img_cbs.L = util.img.validate_similar_imgs(path_cbs.L);
    img_cbs.R = util.img.validate_similar_imgs(path_cbs.R);

    % Set ground truth
    p_fp_p_ss.L{1} = 1.0e+03 * [1.745946902078161   1.339300845902316;
                                1.700332313791803   0.774489507929144;
                                1.126773217917228   1.528082985124175;
                                1.155282383932692   0.983238874363418];
    p_fp_p_ss.R{1} = 1.0e+03 * [2.006125069572509   1.686653095946565;
                                1.940189159967098   0.952832621850176;
                                1.306646622571941   1.686530504572421;
                                1.279088248919651   0.837305546570473];

    p_fp_p_ss_test = alg.stereo_four_points_detect_LoG(img_cbs, calib_config);

    %{
    % Plot example
    f = figure;
    subplot(1, 2, 1);
    img_cbs.L.imshow();
    hold on;
    plot(p_fp_p_ss_test.L{1}(:, 1), p_fp_p_ss_test.L{1}(:, 2), '-rs');
    for j = 1:4
        text(p_fp_p_ss_test.L{1}(j, 1)+20, p_fp_p_ss_test.L{1}(j, 2)+20, num2str(j), 'FontSize', 20, 'Color', 'g');
    end
    subplot(1, 2, 2);
    img_cbs.R.imshow();
    hold on;
    plot(p_fp_p_ss_test.R{1}(:, 1), p_fp_p_ss_test.R{1}(:, 2), '-rs');
    for j = 1:4
        text(p_fp_p_ss_test.R{1}(j, 1)+20, p_fp_p_ss_test.R{1}(j, 2)+20, num2str(j), 'FontSize', 20, 'Color', 'g');
    end
    pause(1)
    close(f);
    %}

    % Assert
    assert(all(all(abs(p_fp_p_ss_test.L{1} - p_fp_p_ss.L{1}) < 1e-4)));
    assert(all(all(abs(p_fp_p_ss_test.R{1} - p_fp_p_ss.R{1}) < 1e-4)));
end
