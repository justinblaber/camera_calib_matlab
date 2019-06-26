function test_fp_detect_LoG
    % Circle -------------------------------------------------------------%
    % Get tests path
    tests_path = fileparts(fileparts(fileparts(mfilename('fullpath'))));

    % Set opts
    opts.blob_detect_LoG_r_range1 = 1;
    opts.blob_detect_LoG_r_range2 = 15;
    opts.blob_detect_LoG_step = 0.5;
    opts.blob_detect_LoG_num_cutoff = 1000;
    opts.blob_detect_LoG_val_cutoff = 0.1;
    opts.blob_detect_LoG_interp = 'cubic';
    opts.blob_detect_LoG_eccentricity_cutoff = 5;
    opts.blob_detect_LoG_lambda = 0.01;
    opts.blob_detect_LoG_maxima_it_cutoff = 10;
    opts.blob_detect_LoG_maxima_norm_cutoff = 1e-6;
    opts.blob_detect_LoG_centroid_it_cutoff = 10;
    opts.blob_detect_LoG_centroid_norm_cutoff = 0.1;
    opts.blob_detect_LoG_d_cluster = 2;
    opts.blob_detect_LoG_r1_cluster = 2;
    opts.blob_detect_LoG_r2_cluster = 2;
    opts.ellipse_detect_num_samples_theta = 100;
    opts.ellipse_detect_interp = 'cubic';
    opts.ellipse_detect_sf_cost = 2;
    opts.ellipse_detect_it_cutoff = 100;
    opts.ellipse_detect_norm_cutoff = 1e-3;
    opts.ellipse_detect_lambda_init = 1;
    opts.ellipse_detect_lambda_factor = 2;
    opts.ellipse_detect_d_cluster = 2;
    opts.ellipse_detect_r1_cluster = 2;
    opts.ellipse_detect_r2_cluster = 2;
    opts.fp_detect_marker_templates_path = '+markers/marker_templates.txt';
    opts.fp_detect_marker_config_path = '+markers/marker.conf';
    opts.fp_detect_num_cutoff = 20;
    opts.fp_detect_mse_cutoff = 0.2;
    opts.fp_detect_padding_radial = 5;

    % Set images
    path_cbs = {fullfile(tests_path, 'data', 'circle', '1.jpg'), ...
                fullfile(tests_path, 'data', 'circle', '2.jpg'), ...
                fullfile(tests_path, 'data', 'circle', '3.jpg')};

    % Validate all calibration board images
    img_cbs = class.img.path.validate_similar_imgs(path_cbs);

    % Set ground truth
    p_fp_p_ds{1} = 1.0e+02 * [1.958055038727798   3.321358098633375;
                              5.047005632612745   3.444079477749074;
                              2.315182886159454   0.865311946475428;
                              4.855980869371588   0.902527251122002];
    p_fp_p_ds{2} = 1.0e+02 * [1.886158327897545   3.600331893195074;
                              4.342474379356603   4.090064427791057;
                              1.998961182066147   1.103012124443165;
                              4.767550698933377   1.400100786798858];
    p_fp_p_ds{3} = 1.0e+02 * [2.754474608914856   3.879106155129148
                              4.826643807303913   3.652694433609757
                              2.761700344260849   0.855228951498274
                              4.745638799795646   1.308658181439984];

    for i = 1:3
        array = img_cbs(i).get_array_gs();
        p_fp_ps = alg.fp_detect_LoG(array, opts);

        %{
        % Plot example
        f = figure;
        imshow(array, []);
        hold on;
        plot(p_fp_ps(:, 1), p_fp_ps(:, 2), '-rs');
        for j = 1:4
            text(p_fp_ps(j, 1)+20, p_fp_ps(j, 2)+20, num2str(j), 'FontSize', 20, 'Color', 'g');
        end
        pause(1)
        close(f);
        %}

        % Assert
        assert(all(all(abs(p_fp_ps - p_fp_p_ds{i}) < 1e-4)));
    end

    clear

    % Checker ------------------------------------------------------------%
    % Get tests path
    tests_path = fileparts(fileparts(fileparts(mfilename('fullpath'))));

    % Set opts
    opts.blob_detect_LoG_r_range1 = 1;
    opts.blob_detect_LoG_r_range2 = 15;
    opts.blob_detect_LoG_step = 0.5;
    opts.blob_detect_LoG_num_cutoff = 1000;
    opts.blob_detect_LoG_val_cutoff = 0.1;
    opts.blob_detect_LoG_interp = 'cubic';
    opts.blob_detect_LoG_eccentricity_cutoff = 5;
    opts.blob_detect_LoG_lambda = 0.01;
    opts.blob_detect_LoG_maxima_it_cutoff = 10;
    opts.blob_detect_LoG_maxima_norm_cutoff = 1e-6;
    opts.blob_detect_LoG_centroid_it_cutoff = 10;
    opts.blob_detect_LoG_centroid_norm_cutoff = 0.1;
    opts.blob_detect_LoG_d_cluster = 2;
    opts.blob_detect_LoG_r1_cluster = 2;
    opts.blob_detect_LoG_r2_cluster = 2;
    opts.ellipse_detect_num_samples_theta = 100;
    opts.ellipse_detect_interp = 'cubic';
    opts.ellipse_detect_sf_cost = 2;
    opts.ellipse_detect_it_cutoff = 100;
    opts.ellipse_detect_norm_cutoff = 1e-3;
    opts.ellipse_detect_lambda_init = 1;
    opts.ellipse_detect_lambda_factor = 2;
    opts.ellipse_detect_d_cluster = 2;
    opts.ellipse_detect_r1_cluster = 2;
    opts.ellipse_detect_r2_cluster = 2;
    opts.fp_detect_marker_templates_path = '+markers/marker_templates.txt';
    opts.fp_detect_marker_config_path = '+markers/marker.conf';
    opts.fp_detect_num_cutoff = 20;
    opts.fp_detect_mse_cutoff = 0.2;
    opts.fp_detect_padding_radial = 5;

    % Set images
    path_cbs = {fullfile(tests_path, 'data', 'checker', '1.jpg'), ...
                fullfile(tests_path, 'data', 'checker', '2.jpg'), ...
                fullfile(tests_path, 'data', 'checker', '3.jpg')};

    % Validate all calibration board images
    img_cbs = class.img.path.validate_similar_imgs(path_cbs);

    % Set ground truth
    p_fp_p_ds{1} = 1.0e+02 * [2.478639068425884   3.128795746782981;
                              5.017537123754400   3.222303330028511;
                              2.578789422075877   0.978670584423481;
                              4.727973505662725   1.144875977329949];
    p_fp_p_ds{2} = 1.0e+02 * [2.925883374962980   3.801627433581859;
                              5.068398897055926   3.731215755180295;
                              3.029368762096137   0.850845060513658;
                              5.071038215499118   1.364318906716536];
    p_fp_p_ds{3} = 1.0e+02 * [2.571279808555108   3.917651365703453;
                              4.745454023986490   3.726693945365171;
                              2.825584033938674   1.274254134539626;
                              5.224642013774388   1.485606887542392];

    for i = 1:3
        array = img_cbs(i).get_array_gs();
        p_fp_ps = alg.fp_detect_LoG(array, opts);

        %{
        % Plot example
        f = figure;
        imshow(array, []);
        hold on;
        plot(p_fp_ps(:, 1), p_fp_ps(:, 2), '-rs');
        for j = 1:4
            text(p_fp_ps(j, 1)+20, p_fp_ps(j, 2)+20, num2str(j), 'FontSize', 20, 'Color', 'g');
        end
        pause(1)
        close(f);
        %}

        % Assert
        assert(all(all(abs(p_fp_ps - p_fp_p_ds{i}) < 1e-4)));
    end
end
