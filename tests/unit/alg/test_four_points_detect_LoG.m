function test_four_points_detect_LoG    
    % Circle -------------------------------------------------------------%
    % Get tests path
    tests_path = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    
    % Read calibration config
    calib_config = util.read_calib_config(fullfile(tests_path,'data','circle','circle.conf'));

    % Set images
    path_cbs = {fullfile(tests_path,'data','circle','1.jpg'), ...
                fullfile(tests_path,'data','circle','2.jpg'), ...
                fullfile(tests_path,'data','circle','3.jpg')};

    % Validate all calibration board images
    img_cbs = util.img.validate_similar_imgs(path_cbs);

    % Set ground truth    
    p_fp_p_dss{1} = 1.0e+02 * [1.958055038727798   3.321358098633375;
                               5.047005632612745   3.444079477749074;
                               2.315182886159454   0.865311946475428;
                               4.855980869371588   0.902527251122002];   
    p_fp_p_dss{2} = 1.0e+02 * [1.886158327897545   3.600331893195074;
                               4.342474379356603   4.090064427791057;
                               1.998961182066147   1.103012124443165;
                               4.767550698933377   1.400100786798858];  
    p_fp_p_dss{3} = 1.0e+02 * [2.754474608914856   3.879106155129148
                               4.826643807303913   3.652694433609757
                               2.761700344260849   0.855228951498274
                               4.745638799795646   1.308658181439984];  

    for i = 1:3
        array = img_cbs(i).get_array_gs();
        p_fp_ps = alg.four_points_detect_LoG(array,calib_config);
        
        %{
        % Plot example
        f = figure;
        imshow(array,[]);
        hold on;
        plot(p_fp_ps(:,1),p_fp_ps(:,2),'-rs');
        for j = 1:4
            text(p_fp_ps(j,1)+20,p_fp_ps(j,2)+20,num2str(j),'FontSize',20,'Color','g');
        end
        pause(1)
        close(f);
        %}
        
        % Assert
        assert(all(all(abs(p_fp_ps - p_fp_p_dss{i}) < 1e-4)));
    end
    
    clear
                               
    % Checker ------------------------------------------------------------%
    % Get tests path
    tests_path = fileparts(fileparts(fileparts(mfilename('fullpath'))));

    % Read calibration config
    calib_config = util.read_calib_config(fullfile(tests_path,'data','checker','checker.conf'));
    
    % Set images
    path_cbs = {fullfile(tests_path,'data','checker','1.jpg'), ...
                fullfile(tests_path,'data','checker','2.jpg'), ...
                fullfile(tests_path,'data','checker','3.jpg')};

    % Validate all calibration board images
    img_cbs = util.img.validate_similar_imgs(path_cbs);

    % Set ground truth    
    p_fp_p_dss{1} = 1.0e+02 * [2.478639068425884   3.128795746782981;
                               5.017537123754400   3.222303330028511;
                               2.578789422075877   0.978670584423481;
                               4.727973505662725   1.144875977329949];   
    p_fp_p_dss{2} = 1.0e+02 * [2.925883374962980   3.801627433581859;
                               5.068398897055926   3.731215755180295;
                               3.029368762096137   0.850845060513658;
                               5.071038215499118   1.364318906716536];  
    p_fp_p_dss{3} = 1.0e+02 * [2.571279808555108   3.917651365703453;
                               4.745454023986490   3.726693945365171;
                               2.825584033938674   1.274254134539626;
                               5.224642013774388   1.485606887542392];  

    for i = 1:3
        array = img_cbs(i).get_array_gs();
        p_fp_ps = alg.four_points_detect_LoG(array,calib_config);
        
        %{
        % Plot example
        f = figure;
        imshow(array,[]);
        hold on;
        plot(p_fp_ps(:,1),p_fp_ps(:,2),'-rs');
        for j = 1:4
            text(p_fp_ps(j,1)+20,p_fp_ps(j,2)+20,num2str(j),'FontSize',20,'Color','g');
        end
        pause(1)
        close(f);
        %}
        
        % Assert
        assert(all(all(abs(p_fp_ps - p_fp_p_dss{i}) < 1e-4)));
    end
    
    clear
end