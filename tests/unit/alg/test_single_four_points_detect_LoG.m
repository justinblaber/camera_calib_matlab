function test_single_four_points_detect_LoG    
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
    p_fp_p_ss{1} = 1.0e+02 * [1.958093817366980   3.321367973803598;
                              5.047045883488412   3.444087740490788;
                              2.315190657038523   0.865420505042654;
                              4.856066159002345   0.902576404040427];
    p_fp_p_ss{2} = 1.0e+02 * [1.886107656616930   3.600313957272261;
                              4.342373715541079   4.089996366420239;
                              1.998891511786369   1.102973173402852;
                              4.767630817332041   1.400119347161953];
    p_fp_p_ss{3} = 1.0e+02 * [2.754512734382462   3.879126096733291;
                              4.826685658804342   3.652821960654624;
                              2.761711229540450   0.855223708793761;
                              4.745691979350179   1.308546811147698];

    p_fp_p_ss_test = alg.single_four_points_detect_LoG(img_cbs,calib_config);
        
    for i = 1:3        
        %{
        % Plot example
        f = figure;
        img_cbs(i).imshow();
        hold on;
        plot(p_fp_p_ss_test{i}(:,1),p_fp_p_ss_test{i}(:,2),'-rs');
        for j = 1:4
            text(p_fp_p_ss_test{i}(j,1)+20,p_fp_p_ss_test{i}(j,2)+20,num2str(j),'FontSize',20,'Color','g');
        end
        pause(1)
        close(f);
        %}
      
        % Assert
        assert(all(all(abs(p_fp_p_ss_test{i} - p_fp_p_ss{i}) < 1e-4)));
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
    p_fp_p_ss{1} =    1.0e+02 * [2.478560080855610   3.128811574943620;
                                 5.017550646217203   3.222340184937015;
                                 2.578858092193191   0.978713657276637;
                                 4.728057229926550   1.144768037695122];
    p_fp_p_ss{2} = 1.0e+02 * [2.925910339834197   3.801571522076828;
                              5.068529837395690   3.731235271114983;
                              3.029352962698060   0.850811968254622;
                              5.071128818731172   1.364401069831986];
    p_fp_p_ss{3} = 1.0e+02 * [2.571316156472532   3.917643468669751;
                              4.745479876584501   3.726739573724457;
                              2.825616445648377   1.274264610508025;
                              5.224669577204049   1.485639129893151]; 

    p_fp_p_ss_test = alg.single_four_points_detect_LoG(img_cbs,calib_config);
        
    for i = 1:3    
        %{
        % Plot example
        f = figure;
        img_cbs(i).imshow();
        hold on;
        plot(p_fp_p_ss_test{i}(:,1),p_fp_p_ss_test{i}(:,2),'-rs');
        for j = 1:4
            text(p_fp_p_ss_test{i}(j,1)+20,p_fp_p_ss_test{i}(j,2)+20,num2str(j),'FontSize',20,'Color','g');
        end
        pause(1)
        close(f);
        %}
      
        % Assert
        assert(all(all(abs(p_fp_p_ss_test{i} - p_fp_p_ss{i}) < 1e-4)));
    end
    
    clear
end