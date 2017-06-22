function debug_refine_corners(corners,cb_img,rw,a)
    % This will plot the corners and the window around the corners
        
    if ~exist('a','var')
        f = figure(); 
        a = axes(f);
    end
             
    % Plot corners over calibration board image    
    cb_img.imshow(a);
    hold(a,'on');
    plot(corners(:,1),corners(:,2),'go','parent',a);
    
    % Plot boxes around corners
    for i = 1:size(corners,1)     
        width = 2*rw+1;
        x_w = [repmat(corners(i,1)-rw,1,width) corners(i,1)-rw:corners(i,1)+rw repmat(corners(i,1)+rw,1,width) corners(i,1)+rw:-1:corners(i,1)-rw]';
        y_w = [corners(i,2)-rw:corners(i,2)+rw repmat(corners(i,2)+rw,1,width) corners(i,2)+rw:-1:corners(i,2)-rw repmat(corners(i,2)-rw,1,width)]';

        plot(x_w,y_w,'-r','parent',a);
    end    
    
    drawnow;
end
