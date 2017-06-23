function debug_refine_points(points,cb_img,rw,a)
    % This will plot the points and the window around the points
        
    if ~exist('a','var')
        f = figure(); 
        a = axes(f);
    end
             
    % Plot points over calibration board image    
    cb_img.imshow(a);
    hold(a,'on');
    plot(points(:,1),points(:,2),'go','parent',a);
    
    % Plot boxes around points   
    width = 2*rw+1;
    for i = 1:size(points,1)  
        x_w = [repmat(points(i,1)-rw,1,width) points(i,1)-rw:points(i,1)+rw repmat(points(i,1)+rw,1,width) points(i,1)+rw:-1:points(i,1)-rw]';
        y_w = [points(i,2)-rw:points(i,2)+rw repmat(points(i,2)+rw,1,width) points(i,2)+rw:-1:points(i,2)-rw repmat(points(i,2)-rw,1,width)]';

        plot(x_w,y_w,'-r','parent',a);
    end    
    
    drawnow;
end
