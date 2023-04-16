function res = move_block(block, img, projMatrix, camParams)
% MOVE_BLOCK Returns the commands to move the specified blocks to their target position.
%
%    Inputs:
%        blocks: a cell array of string values that determine which blocks you should move 
%                and in which order. For example, if blocks = {"red", "green", "blue"} 
%                that means that you need to first move red block. 
%                Your function should at minimum work for the following values of blocks:
%                blocks = {"red"}, blocks = {"green"}, blocks = {"blue"}.
%        img: an image containing the current arrangement of robot and blocks.
%        calib: calibration results from function calibrate.
%
%    Output:
%        res: robot commands separated by ";". 
%             An example output: "go(20); grab(); turn(90);  go(-10); let_go()"
    

%   coord_set = [robpos;block;target]
    cube_clr = block(1);
    coord_set=find_obj(img,cube_clr);

    %coord_set =[40 20 100; 
    %            30 60 80];
    


    %offset_angle = -21.66;

    bot = coord_set(:,1);
    cyan = coord_set(:,2);
    mag = coord_set(:,3);
    cube = coord_set(:,4);
    target = coord_set(:,5);
    %Get Ange of robot
    param_bot = val_calc(cyan,mag,0);
    offset_angle = param_bot(1);
    %Get Cube
    param_cube = val_calc(cube,bot,offset_angle);
    %Calculate new position
    offset_angle = offset_angle + param_cube(1);
    param_target = val_calc(target,cube,offset_angle);
    
    display(param_cube)
    display(param_target)
    
    res=[];
    %res = [turn(param_cube(1));go(param_cube(2));grab();turn(param_target(1));go(param_target(2));let_go()];
end

function params = val_calc(cord_targ,cord_rob,offset_angle)
    
    dist_x = cord_rob(1)-cord_targ(1);
    dist_y = cord_rob(2)-cord_targ(2);
    angle = 0;

    path_dist = 0;
    %Catching div by 0
    if dist_x == 0
        angle = 0;
    %Quad I
    elseif dist_x <= 0 && dist_y <= 0
        fprintf("Quad I")
        dist_x = dist_x*(-1);
        dist_y = dist_y*(-1);
        angle = 90-atand(dist_y/dist_x);
    
    %Quad II
    elseif dist_x >= 0 && dist_y <= 0
        fprintf("Quad II")
        dist_y = dist_y*(-1);
        angle = (90-atand(dist_y/dist_x))*(-1);
    %Quad III
    elseif dist_x >= 0 && dist_y >= 0
        fprintf("Quad III")
        angle = -90- atand(dist_y/dist_x);
    %Quad IV
    elseif dist_x <= 0 && dist_y >= 0
        fprintf("Quad IV")
        dist_x = dist_x*(-1);
        angle = 90+atand(dist_y/dist_x)
   
    end
    angle = angle - offset_angle;
    dist = sqrt(dist_x^2+dist_y^2);
    params = [angle,dist];
    

end