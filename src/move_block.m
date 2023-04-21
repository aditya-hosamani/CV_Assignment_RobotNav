function res = move_block(blocks, img, projMatrix)
% MOVE_BLOCK Returns the commands to move the specified blocks to their target position.
%
%    Inputs:
%        blocks: a list of string values that determine which blocks you should move 
%                and in which order. For example, if blocks = ["red",
%                "green", "blue"]
%                that means that you need to first move red block. 
%                Your function should at minimum work for the following values of blocks:
%                blocks = {"red"}, blocks = {"green"}, blocks = {"blue"}.
%        img: an image containing the current arrangement of robot and blocks.
%        calib: calibration results from function calibrate.
%
%    Output:
%        res: robot commands separated by ";". 
%             An example output: "go(20); grab(); turn(90);  go(-10); let_go()"
    

    cube_clr = blocks(1);
    coord_set=find_objects(img,cube_clr);
    coord_set_2d = zeros(4,size(coord_set,2));
    Z=25;

    display(projMatrix)
    for i = 1:size(coord_set,2)
        coord_set_2d(:,i) = trans_cord(double(coord_set(:,i)),projMatrix,Z);
    end

    display(coord_set)
    
    cyan = coord_set(:,1);
    mag = coord_set(:,2);
    cube = coord_set(:,4);
    target =coord_set(:,7);
    disp("Stage 1")
    if contains(cyan,cube,coord_set(:,3)) == true %|| %contains(cyan,cube,coord_set(:,5))==true
        disp("Obstacle detected")
    
    else
        disp("No Obstacle detected")
    end
    disp("Stage 2")
    if contains(cube,target,coord_set(:,3)) == true %|| contains(cube,target,coord_set(:,5))==true
        disp("Obstacle detected")
    
    else
        disp("No Obstacle detected")
    end

    %Get Angle of robot
    param_bot = val_calc(cyan,mag,0);
    offset_angle = param_bot(1);
    display(offset_angle)
    %Get Cube
    param_cube = val_calc(cube,cyan,offset_angle);
    %Calculate new position
    offset_angle = offset_angle + param_cube(1);
    param_target = val_calc(target,cube,offset_angle);

    instructions = [];
    instructions = [instructions,turn(param_cube(1)),go(param_cube(2)),grab()];,turn(param_target(1)),go(param_target(2)),let_go()]; 
    
    %instructions = [];
    
    res = join(instructions, "; ");
end

function cont = contains(p_ro,p_targ,p_dist)
    %Disturbance disp("disturbance")
    dist_x=abs(p_ro(1)-p_dist(1));
    dist_y=abs(p_ro(2)-p_dist(2));
    %Referenceframe disp("Reference")
    ref_x = abs(p_ro(1)-p_targ(1));
    ref_y =abs(p_ro(2)-p_targ(2));
    

    if dist_x < ref_x && dist_y < ref_y && sign(p_ro(1)-p_dist(1)) == sign(p_ro(1)-p_targ(1)) && sign(p_ro(2)-p_dist(2)) == sign(p_ro(2)-p_targ(2)) 
        cont = true;
    else
        cont = false;
    end
end


function res = turn(degrees)
    res = sprintf('turn(%g)', degrees);
end

function res = go(dist)
    res = sprintf('go(%g)', (dist-30)/10);
end

function res = grab()
    res = "grab()";
end

function res = let_go()

    res = "let_go()";
end

function V = trans_cord(point,M,Z)
    A = double([M(:,1) M(:,2) -point (Z*M(:,3)+M(:,4))]);
    [~,~,V]=svd(A);
    V=V(:,end);
    V(3) = Z;
    for i =1:size(V,1)
    V(i) = V(i)/V(4);
    end
    V(3) = Z;
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
    elseif dist_x <= 0 && dist_y >= 0
        fprintf("Quad I")
        dist_x = dist_x*(-1);
        %dist_y = dist_y*(-1);
        %angle = 90-atand(dist_x/dist_y);
        angle = atand(dist_x/dist_y);

    %Quad II
    elseif dist_x >= 0 && dist_y >= 0
        fprintf("Quad II")
        %dist_y = dist_y*(-1);
        %angle = (90-atand(dist_y/dist_x))*(-1);
        angle = -atand(dist_x/dist_y);

    %Quad III
    elseif dist_x >= 0 && dist_y <= 0
        fprintf("Quad III")
        dist_y = dist_y*(-1);
        %angle = -90- atand(dist_y/dist_x);
        angle = (180-atand(dist_x/dist_y))*(-1);

    %Quad IV
    elseif dist_x <= 0 && dist_y <= 0
        fprintf("Quad IV")
        dist_y = dist_y*(-1);
        dist_x = dist_x*(-1);
        %angle = 90+atand(dist_y/dist_x)
        angle = 180-atand(dist_x/dist_y);
    end
    angle = angle - offset_angle;
    dist = sqrt(dist_x^2+dist_y^2);
    params = [angle,dist];
    

end

