function res = move_block_pathfinding(cube_clr, img, projMatrix)
% MOVE_BLOCK Returns the commands to move the specified blocks to their target position.
    coord_set=find_objects(img,cube_clr);
    coord_set_2d = zeros(4,size(coord_set,2));
    Z_robot = 75;
    Z_cube=25;

    display(projMatrix)
    for i = 1:2
        coord_set_2d(:,i) = trans_cord(double(coord_set(:,i)),projMatrix,Z_robot);
    end
    
    for i = 3:size(coord_set,2)
        coord_set_2d(:,i) = trans_cord(double(coord_set(:,i)),projMatrix,Z_cube);
        %coord_set(:,i) = ()
    end

    display(coord_set)
    
    cyan = coord_set_2d(:,1);
    mag = coord_set_2d(:,2);

    cube = [0; 0];

    %cube    = coord_set_2d(:,5);
    %target  = coord_set_2d(:,8);
    %obs1    = coord_set_2d(:,3);
    %obs2    = coord_set_2d(:,4);
    %Cube Selection
    if cube_clr == "red"
        disp("Red")
        cube    = coord_set_2d(:,3);
        target  = coord_set_2d(:,6);
        obs1    = coord_set_2d(:,4);
        obs2    = coord_set_2d(:,5);
    
    elseif cube_clr == "green"
        disp("Green")
        cube    = coord_set_2d(:,4);
        target  = coord_set_2d(:,7);
        obs1    = coord_set_2d(:,3);
        obs2    = coord_set_2d(:,5);

    elseif cube_clr == "blue"
        disp("Blue")
        cube    = coord_set_2d(:,5);
        target  = coord_set_2d(:,8);
        obs1    = coord_set_2d(:,3);
        obs2    = coord_set_2d(:,4);
    end
    
    instructions = [];

    %Get Angle of robot
    param_bot = val_calc(cyan,mag,0);
    offset_angle = param_bot(1);

    
    %Object detection stage 1
    disp("Stage 1")
    if contains(cyan,cube,obs1) == true || contains(cyan,cube,obs2) == true
        disp("Obstacle detected")
        avd = avoid(cyan,obs1,obs2)
        avd_param = val_calc(avd,cyan,offset_angle);
        instructions = [instructions,"Avoid",turn(avd_param(1)),go(avd_param(2)),"Avoid End"];
        offset_angle=offset_angle+avd_param(1);
        param_cube = val_calc(cube,avd,offset_angle);
    else
        disp("No Obstacle detected")
        param_cube = val_calc(cube,cyan,offset_angle);
    end

    %Adjust offset angle
    offset_angle = offset_angle + param_cube(1)
    
    %Parsing instructions
    instructions = [instructions,turn(param_cube(1)),go(param_cube(2)),grab()];
    

    %Object detection stage 1
    disp("Stage 2")
    if contains(cube,target,obs1) == true || contains(cube,target,obs2)==true
        disp("Obstacle detected")      
        avd = avoid(cube,obs1,obs2)
        avd_param = val_calc(avd,cube,offset_angle);
        instructions = [instructions,turn(avd_param(1)),go(avd_param(2))];
        disp("Stage 2 Offset")
        offset_angle=offset_angle+avd_param(1);
        param_target = val_calc(target,avd,offset_angle);
    
    else
        disp("No Obstacle detected")
        param_target = val_calc(target,cube,offset_angle);
    end
    
    gripperlength = 120 - 25;
    %Parsing instructions
    instructions = [instructions,turn(param_target(1)),go(param_target(2)-gripperlength),let_go(),go(-200)];
    %instructions = [turn(param_cube(1));go(param_cube(2));grab();turn(param_target(1));go(param_target(2)-gripperlength);let_go();go(-100)];
    res = join(instructions, "; ")
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
    res = sprintf('go(%g)', dist/10);       %(dist-30)
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

    %point_xy = double([V(1)/V(4); V(2)/V(4)]);
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

function avd = avoid(bot,obs1,obs2)
    obs = [obs1 obs2];
    mx = max(obs(1,:));
    my = max(obs(2,:));
    avd = bot;
    avd(1) = mx+120;
    avd(2) = my+120;
end