function res = move_block(block, img, calib)
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
    
    res = [];
end