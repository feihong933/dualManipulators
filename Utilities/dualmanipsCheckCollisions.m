function [isInCollision, collisionPairIdx] = dualmanipsCheckCollisions(tree1,tree2,bodyCollisionArray1, bodyCollisionArray2, config1,config2,isExhaustiveChecking)
% 双机械臂间关节碰撞检测

    % 输入参数定义
    narginchk(7,7)
    validateattributes(tree1, {'robotics.RigidBodyTree'}, {'nonempty'}, 'manipsCollisions', 'tree');
    validateattributes(tree2, {'robotics.RigidBodyTree'}, {'nonempty'}, 'manipsCollisions', 'tree');
    validateattributes(bodyCollisionArray1, {'cell'}, {'nonempty','nrows',tree1.NumBodies, 'ncols', 2}, 'manipsCollisions', 'tree');
    validateattributes(bodyCollisionArray2, {'cell'}, {'nonempty','nrows',tree2.NumBodies, 'ncols', 2}, 'manipsCollisions', 'tree');
    validateattributes(config1, {'double'}, {'nonempty','vector','nrows',length(tree1.homeConfiguration)}, 'manipsCollisions', 'config');
    validateattributes(config2, {'double'}, {'nonempty','vector','nrows',length(tree2.homeConfiguration)}, 'manipsCollisions', 'config');
    validateattributes(isExhaustiveChecking, {'logical'}, {'nonempty','scalar'}, 'manipsCollisions', 'isExhaustiveChecking');

    % 初始化输出参数
    isInCollision = false;
    collisionPairIdx = [];
    
    % 定义数据格式等
    tree1.DataFormat = 'column';
    tree2.DataFormat = 'column';
    robotBodies1 = [{tree1.Base} tree1.Bodies];
    robotBodies2 = [{tree2.Base} tree2.Bodies];
    
    % Rather than calling getTransform at each loop, populate a transform
    % tree, which is a cell array of all body transforms with respect to
    % the base frame
    transformTree1 = cell(numel(robotBodies1)-1,1); 
    transformTree2 = cell(numel(robotBodies2)-1,1); 
    
    
    % For the base, this is the identity
    transformTree1{1} = eye(4);
    for i = 1:numel(robotBodies1)-1
        transformTree1{i} = getTransform(tree1, config1, robotBodies1{i}.Name);
    end
    
     transformTree2{1} = eye(4);
    for i = 1:numel(robotBodies2)-1
        transformTree2{i} = getTransform(tree2, config2, robotBodies2{i}.Name);
    end
    
    % 遍历机械臂1除基座外所有关节，除机械臂末端
    for j = 2:numel(robotBodies1)-1
        %遍历机械臂2除基座外所有关节，除机械臂末端
        for k = 2:numel(robotBodies2)-1
         
                % Ensure that both bodies have associated collision objects
                if ~isempty(bodyCollisionArray1{j,1}) && ~isempty(bodyCollisionArray2{k,1})

                    % Get the collision object pose from the associated
                    % rigid body tree. The updated pose is the product of
                    % the associated rigid body tree pose and transform
                    % that relates the collision object origin to the rigid
                    % body position (measured from the parent joint).
                    bodyCollisionArray1{j,1}.Pose = transformTree1{j}*bodyCollisionArray1{j,2};
                    bodyCollisionArray2{k,1}.Pose = transformTree2{k}*bodyCollisionArray2{k,2};

                    % Check for local collision and update the overall
                    % collision status flag
                    localCollisionStatus = checkCollision(bodyCollisionArray1{j}, bodyCollisionArray2{k});
                    isInCollision = isInCollision || localCollisionStatus;

                    % If a collision is detected, update the matrix of bodies
                    % in collision
                    if localCollisionStatus
                        collisionPairIdx = [collisionPairIdx; [j k]]; %#ok<AGROW>
                        
                        if ~isExhaustiveChecking
                            return;
                        end
                    end
                end        
            
        end
        

    end

end