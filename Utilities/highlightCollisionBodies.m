function highlightCollisionBodies(robot, collisionBodyIdx, ax)


%   Highlight the bodies with indices given in the COLLISIONBODYIDX matrix,
%   where the indices start correspond to the following ordering:
%   [robot.Base robot.Bodies]. The visualization occurs in the axes
%   specified by AX, which must already contain a visualization of the
%   associated rigidbodytree, given by ROBOT.


validateattributes(collisionBodyIdx, {'double'}, {}, 'showCollision', 'collisionBodyIdx');
% if ~isempty(collisionBodyIdx)
%     rigidBodyIdx = collisionBodyIdx-1
% else
    rigidBodyIdx = collisionBodyIdx;
% end

highlightColor = [1 0 0]; % Red

for i = 1:numel(rigidBodyIdx)
    if rigidBodyIdx(i) == 1
        % 机械臂基座
        p = findall(ax, 'type', 'patch', 'displayname', [robot.Base.Name '_mesh']);
    else
        % 除基座外其他关节
        p = findall(ax, 'type', 'patch', 'displayname', [robot.Bodies{rigidBodyIdx(i)-1}.Name '_mesh']);
    end
    
    if isempty(p)
        continue
    else
        p(1).FaceColor = highlightColor;
    end
end
end