function highlightCollisionBodies_Arms(robot,collisionBodyIdx, ax)

validateattributes(collisionBodyIdx, {'double'}, {}, 'showCollision', 'collisionBodyIdx');
% if ~isempty(collisionBodyIdx)
%     rigidBodyIdx = collisionBodyIdx-1
% else
    rigidBodyIdx = collisionBodyIdx;
% end

highlightColor = [1 0 0]; % Red


    for i=1:numel(rigidBodyIdx)
       %发生碰撞的关节序号 
       rigidBodyIdx1=rigidBodyIdx;
       
       if  rigidBodyIdx1(i)== 1
        % 机械臂基座
        p = findall(ax, 'type', 'patch', 'displayname', [robot.Base.Name '_mesh']);
       else
        % 除基座外其他关节
        p = findall(ax, 'type', 'patch', 'displayname', [robot.Bodies{rigidBodyIdx1(i)-1}.Name '_mesh']);
       end
    
      if isempty(p)
        continue
      else
        p(1).FaceColor = highlightColor;
      end

     
    end
  end
