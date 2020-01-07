function highlightCollisionBodies_Arms(robot,collisionBodyIdx, ax)

validateattributes(collisionBodyIdx, {'double'}, {}, 'showCollision', 'collisionBodyIdx');
% if ~isempty(collisionBodyIdx)
%     rigidBodyIdx = collisionBodyIdx-1
% else
    rigidBodyIdx = collisionBodyIdx;
% end

highlightColor = [1 0 0]; % Red


    for i=1:numel(rigidBodyIdx)
       %������ײ�Ĺؽ���� 
       rigidBodyIdx1=rigidBodyIdx;
       
       if  rigidBodyIdx1(i)== 1
        % ��е�ۻ���
        p = findall(ax, 'type', 'patch', 'displayname', [robot.Base.Name '_mesh']);
       else
        % �������������ؽ�
        p = findall(ax, 'type', 'patch', 'displayname', [robot.Bodies{rigidBodyIdx1(i)-1}.Name '_mesh']);
       end
    
      if isempty(p)
        continue
      else
        p(1).FaceColor = highlightColor;
      end

     
    end
  end
