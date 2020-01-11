function showCollisionTree(rigidBodyTree, rigidBodyCollisionArray, config)
%ShowCollisionTree Plot collision objects using pose from rigidbodytree configuration

rbt = copy(rigidBodyTree);
rbt.DataFormat = 'column';
rbt.show(config,'PreservePlot',false,'Frames','off');


bodies = [{rbt.Base} rbt.Bodies];
for i = 1:numel(bodies)-1
    % Get tree pose
    TForm = getTransform(rbt, config, bodies{i}.Name, rbt.Base.Name);
    
    % Get collision object information
    collisionObject = rigidBodyCollisionArray{i,1};
    collisionObjectPosition = rigidBodyCollisionArray{i,2};
    
    % Collision object position is a combination of the joint
    % position and the relative pose of the object to the
    % joint.
    if ~isempty(collisionObject)
        collisionObject.Pose = TForm*collisionObjectPosition;
        collisionObject.show('Parent',gca);
        hold on
    end
end

hold off
end

