function result = solve_attitude(method, body, reference, weights)
%SOLVE_ATTITUDE Body-to-NED TRIAD, Davenport or SVD solution.

for i=1:2; body(i,:)=body(i,:)/norm(body(i,:)); reference(i,:)=reference(i,:)/norm(reference(i,:)); end
weights=weights(:)/sum(weights); name=upper(string(method));
if name=="TRIAD"
    cb=cross(body(1,:),body(2,:)); cr=cross(reference(1,:),reference(2,:));
    if norm(cb)<1e-8 || norm(cr)<1e-8; error('fusion:Attitude','TRIAD vectors are collinear.'); end
    B=[body(1,:)',(cb/norm(cb))',cross(body(1,:),cb/norm(cb))'];
    R=[reference(1,:)',(cr/norm(cr))',cross(reference(1,:),cr/norm(cr))'];
    C=R*B'; canonical='TRIAD';
else
    A=zeros(3); for i=1:2; A=A+weights(i)*reference(i,:)'*body(i,:); end
    if name=="SVD"
        [u,~,v]=svd(A); C=u*diag([1,1,det(u*v')])*v'; canonical='SVD';
    elseif name=="DAVENPORT"
        sigma=trace(A); S=A+A'; z=[A(2,3)-A(3,2);A(3,1)-A(1,3);A(1,2)-A(2,1)];
        K=[sigma,z';z,S-sigma*eye(3)]; [vectors,values]=eig(K); [~,idx]=max(diag(values)); q=vectors(:,idx)';
        candidates=[q;[q(1),-q(2:4)]]; scores=zeros(2,1); matrices=zeros(3,3,2);
        for j=1:2
            matrices(:,:,j)=fusion.math3d('quaternion_to_matrix',candidates(j,:));
            for i=1:2; scores(j)=scores(j)+weights(i)*norm(matrices(:,:,j)*body(i,:)'-reference(i,:)')^2; end
        end
        [~,idx]=min(scores); C=matrices(:,:,idx); canonical='Davenport';
    else
        error('fusion:Attitude','Method must be TRIAD, Davenport or SVD.');
    end
end
C=fusion.math3d('project_rotation',C); q=fusion.math3d('matrix_to_quaternion',C);
gravityError=acosd(max(-1,min(1,dot(C*body(1,:)',reference(1,:)'))));
earthError=acosd(max(-1,min(1,dot(C*body(2,:)',reference(2,:)'))));
result=struct('method',canonical,'c_body_to_ned',C,'quaternion_wxyz_body_to_ned',q, ...
    'quaternion_norm',norm(q),'gravity_error_deg',gravityError,'earth_rate_error_deg',earthError);
end
