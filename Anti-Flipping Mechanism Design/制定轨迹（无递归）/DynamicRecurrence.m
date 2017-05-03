function [ Fjoint1, Mjoint1,w ] = DynamicRecurrence( qd, qdd, w0, w0d, p0dd, f_end, u_end, R, r, rc, m, I, n)

    % Direct Kinematics
    % angle, angular velocity, accelaration and COG accelaration
    z0=[0; 0; 1];
    w(1:3,1)=R(:,:,1)'*(w0+qd(1)*z0);
    wd(1:3,1)=R(:,:,1)'*(w0d+qdd(1)*z0+qd(1)*cross(w0,z0));
    pdd(1:3,1)=R(:,:,1)'*p0dd+cross(wd(1:3,1),r(1:3,1))+cross(w(1:3,1),cross(w(1:3,1),r(1:3,1)));
    pcdd(1:3,1)=pdd(1:3,1)+cross(wd(1:3,1),rc(1:3,1))+cross(w(1:3,1),cross(w(1:3,1),rc(1:3,1)));
    for i=2:n
        w(1:3,i)=R(:,:,i)'*(w(1:3,i-1)+qd(i)*z0);
        wd(1:3,i)=R(:,:,i)'*(wd(1:3,i-1)+qdd(i)*z0+qd(i)*cross(w(1:3,i-1),z0));
        pdd(1:3,i)=R(:,:,i)'*pdd(1:3,i-1)+cross(wd(1:3,i),r(1:3,i))+cross(w(1:3,i),cross(w(1:3,i),r(1:3,i)));
        pcdd(1:3,i)=pdd(1:3,i)+cross(wd(1:3,i),rc(1:3,i))+cross(w(1:3,i),cross(w(1:3,i),rc(1:3,i)));
    end
    % Reverse Dynamics
    % Force, torque
    f(1:3,n)=eye(3)*f_end+m(n)*pcdd(1:3,n);
    u(1:3,n)=-cross(f(1:3,n),r(1:3,n)+rc(1:3,n))+eye(3)*u_end+cross(eye(3)*f_end,rc(1:3,n))+...
    I(1:3,1:3,n)*wd(1:3,n)+cross(w(1:3,n),I(1:3,1:3,n)*w(1:3,n));
    for i=(n-1):-1:1
        f(1:3,i)=R(:,:,i+1)*f(1:3,i+1)+m(i)*pcdd(1:3,i);
        u(1:3,i)=-cross(f(1:3,i),r(1:3,i)+rc(1:3,i))+R(:,:,i+1)*u(1:3,i+1)+cross(R(:,:,i+1)*f(1:3,i+1),rc(1:3,i))+...
        I(1:3,1:3,i)*wd(1:3,i)+ cross(w(1:3,i),I(1:3,1:3,i)*w(1:3,i));
    end
    % Base Counterforce
    Fjoint1=-R(:,:,1)*f(1:3,1);
    Mjoint1=-R(:,:,1)*u(1:3,1);


end

