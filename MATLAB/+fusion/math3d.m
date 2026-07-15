function varargout = math3d(action, varargin)
%MATH3D Frame and scalar-first quaternion utilities.

switch lower(action)
    case 'ecef_to_geodetic'
        r = varargin{1}(:); x = r(1); y = r(2); z = r(3);
        a = 6378137.0; e2 = 6.69437999014e-3;
        lon = atan2(y, x); p = hypot(x, y);
        lat = atan2(z, p * (1 - e2));
        for i = 1:10
            n = a / sqrt(1 - e2 * sin(lat)^2);
            alt = p / max(cos(lat), 1e-15) - n;
            next = atan2(z, p * (1 - e2 * n / (n + alt)));
            if abs(next - lat) < 1e-13; lat = next; break; end
            lat = next;
        end
        n = a / sqrt(1 - e2 * sin(lat)^2);
        alt = p / max(cos(lat), 1e-15) - n;
        varargout = {lat, lon, alt};
    case 'ecef_to_ned_matrix'
        lat = varargin{1}; lon = varargin{2};
        sl = sin(lat); cl = cos(lat); so = sin(lon); co = cos(lon);
        varargout{1} = [-sl*co, -sl*so, cl; -so, co, 0; -cl*co, -cl*so, -sl];
    case 'normalize'
        v = varargin{1}(:); n = norm(v);
        if n < 1e-12; error('fusion:Vector', 'Cannot normalize a zero vector.'); end
        varargout{1} = v / n;
    case 'project_rotation'
        [u,~,v] = svd(varargin{1});
        varargout{1} = u * diag([1, 1, det(u*v')]) * v';
    case 'matrix_to_quaternion'
        m = fusion.math3d('project_rotation', varargin{1});
        tr = trace(m);
        if tr > 0
            s = sqrt(tr + 1) * 2;
            q = [0.25*s, (m(3,2)-m(2,3))/s, (m(1,3)-m(3,1))/s, (m(2,1)-m(1,2))/s];
        else
            [~,idx] = max(diag(m));
            if idx == 1
                s = sqrt(1 + m(1,1)-m(2,2)-m(3,3))*2;
                q = [(m(3,2)-m(2,3))/s, .25*s, (m(1,2)+m(2,1))/s, (m(1,3)+m(3,1))/s];
            elseif idx == 2
                s = sqrt(1 + m(2,2)-m(1,1)-m(3,3))*2;
                q = [(m(1,3)-m(3,1))/s, (m(1,2)+m(2,1))/s, .25*s, (m(2,3)+m(3,2))/s];
            else
                s = sqrt(1 + m(3,3)-m(1,1)-m(2,2))*2;
                q = [(m(2,1)-m(1,2))/s, (m(1,3)+m(3,1))/s, (m(2,3)+m(3,2))/s, .25*s];
            end
        end
        q = q / norm(q); if q(1) < 0; q = -q; end
        varargout{1} = q;
    case 'quaternion_to_matrix'
        q = varargin{1}(:)' / norm(varargin{1});
        w=q(1); x=q(2); y=q(3); z=q(4);
        varargout{1} = [1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y); ...
            2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x); ...
            2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)];
    case 'quaternion_multiply'
        a=varargin{1}(:)'; b=varargin{2}(:)';
        varargout{1} = [a(1)*b(1)-dot(a(2:4),b(2:4)), ...
            a(1)*b(2)+a(2)*b(1)+a(3)*b(4)-a(4)*b(3), ...
            a(1)*b(3)-a(2)*b(4)+a(3)*b(1)+a(4)*b(2), ...
            a(1)*b(4)+a(2)*b(3)-a(3)*b(2)+a(4)*b(1)];
    case 'quaternion_from_rotvec'
        v=varargin{1}(:)'; angle=norm(v);
        if angle < 1e-12; q=[1,.5*v]; q=q/norm(q); else; q=[cos(angle/2),sin(angle/2)*v/angle]; end
        varargout{1}=q;
    otherwise
        error('fusion:Math', 'Unknown math3d action: %s', action);
end
end
