function hFrame = drawSSV(obj,nlnk,varargin)
% function [surf_obj] = drawSSV(obj,nlnk)
% function [surf_obj] = drawSSV(obj,nlnk,ssv_index)
% function [surf_obj] = drawSSV(h,____)
% function [surf_obj] = drawSSV(____,'Property',Value)
% Draws an SSV. Points in obj.ssv must be 4xN.
% If the SSV in question is polygonal, points in obj.ssv must be coplanar
% and enumerated counter-clockwise if seen from above.
%
% Prop/value: points = number of points of render (must be even, default 30)
%             span = linear/polygonal, type of SSV (default = linear)
%             frame = ground/chain, renders the nlnk in place or unconstrained
% other props: plot props (must be the last ones)

N = 32;
if nlnk <= length(obj.SSV)
    ssv = obj.SSV(nlnk);
    
    % parametri in ingresso
    i = 1;
    while i <= length(varargin)
        prop = varargin{i};
        switch lower(prop)
            case 'points'
                N = varargin{i+1};
            case {'facecolor','edgecolor','facealpha','edgealpha'}
                faceProp = varargin(i:end);
                break;
        end
        i = i + 2;
    end
    hFrame = gobjects(1,length(ssv.L));
    for ns = 1:length(ssv.L)
        
        % calcolo punti
        [X,Y,Z] = sphere(N-1);
        ND = N/2;
        X(ND+1:end,:) = ssv.R(ns)*X(ND+1:end,:);
        Y(ND+1:end,:) = ssv.R(ns)*Y(ND+1:end,:);
        Z(ND+1:end,:) = ssv.R(ns)*Z(ND+1:end,:)+ssv.L(ns);
        X(1:ND,:) = ssv.R(ns)*X(1:ND,:);
        Y(1:ND,:) = ssv.R(ns)*Y(1:ND,:);
        Z(1:ND,:) = ssv.R(ns)*Z(1:ND,:);
        % rotazione e shift
        [m,n] = size(X);
        XYZ = ssv.rot(:,:,ns)*ssv.shift(:,:,ns)*[X(:)';Y(:)';Z(:)';ones(1,numel(X))];
        % visualizzazione
        hFrame(ns) = hgtransform('Matrix',eye(4),'Parent',obj.hLink(nlnk));
        %hFrame(ns) = hgtransform('Matrix',obj.matrici.T0a*obj.matrici.Ti0{nlnk},'Parent',obj.hAx);
        surf(reshape(XYZ(1,:),m,n),reshape(XYZ(2,:),m,n),reshape(XYZ(3,:),m,n),faceProp{:},'Parent',hFrame(ns));
    end
end