function TF = rotx( q,ext_coord,fixed )
if nargin<2
    ext_coord=q;
end
if nargin<3
    fixed=0;
end
if strcmp(class(q),'sym')
    idx=find(logical(ext_coord==q));
else
    idx=q;
end

%         M=[1    0       0       0;
%            0    cos(q)  -sin(q) 0;
%            0    sin(q)  cos(q)  0;
%            0    0       0       1];


if fixed
    one=SRD.one(q,ext_coord);
    zero=SRD.zero(q,ext_coord);
    c=SRD.one(q,ext_coord)*cos(q);
    s=SRD.one(q,ext_coord)*sin(q);
else
    one=SRD.one(q,ext_coord);
    zero=SRD.zero(q,ext_coord);
    c=SRD.cos(q,ext_coord);
    s=SRD.sin(q,ext_coord);
end
TF(1,1)=one;  TF(1,2)=zero; TF(1,3)=zero; TF(1,4)=zero;
TF(2,1)=zero; TF(2,2)=c;    TF(2,3)=-s;   TF(2,4)=zero;
TF(3,1)=zero; TF(3,2)=s;    TF(3,3)=c;    TF(3,4)=zero;
TF(4,1)=zero; TF(4,2)=zero; TF(4,3)=zero; TF(4,4)=one;

end

