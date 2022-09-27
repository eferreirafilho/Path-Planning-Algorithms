clear all
close all

qi=[0; 0; 0];

qf=[20; 10; pi/1.5];

r = 5;

% Calcular os quatro c�rculos
cir=qi(1:2)+r*[sin(qi(3)); -cos(qi(3))];
cil=qi(1:2)+r*[-sin(qi(3)); cos(qi(3))];
cfr=qf(1:2)+r*[sin(qf(3)); -cos(qf(3))];
cfl=qf(1:2)+r*[-sin(qf(3)); cos(qf(3))];

figure(1)
hold on


circle(cir(1), cir(2), r, 'r');
circle(cil(1), cil(2), r, 'r');
quiver(qi(1), qi(2), r*cos(qi(3)), r*sin(qi(3)), 'color', 'r');


circle(cfr(1), cfr(2), r, 'b');
circle(cfl(1), cfl(2), r, 'b');
quiver(qf(1), qf(2), r*cos(qf(3)), r*sin(qf(3)), 'color', 'b');

axis equal
%% 

% calcular LSL

% Calcular vetor entre os dois c�rculos
vll=cfl-cil;

% Calcular a tangente externa � direita de vll -> sempre usaremos somente essa
% tangente!!

ortvll=[vll(2); -vll(1)];
pti=cil+r*ortvll/norm(ortvll);
ptf=pti+vll;

ptiLSL=pti;
ptfLSL=ptf;


% Calcular o tamanho do caminho
% Arco no primeiro circulo
vLSL=cross([(qi(1:2)-cil)/r; 0], [(pti-cil)/r; 0]);
deltatheta=asin(norm(vLSL));
if vLSL(3)<0 
    deltatheta=2*pi-deltatheta;
end
L_LSL=deltatheta*r;

% Arco no segundo circulo
vLSL=cross([(qf(1:2)-cfl)/r; 0], [(ptf-cfl)/r; 0]);
deltatheta=asin(norm(vLSL));
if vLSL(3)>0
    deltatheta=2*pi-deltatheta;
end
L_LSL=L_LSL+deltatheta*r;

% Comprimento do vetor
L_LSL=L_LSL+norm(vll);
%% 

%% 

% calcular RSR

% Calcular vetor entre os dois c�rculos
vrr=cfr-cir;

% Calcular a tangente externa � esquerda de vll -> sempre usaremos somente essa
% tangente!!

ortvrr=[-vrr(2); vrr(1)];
pti=cir+r*ortvrr/norm(ortvrr);
ptf=pti+vrr;

ptiRSR=pti;
ptfRSR=ptf;


% Calcular o tamanho do caminho
% Arco no primeiro circulo
vRSR=cross([(qi(1:2)-cir)/r; 0], [(pti-cir)/r; 0]);
deltatheta=asin(norm(vRSR));
if vRSR(3)<0 
    deltatheta=2*pi-deltatheta;
end;
L_RSR=deltatheta*r;

% Arco no segundo circulo
vRSR=cross([(qf(1:2)-cfr)/r; 0], [(ptf-cfr)/r; 0]);
deltatheta=asin(norm(vRSR));
if vRSR(3)>0
    deltatheta=2*pi-deltatheta;
end;
L_RSR=L_RSR+deltatheta*r;

% Comprimento do vetor
L_RSR=L_RSR+norm(vrr);
%% 




%% 

% calcular LSR

% Calcular vetor entre os dois c�rculos
vlr=cfr-cil;

% calcular a tangente interna cujo ponto em cil est� a direita do vetor vlr 

%Encontre os ângulos envolvidos
% Calculo de theta
thetaLSR=acos(2*r/norm(vlr));

%Calculo de alpha
alphaLSR=atan2(-vlr(2),vlr(1));

%Com os angulos, encontre o vetor vn1 e o ponto c=2 . vn1
%Calculo de vn1
vn1LSR(1,1)=r*cos(thetaLSR + alphaLSR);
vn1LSR(2,1)=-r*sin(thetaLSR + alphaLSR);
cLSR=2*vn1LSR;

%Os extremos da tangente são c1+vn1 e c2-c-vn1

tanLSR(2,:)=cil+vn1LSR;
tanLSR(1,:)=cfr-cLSR+vn1LSR;

% Calcular o tamanho do caminho
% Arco no primeiro circulo
vLSR=cross([(qi(1:2)-cil)/r; 0], [(pti-cil)/r; 0]);
deltatheta=asin(norm(vLSR));
if vLSR(3)<0 
    deltatheta=2*pi-deltatheta;
end
L_LSR=deltatheta*r;

% Arco no segundo circulo
vLSR=cross([(qf(1:2)-cfr)/r; 0], [(ptf-cfr)/r; 0]);
deltatheta=asin(norm(vLSR));
if vLSR(3)>0
    deltatheta=2*pi-deltatheta;
end
L_LSR=L_LSR+deltatheta*r;

% Comprimento do vetor
L_LSR=L_LSR+norm(vlr);
%% 



%% 

% calcular RSL

% Calcular vetor entre os dois c�rculos
vrl=cfl-cir;

% calcular a tangente interna cujo ponto em cir est� a esquerda do vetor vrl 

%Encontre os ângulos envolvidos
% Calculo de theta
thetaRSL=acos(2*r/norm(vrl));

%Calculo de alpha
alphaRSL=atan2(vrl(2),vrl(1));

%Com os angulos, encontre o vetor vn1 e o ponto c=2 . vn1
%Calculo de vn1
vn1RSL(1,1)=r*cos(thetaRSL + alphaRSL);
vn1RSL(2,1)=r*sin(thetaRSL + alphaRSL);
cRSL=2*vn1RSL;

%Os extremos da tangente são c1+vn1 e c2-c-vn1

tanRSL(2,:)=cir+vn1RSL;
tanRSL(1,:)=cfl-cRSL+vn1RSL;

% 
% plot(tanRSL(1,1),tanRSL(1,2),'d');
% plot(tanRSL(2,1),tanRSL(2,2), 'd');
% line(tanRSL(:,1),tanRSL(:,2))



% Calcular o tamanho do caminho
% Arco no primeiro circulo
vRSL=cross([(qi(1:2)-cir)/r; 0], [(pti-cir)/r; 0]);
deltatheta=asin(norm(vRSL));
if vRSL(3)<0 
    deltatheta=2*pi-deltatheta;
end
L_RSL=deltatheta*r;

% Arco no segundo circulo
vRSL=cross([(qf(1:2)-cfl)/r; 0], [(ptf-cfl)/r; 0]);
deltatheta=asin(norm(vRSL));
if vRSL(3)>0
    deltatheta=2*pi-deltatheta;
end
L_RSL=L_RSL+deltatheta*r;

% Comprimento do vetor
L_RSL=L_RSL+norm(vrl);
%% 


%Choosing and ploting smallest dubin path

L=[L_LSL L_RSR L_LSR L_RSL];

[I] = find(L==min(L(:)));

disp('Small(est) Dubin Path(s): ')
if any(I==1)
    disp('    LSL')
    plot(ptiLSL(1),ptiLSL(2), 'ko');
    plot(ptfLSL(1),ptfLSL(2), 'ko');
    plot([ptiLSL(1,1) ptfLSL(1,1)],[ptiLSL(2,1) ptfLSL(2,1)],'k')
end
if any(I==2)
    disp('     RSR')
    plot(ptiRSR(1),ptiRSR(2), 'ko');
    plot(ptfRSR(1),ptfRSR(2), 'ko');
    plot([ptiRSR(1,1) ptfRSR(1,1)],[ptiRSR(2,1) ptfRSR(2,1)],'k')
end
if any(I==3)
    disp('    LSR')
    plot(tanLSR(1,1),tanLSR(1,2),'ko');
    plot(tanLSR(2,1),tanLSR(2,2), 'ko');
    plot(tanLSR(:,1),tanLSR(:,2),'k')
end
if any(I==4)
    disp('    RSL')
    plot(tanLSR(1,1),tanLSR(1,2),'ko');
    plot(tanLSR(2,1),tanLSR(2,2), 'ko');
    plot(tanLSR(:,1),tanLSR(:,2),'k')
end
    
disp('Cost: ')
disp(L(I(1)))
    











