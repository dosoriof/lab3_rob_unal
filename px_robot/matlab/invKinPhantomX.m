%{
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Cinematica inversa para robot Phantom X
Por: Diego Osorio y Ricardo Galindo
Entradas:
T: Pose del EF
elbow: 'up' para codo arriba, 'down' para codo abajo
Salida:
q_inv: Variables del espacio articular en radianes
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%}
function q_inv = invKinPhantomX(T, elbow)
    l = [14.5, 10.25, 10.25, 9]; % Longitudes eslabones
    
    %Desacople
    Posw = T(1:3,4) - l(4)*T(1:3,3); % MTH Wrist
    
    %q1
    if (T(2,4)==0)&&(T(1,4)==0)
        disp('Singular position. q1 can have any value')
        q1 = NaN;
    else
        q1 = atan2(T(2,4),T(1,4));
    end
    
    % Solución 2R
    h = Posw(3) - l(1);
    r = sqrt(Posw(1)^2 + Posw(2)^2);
    
    % Solucion de q4
    Rp = (rotz(q1))'*T(1:3,1:3);
    pitch = atan2(Rp(3,1),Rp(1,1));
    
    
    the3 = acos((r^2+h^2-l(2)^2-l(3)^2)/(2*l(2)*l(3)));
    if isreal(the3)
        if(strcmp('down',elbow))
            the2 = atan2(h,r) - atan2(l(3)*sin(the3),l(2)+l(3)*cos(the3));
            q2 = -(pi/2-the2);
            q3 = the3;
            q4 = pitch - q2 - q3;
        end
        if (strcmp('up',elbow))
            the2 = atan2(h,r) + atan2(l(3)*sin(the3),l(2)+l(3)*cos(the3));
            q2 = -(pi/2-the2);
            q3 = -the3; 
            q4 = pitch - q2 - q3;
        end
    else
         q1 = NaN; q2 = NaN; q3 = NaN; q4 = NaN;
    end
        
    
    if (~(strcmp('down',elbow))&&~(strcmp('up',elbow)))
        disp('Wrong elbow imput - Write up or down')
        q1 = NaN; q2 = NaN; q3 = NaN; q4 = NaN;
    end
    q_inv = [q1 q2 q3 q4];
end