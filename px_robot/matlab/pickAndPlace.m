%% Posiciones 
%% Se calcula las posiciones de los vértices de la ruta que debe realizar el robot.

d = 15; % Radio de los puntos de acenso y descenso
h = 5; % Desfase en altura entre el desplazamiento y el agarre de la pieza
TRa = transl(0,-d,10)*trotz(-pi/2)*troty(-pi); % Matriz de translacion Derecha Arriba
TRb = transl(0,-d,h)*trotz(-pi/2)*troty(-pi); % Matriz de translacion Derecha Abajo

TLa = transl(0,d,10)*trotz(pi/2)*troty(-pi); % Matriz de translacion Izquierda Arriba
TLb = transl(0,d,h)*trotz(pi/2)*troty(-pi); % Matriz de translacion Izquierda Abajo

TCa = transl(d,0,10)*troty(-pi);  % Matriz de translacion Centro Arriba
TCb = transl(d,0,h)*troty(-pi);  % Matriz de translacion Centro Abajo


%% Robot plot
%% Se definen los parámetros geométricos del robot.
l = [14.5, 10.25, 10.25, 9]; % Links lenght
% Robot Definition RTB
%% Se construye el modelo geométrico del robot en el toolbox del Peter Corke
L(1) = Link('revolute','alpha',pi/2,'a',0,   'd',l(1),'offset',0,   'qlim',[-3*pi/4 3*pi/4]);
L(2) = Link('revolute','alpha',0,   'a',l(2),'d',0,   'offset',pi/2,'qlim',[-3*pi/4 3*pi/4]);
L(3) = Link('revolute','alpha',0,   'a',l(3),'d',0,   'offset',0,   'qlim',[-3*pi/4 3*pi/4]);
L(4) = Link('revolute','alpha',0,   'a',0,   'd',0,   'offset',0,   'qlim',[-3*pi/4 3*pi/4]);
PhantomX = SerialLink(L,'name','Px');
PhantomX.tool = [0 0 1 l(4); -1 0 0 0; 0 -1 0 0; 0 0 0 1];
% Plotting
q = [0  0 0 0]; %% Home
q_rad = deg2rad(q);
PhantomX.plot(q_rad,'notiles','noname');
hold on
ws = [-50 50];
trplot(eye(4),'rgb','arrow','length',15,'frame','0')
axis([repmat(ws,1,2) 0 60])

%% Trayectorias 
%% Se calcula la cinemática inversa del robot para cada una de 
%% las posiciones de translación de vértices anteriormente calculadas.
% pos inicial - centro
P1 = invKinPhantomX(TCa,'up');
% pos derecha arriba
P2 = invKinPhantomX(TRa,'up');
% pos derecha abajo
P3 = invKinPhantomX(TRb,'up');
% pos izquierda arriba
P4 = invKinPhantomX(TLa,'up');
% pos izquierda abajo
P5 = invKinPhantomX(TLb,'up');
% pos centro abajo
P6 = invKinPhantomX(TCb,'up');

% Rutas
%% Se Calcula la interpolación entre vértices para hacer una ruta más suave.
%% Se usa un numero de interpolaciones para las rutas 
%% horizontales y otro para las rutas verticasles.
n = 30; % Numero de interpolaciones de ruta Horizontal.
n2 = 15; % Numero de interpolaciones de ruta Vertical
TC_R = ctraj(TCa,TRa,n); % Ruta Centro Arriba -- Derecha Arriba
TR_C = ctraj(TRa,TCa,n); % Ruta Derecha Arriba -- Centro Arriba
TC_L = ctraj(TCa,TLa,n); % Ruta Centro Arriba -- Izquierda Arriba
TL_C = ctraj(TLa,TCa,n); % Ruta Izquierda Arriba -- Centro Arriba
TC_a = ctraj(TCb,TCa,n2);  % Ruta Centro Abajo -- Centro Arriba
TC_b = ctraj(TCa,TCb,n2);  % Ruta Centro Arriba -- Centro Abajo
TR_a = ctraj(TRb,TRa,n2);  % Ruta Derecha Abajo -- Derecha Arriba
TR_b = ctraj(TRa,TRb,n2);  % Ruta Derecha Arriba -- Derecha Abajo
TL_a = ctraj(TLb,TLa,n2);  % Ruta Izquierda Abajo -- Izquierda Arriba
TL_b = ctraj(TLa,TLb,n2);  % Ruta Izquierda Arriba -- Izquierda Abajo

%% Valores para las posisiones del gripper
open = 0;
close = 0.8624;%mapfun(680,0,1023,deg2rad(-150),deg2rad(150))

%% Calculo de la ruta completa, teniendo en cuenta los 
%% vértices y los puntos de interpolación.

%home
q_home = [0 0 0 0 open];

%Pos ini - centro arriba
qC_a = [P1 open];

%Centro a derecha
for i=1:n
   qinv = invKinPhantomX(TC_R(:,:,i),'up');
   qC_R(i,:) = [qinv open];
end

%Derecha abajo y cerrar
for i=1:n2
   qinv = invKinPhantomX(TR_b(:,:,i),'up');
   qR_b(i,:) = [qinv open];
end
qR_b = [qR_b; P3 open; P3 close];


%Derecha arriba
for i=1:n2
   qinv = invKinPhantomX(TR_a(:,:,i),'up');
   qR_a(i,:) = [qinv close];
end
% qR_a = [P2 close];

%Derecha a Centro
for i=1:n
   qinv = invKinPhantomX(TR_C(:,:,i),'up');
   qR_C(i,:) = [qinv close];
end

%Centro abajo y abrir
for i=1:n2
   qinv = invKinPhantomX(TC_b(:,:,i),'up');
   qC_b(i,:) = [qinv close];
end
qC_b = [qC_b; P6 close; P6 open];

%Centro arriba 2
for i=1:n2
   qinv = invKinPhantomX(TC_a(:,:,i),'up');
   qC_a2(i,:) = [qinv open];
end

%Centro a izquierda
for i=1:n
   qinv = invKinPhantomX(TC_L(:,:,i),'up');
   qC_L(i,:) = [qinv open];
end

%Izquierda abajo y cerrar
for i=1:n2
   qinv = invKinPhantomX(TL_b(:,:,i),'up');
   qL_b(i,:) = [qinv open];
end
qL_b = [qL_b; P5 open; P5 close];

%Izquierda arriba
for i=1:n2
   qinv = invKinPhantomX(TL_a(:,:,i),'up');
   qL_a(i,:) = [qinv close];
end
qL_a = [qL_a; P4 close];

%Izquierda a centro
for i=1:n
   qinv = invKinPhantomX(TL_C(:,:,i),'up');
   qL_C(i,:) = [qinv close];
end

%Centro abajo y abrir

%Centro arriba 2

%Matriz total de valores articulares de la ruta completa.

qTotal = [q_home;
          qC_a;
          qC_R;
          qR_b;
          qR_a;
          qR_C;
          qC_b;
          qC_a2;
          qC_L;
          qL_b;
          qL_a;
          qL_C;
          qC_b;
          qC_a2];
      
qTotal(:,4) = qTotal(:,4)-2*pi;
qTotal(1,4) = 0;
      
for i=1:length(qTotal)
    PhantomX.plot(qTotal(i,1:4),'notiles','noname')
end
        



%% ROS Connection 
rosinit
%%
motorSvcClient = rossvcclient('dynamixel_workbench/dynamixel_command');%creacion del cliente
motorCommandMsg = rosmessage(motorSvcClient);%creacion del mensaje

%% Configuracion de la velocidad maxima para todas 
%% las articulaciones, para garantizar que los motores 
%% no se mueven a altas velocidades, con esto se evita 
%% reducierles el torque a los motores y se tiene tiempo
%% para prevenir choques del robot.
motorCommandMsg.AddrName = "Moving_Speed";
motorCommandMsg.Value = 100;
motorCommandMsg.Id = 1;
call(motorSvcClient,motorCommandMsg);
motorCommandMsg.Id = 2;
call(motorSvcClient,motorCommandMsg);
motorCommandMsg.Id = 3;
call(motorSvcClient,motorCommandMsg);
motorCommandMsg.Id = 4;
call(motorSvcClient,motorCommandMsg);
motorCommandMsg.Id = 5;
call(motorSvcClient,motorCommandMsg);

%% Se ejecutan los movientes de todas las articulaciones
%% para cumplir con la ruta planeada, al usar la 
%% función Goal_Position se presenta el inconveniente
%% de que el robot no ejecuta la siguiente acción hasta 
%% que se haya completado la primera.
motorCommandMsg.AddrName = "Goal_Position";
for j=1:length(qTotal)
    for i=1:5
        if j~=1
           if qTotal(j,i)~=qTotal(j-1,i)
               motorCommandMsg.Id = i;
               motorCommandMsg.Value = round(mapfun(qTotal(j,i),deg2rad(-150),deg2rad(150),0,1023));
               call(motorSvcClient,motorCommandMsg);
           end
        else
            motorCommandMsg.Id = i;
            motorCommandMsg.Value = round(mapfun(qTotal(j,i),deg2rad(-150),deg2rad(150),0,1023));
            call(motorSvcClient,motorCommandMsg);
        end
        
    end
    pause(0.5)
end

%% Move with keyboard
%% Función para mover el robot en pasos predeterminados
%% con el teclado, se mueve en los ejes X, Y, Z y se rota
%% el ángulo de ataque respecto a la Horizontal
X = 15;
Y = 0;
Z = 10;
%% Se determina la cinemática inversa de la posición inicial.
T_ini = transl(X,Y,Z)*trotz(atan2(Y,X))*troty(-pi);
Pini = invKinPhantomX(T_ini, 'up')
PhantomX.plot(Pini,'notiles','noname')
T_act = T_ini;
for i=1:5
    [X, Y, Z] = transl(T_act); % Posición cartesiana actual.
    angles = tr2eul(T_act); % Angulos de Euler actuales.
    aZ1 = angles(1)
    aY = angles(2)
    aZ2 = angles(3)
    d_tra = 1; % Paso de translacion
    d_rot = 5*pi/180;
    aY = aY + -1*d_rot;
    Y = Y + -1*d_tra;
    aZ2 = -atan2(Y,X);
    T_act = rt2tr(eul2r(aZ1,aY,aZ2),[X Y Z]');
    q_act = invKinPhantomX(T_act, 'up');
    PhantomX.plot(q_act,'notiles','noname')
    i
end
