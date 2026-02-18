%% Robot diferencial con lidar
% Robotica Movil - 2025 2c
close all
clear all

verMatlab= ver('MATLAB');       % en MATLAB2020a funciona bien, ajustado para R2016b, los demas a pelearla...

simular_ruido_lidar = true;    %simula datos no validos del lidar real, probar si se la banca
use_roomba=false;               % false para desarrollar usando el simulador, true para conectarse al robot real

%% Número de desafío
desafio = 2;

%% Roomba
if use_roomba   % si se usa el robot real, se inicializa la conexion    
    rosshutdown
    pause(1)
    ipaddress_core = '192.168.0.102';
    ipaddress_local = '192.168.0.105';  %mi ip en a red TurtleNet
    setenv('ROS_IP', '192.168.0.105');
    setenv('ROS_MASTER_URI', ['http://', ipaddress_core, ':11311']);
    rosinit(ipaddress_core,11311, 'NodeHost', ipaddress_local)
    pause(.5)
    laserSub = rossubscriber('/scan');
    odomSub = rossubscriber('/odom');
    cmdPub = rospublisher('/auto_cmd_vel', 'geometry_msgs/Twist');
    pause(.5) % Esperar a que se registren los canales
    cmdMsg = rosmessage(cmdPub);  
end
    

%% Definicion del robot (disco de diametro = 0.35m)
R = 0.072/2;                % Radio de las ruedas [m]
L = 0.235;                  % Distancia entre ruedas [m]
dd = DifferentialDrive(R,L); % creacion del Simulador de robot diferencial

%% Creacion del entorno
load mapa_TP_2025b.mat      %carga el mapa como occupancyMap en la variable 'map'
% load mapa_fiuba_1p.mat      %carga el mapa como occupancyMap en la variable 'map'
% load mapa_lae.mat         %mapa viejo para probar cosas

if verMatlab.Release=='(R2016b)'
    %Para versiones anteriores de MATLAB, puede ser necesario ajustar mapa
    imagen_mapa = 1-double(imread('mapa_fiuba_1p.tiff'))/255;
    map = robotics.OccupancyGrid(imagen_mapa, 25);
elseif verMatlab.Release(1:5)=='(R201'    % Completar con la version que tengan
    %Ni idea que pasa, ver si el truco R2016b funciona
    disp('ver si la compatibilidad R2016b funciona');
else
    disp(['Utilizando MATLAB ', verMatlab.Release]);
end

%% Crear sensor lidar en simulador
lidar = LidarSensor;
lidar.sensorOffset = [0,0];     % Posicion del sensor en el robot (asumiendo mundo 2D)

if desafio == 1
    scaleFactor = 19;                %decimar lecturas de lidar acelera el algoritmo
else 
    scaleFactor = 3;
end

num_scans = 513/scaleFactor;
hokuyo_step_a = deg2rad(-90);
hokuyo_step_c = deg2rad(90);

lidar.scanAngles = linspace(hokuyo_step_a,hokuyo_step_c,num_scans);
lidar.maxRange = 5;

%% Crear visualizacion
viz = Visualizer2D;
viz.mapName = 'map';
attachLidarSensor(viz,lidar);

%% Parametros de la Simulacion

simulationDuration = 180;         % Duracion total [s]
sampleTime = 0.1;                   % Sample time [s]
% initPose = [14; 14; 3*pi/4];           % Pose inicial (x y theta) del robot simulado (el robot puede arrancar en cualquier lugar valido del mapa)
                                    %  probar iniciar el robot en distintos lugares                                  
% initPose = [10; 17; deg2rad(90)];
initPose = [26; 9; 3*pi/4];
% initPose = [21; 13; 3*pi/4]; %3/4
% initPose = [21; 21; 5*pi/4]; %1/4
% initPose = [27; 27; 3*pi/4]; %3/4
% initPose = [9; 8; pi/4];
% Inicializar vectores de tiempo:1010
tVec = 0:sampleTime:simulationDuration;         % Vector de Tiempo para duracion total

%% generar comandosx a modo de ejemplo
% vxRef = 0.1*ones(size(tVec));   % Velocidad lineal a ser comandada
% wRef = zeros(size(tVec));       % Velocidad angular a ser comandada
% wRef(tVec < 5) = -0.1;
% wRef(tVec >=7.5) = 0.1;

v_cmd = 0;
w_cmd = 0;

pose = zeros(3,numel(tVec));    % Inicializar matriz de pose
pose(:,1) = initPose;
move_count = 0;

% Inicializar Aca

if desafio == 1
    state = "Localization";
    n_iter = 0;

    path_obj = [12.5, 15];
    path = [0, 0];

    N_particles = 3000;
    particles = localization.initialize_particles(map, N_particles);
else
    move_count = 201;
    fig1 = figure(100);
    ax1 = axes(fig1);
    fig2 = figure(200);
    ax2 = axes(fig2);
    maxRange = 5;
    mapResolution = 15; % celdas por metro

    slamAlg = lidarSLAM(mapResolution, maxRange);
    slamAlg.LoopClosureThreshold = 210;
    slamAlg.LoopClosureSearchRadius = 8;
end

% cuidado = false;
% ang_obj = 0;
% w_cmd_ant = 0;

% Fin Inicializar Aca

%% Simulacion

if verMatlab.Release=='(R2016b)'
    r = robotics.Rate(1/sampleTime);    %matlab viejo no tiene funcion rateControl
else
    r = rateControl(1/sampleTime);  %definicion para R2020a, y posiblemente cualquier version nueva
end

for idx = 2:numel(tVec)   

    % Generar aqui criteriosamente velocidades lineales v_cmd y angulares w_cmd
    % -0.5 <= v_cmd <= 0.5 and -4.25 <= w_cmd <= 4.25
    % (mantener las velocidades bajas (v_cmd < 0.1) (w_cmd < 0.5) minimiza vibraciones y
    % mejora las mediciones.   
%     v_cmd = vxRef(idx-1);   % estas velocidades estan como ejemplo ...
%     w_cmd = wRef(idx-1);    %      ... para que el robot haga algo.
    
    %% COMPLETAR ACA:
     % generar velocidades para este timestep
%      w_cmd_ant = w_cmd;
%      
%      if cuidado
%          v_cmd = 0.01;
%          w_cmd = 0.3*ang_obj;
%      else
%          v_cmd = 0.1;
%          w_cmd = 0.5*ang_obj;
%          
%          w_cmd = w_cmd + sign(w_cmd)*0.1 + sign(w_cmd_ant)*0.1;
%      end
%          
%      w_cmd = max(min(w_cmd, 0.5), -0.5);
     
%     w_cmd = -0.4;
%     v_cmd = 0.2;
        
     % fin del COMPLETAR ACA
    
    %% a partir de aca el robot real o el simulador ejecutan v_cmd y w_cmd:
    
    if use_roomba       % para usar con el robot real
        
        % Enviar comando de velocidad en el formato que pide el robot:
        cmdMsg.Linear.X = v_cmd;
        cmdMsg.Angular.Z = w_cmd;
        send(cmdPub,cmdMsg);
        
        % Recibir datos de lidar y odometría
        scanMsg = receive(laserSub);
        odompose = odomSub.LatestMessage;
        
        % Obtener vector de distancias del lidar
        ranges_full = laserSub.LatestMessage.Ranges;
        ranges = double(ranges_full(1:scaleFactor:end));
        ranges(ranges==0)=NaN; % lecturas erroneas y maxrange
        ranges(ranges<0.05)=NaN; % compensación por errores de medicion no identificados a Dic/24
        
        % Obtener pose del robot [x,y,yaw] de datos de odometría (integrado por encoders).
        odomQuat = [odompose.Pose.Pose.Orientation.W, odompose.Pose.Pose.Orientation.X, ...
        odompose.Pose.Pose.Orientation.Y, odompose.Pose.Pose.Orientation.Z];
        odomRotation = quat2eul(odomQuat);
        pose(:,idx) = [odompose.Pose.Pose.Position.X + initPose(1); odompose.Pose.Pose.Position.Y+ initPose(2); odomRotation(1)];
    
    else        % para usar el simulador
   
        % Mover el robot segun los comandos generados
        [wL,wR] = inverseKinematics(dd,v_cmd,w_cmd);
        % Velocidad resultante
        [v,w] = forwardKinematics(dd,wL,wR);
        velB = [v;0;w]; % velocidades en la terna del robot [vx;vy;w]
        vel = bodyToWorld(velB,pose(:,idx-1));  % Conversion de la terna del robot a la global
        % Realizar un paso de integracion
        pose(:,idx) = pose(:,idx-1) + vel*sampleTime; 
        % Tomar nueva medicion del lidar
        ranges = double(lidar(pose(:,idx)));
        if simular_ruido_lidar
            % Simular ruido de un lidar ruidoso (probar a ver si se la banca)
            chance_de_medicion_no_valida = 0.17;
            not_valid=rand(length(ranges),1);
            ranges(not_valid<=chance_de_medicion_no_valida)=NaN;
        end
    end
    %%
    % Aca el robot ya ejecutó las velocidades comandadas y devuelve en la
    % variable ranges la medicion del lidar para ser usada y
    % en la variable pose(:,idx) la odometría actual.
    
    %% COMPLETAR ACA:
    % hacer algo con la medicion del lidar (ranges) y con el estado
    % actual de la odometria ( pose(:,idx) ) que se utilizará en la
    % proxima iteración para la generacion de comandos de velocidad
    
    disp('--------------------------')
    if desafio == 1
        [mean_pose, particles, state] = localization.main_loop(map, particles, v_cmd, w_cmd, ranges(1:6:end), lidar.scanAngles(1:6:end), sampleTime, n_iter, state);
        n_iter = n_iter + 1;
        disp(state);

        if state == "FindObjetive"
            path = planning.a_star(map, mean_pose, path_obj);
            state = "FollowPath";
        end

        [v_cmd, w_cmd, state, move_count] = movement.main_movement(mean_pose, state, ranges, lidar.scanAngles, move_count, path, path_obj);
    else
        num_scans = 15;
        if idx <= num_scans
            delta_pose = pose(:,idx)-pose(:,idx-1);
        else
            delta_pose = pose(:,idx)-pose(:,idx-num_scans);
        end
        scan = lidarScan(ranges, lidar.scanAngles);
        if mod(idx,num_scans)==0
            [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, scan, delta_pose);
            %[isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, scan);
        end
        [v_cmd, w_cmd, move_count] = movement.reactive_mapping(pose(:,idx), ranges, lidar.scanAngles, move_count);
    end
    
    
    
%     index = find(isnan(ranges));
%     if max(ranges) < 0.5 || min(ranges) < 0.1
%         cuidado = true;
%     else
%         cuidado = false;
%     end
%     
%     centro = floor(num_scans/2);
%     N = floor(num_scans*0.1);
%     if isempty(index)
%         [max_range, idx_range] = max(ranges(centro-N:centro+N));
%         ang_obj = lidar.scanAngles(idx_range);
%     else
%         angulos = lidar.scanAngles(index);
%         [~, k] = min(abs(angulos));
%         ang_obj = angulos(k);
%     end        
%         
%     if ang_obj < 0.1
%         [~,k] = min(ranges);
%         ang_obj = ang_obj - sign(k-centro)*0.5;
%     end
    % Fin del COMPLETAR ACA
        
    %%
    % actualizar visualizacion
    viz(pose(:,idx),ranges)
    
    waitfor(r);
end

if desafio == 2
    if isScanAccepted
        show(slamAlg, 'Parent', ax1);
        drawnow;
    end
    [scans, poses] = scansAndPoses(slamAlg);
    newMap = buildMap(scans, poses, mapResolution, maxRange);
    show(newMap, 'Parent', ax2);
end