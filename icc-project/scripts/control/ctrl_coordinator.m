function actuatorCmd = ctrl_coordinator(latCmd, lonCmd, verCmd, vx, VEH, CTRL, LIM)
%CTRL_COORDINATOR [학생 작성] Actuator Allocation — 횡/종/수직 명령을 actuator 로 분배
%
%   상위 제어기들의 명령 (yaw moment, Fx_total, damping) 을 차량 actuator
%   (steerAngle, 4-wheel brake torque, 4-wheel damping) 로 변환.
%
%   Inputs:
%       latCmd.steerAngle - AFS 보조 조향 [rad]
%       latCmd.yawMoment  - ESC 요청 yaw moment [Nm]
%       lonCmd.Fx_total   - 종방향 힘 요구 [N]
%       lonCmd.brakeRatio - 제동 비율
%       verCmd            - 4×1 damping [Ns/m] (ctrl_vertical 출력)
%       vx, VEH, CTRL, LIM
%
%   Output:
%       actuatorCmd.steerAngle    - 최종 조향각 [rad], LIM.MAX_STEER_ANGLE 제한
%       actuatorCmd.brakeTorque   - 4×1 brake torque [Nm], [FL; FR; RL; RR], LIM.MAX_BRAKE_TRQ 제한
%       actuatorCmd.dampingCoeff  - 4×1 [Ns/m]
%
%   요구사항:
%       1. 종방향 제동 (lonCmd.Fx_total < 0) 의 4륜 균등 분배 — 전후 비율 60:40 권장
%       2. ESC yaw moment → brake 차동 분배 (좌/우 비대칭)
%             양의 M_z (CCW) → 좌측 brake 증가 또는 우측 brake 감소
%             track 반거리: t_f/2 = VEH.track_f/2,  t_r/2 = VEH.track_r/2
%             dT_f = M_z · ratio_f / t_f,  dT_r = M_z · (1-ratio_f) / t_r
%       3. AFS steerAngle 그대로 통과 + saturation
%       4. brake torque 합산 후 [0, MAX_BRAKE_TRQ] 클리핑
%
%   가산점 (선택):
%       - 마찰원 제한: 각 휠의 brake torque + cornering force 가 μ·Fz 안으로
%       - WLS allocation: actuator effort minimize 목적함수
%       - per-wheel 최대 토크 제한 — wheel slip 임계 도달 시 감소
%
%   힌트:
%       - half-track: t_f/2 ≈ 0.78 m (BMW_5)
%       - 종방향 brake 시 force-to-torque: T = |Fx_total|/4 · r_w  (r_w ≈ 0.33 m)
%       - allocation matrix form 도 가능 (LQ allocation)

    %% TODO: 학생 구현
    %  (1) lonCmd.Fx_total → 4-wheel 균등 brake (with 60:40 split)
    %  (2) latCmd.yawMoment → 4-wheel 차동 brake
    %  (3) latCmd.steerAngle → actuatorCmd.steerAngle (saturation)
    %  (4) verCmd → actuatorCmd.dampingCoeff (pass-through 또는 추가 가공)
    %  (5) 최종 saturation

    % 임시 baseline (반드시 교체)
 %% 기본 출력
    actuatorCmd.steerAngle   = 0;
    actuatorCmd.brakeTorque  = zeros(4, 1);
    actuatorCmd.dampingCoeff = 1500 * ones(4, 1);

    %% 입력 방어
    if ~exist('latCmd','var') || isempty(latCmd) || ~isstruct(latCmd)
        latCmd = struct();
    end
    if ~exist('lonCmd','var') || isempty(lonCmd) || ~isstruct(lonCmd)
        lonCmd = struct();
    end
    if ~exist('verCmd','var') || isempty(verCmd)
        verCmd = actuatorCmd.dampingCoeff;
    end

    if ~isfield(latCmd, 'steerAngle') || isempty(latCmd.steerAngle)
        latCmd.steerAngle = 0;
    end
    if ~isfield(latCmd, 'yawMoment') || isempty(latCmd.yawMoment)
        latCmd.yawMoment = 0;
    end
    if ~isfield(lonCmd, 'Fx_total') || isempty(lonCmd.Fx_total)
        lonCmd.Fx_total = 0;
    end
    if ~isfield(lonCmd, 'brakeRatio') || isempty(lonCmd.brakeRatio)
        lonCmd.brakeRatio = 0;
    end

    %% 제한값
    if isfield(LIM, 'MAX_STEER_ANGLE')
        maxSteer = LIM.MAX_STEER_ANGLE;
    else
        maxSteer = deg2rad(10);
    end

    if isfield(LIM, 'MAX_BRAKE_TRQ')
        maxBrake = LIM.MAX_BRAKE_TRQ;
    else
        maxBrake = 3000;
    end

    %% 차량 파라미터
    r_w = 0.31;
    track = 1.55;

    if isstruct(VEH)
        if isfield(VEH, 'r_w')
            r_w = VEH.r_w;
        elseif isfield(VEH, 'Rw')
            r_w = VEH.Rw;
        elseif isfield(VEH, 'wheel_radius')
            r_w = VEH.wheel_radius;
        end

        if isfield(VEH, 'track_f')
            track = VEH.track_f;
        end
    end

    track = max(track, 1.0);

    %% 1. 조향 명령
    steerCmd = latCmd.steerAngle;
    if ~isfinite(steerCmd)
        steerCmd = 0;
    end

    actuatorCmd.steerAngle = max(min(steerCmd, maxSteer), -maxSteer);

    %% 2. 종방향 brake 보정
    % brakeRatio > 0 : 추가 제동
    % brakeRatio < 0 : ABS relief, 기존 시나리오 제동 감소
    brakeRatio = lonCmd.brakeRatio;

    if ~isfinite(brakeRatio)
        brakeRatio = 0;
    end

    brakeRatio = max(min(brakeRatio, 1), -1);

    T_lon = zeros(4, 1);

    if brakeRatio > 0
        % 추가 제동: 전후 60:40
        T_lon = brakeRatio * maxBrake * [0.60; 0.60; 0.40; 0.40];

    elseif brakeRatio < 0
        % ABS relief: 기존 브레이크를 줄이기 위한 음수 토크
        relief = abs(brakeRatio);

        % 네 바퀴 모두 줄이되, 전륜을 조금 더 많이 줄여 lock 완화
        T_lon = -relief * maxBrake * [1.00; 1.00; 0.85; 0.85];
    end

    %% 3. Fx_total 기반 추가 제동, 필요 시만 사용
    Fx_total = lonCmd.Fx_total;

    if ~isfinite(Fx_total)
        Fx_total = 0;
    end

    T_fx = zeros(4, 1);

    if Fx_total < 0
        F_brake = abs(Fx_total);

        T_fx = [0.30 * F_brake * r_w; ...
                0.30 * F_brake * r_w; ...
                0.20 * F_brake * r_w; ...
                0.20 * F_brake * r_w];
    end

    %% 4. ESC yaw moment 차동 제동
    Mz = latCmd.yawMoment;

    if ~isfinite(Mz)
        Mz = 0;
    end

    % 너무 큰 yaw moment 제한
    Mz = max(min(Mz, 7000), -7000);

    % 좌우 차동 제동
    % 양의 Mz: 좌측 brake 증가, 우측 brake 감소
    ratio_f = 0.70;
    ratio_r = 0.30;

    dT_f = ratio_f * Mz * r_w / track;
    dT_r = ratio_r * Mz * r_w / track;

    T_esc = [ dT_f; ...
             -dT_f; ...
              dT_r; ...
             -dT_r ];

    %% 5. 최종 brake 보정값
    T_total = T_lon + T_fx + T_esc;
    T_total(~isfinite(T_total)) = 0;

    % 중요:
    % 여기서 0~maxBrake로 자르면 ABS relief가 사라진다.
    % runner가 brk_scenario + brakeESC 후 최종 0~maxBrake saturation을 수행한다.
    actuatorCmd.brakeTorque = max(min(T_total, maxBrake), -maxBrake);

    %% 6. damping 명령
    if isscalar(verCmd)
        damping = verCmd * ones(4, 1);
    else
        damping = verCmd(:);
    end

    if numel(damping) < 4
        damping = 1500 * ones(4, 1);
    else
        damping = damping(1:4);
    end

    damping(~isfinite(damping)) = 1500;

    if isfield(CTRL, 'VER')
        if isfield(CTRL.VER, 'cMin')
            cMin = CTRL.VER.cMin;
        else
            cMin = 500;
        end
        if isfield(CTRL.VER, 'cMax')
            cMax = CTRL.VER.cMax;
        else
            cMax = 5000;
        end
    else
        cMin = 500;
        cMax = 5000;
    end

    actuatorCmd.dampingCoeff = max(min(damping, cMax), cMin);

end