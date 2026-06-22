function [forceCmd, ctrlState] = ctrl_longitudinal(vxRef, vx, ax, ctrlState, CTRL, LIM, dt)
%CTRL_LONGITUDINAL [학생 작성] 종방향 제어기 (속도 추종 + ABS)
%
%   속도 추종 (cruise/decel) 과 anti-lock braking (slip ratio limiting) 을 통합.
%
%   Inputs:
%       vxRef     - 목표 종방향 속도 [m/s]
%       vx        - 실제 종방향 속도 [m/s]
%       ax        - 종가속도 [m/s²]
%       ctrlState - 내부 상태 (.intError, .prevForce, .wheelSlip(4) 추가 가능)
%       CTRL      - .LON.Kp, .Ki, .intMax
%       LIM       - .MAX_AX, .MAX_JERK, .MAX_BRAKE_TRQ
%       dt        - sample time
%
%   Outputs:
%       forceCmd.Fx_total   - 총 종방향 힘 요구 [N], 양수 가속 / 음수 제동
%       forceCmd.brakeRatio - 제동 비율 (0: 가속, 1: 전제동) — 차후 coordinator 가 brake 토크로 변환
%       ctrlState           - 업데이트
%
%   요구사항:
%       1. 속도 추종 PI 제어
%       2. ABS — wheel slip ratio |κ| > 0.12 일 때 brake force 감소 (slip-limit 또는 bang-bang)
%       3. 저크 제한 (LIM.MAX_JERK · m 으로 force 미분 cap)
%       4. anti-windup
%
%   주의:
%       - 본 함수는 wheel slip 정보가 직접 입력으로 들어오지 않음. 학생은 runner 가 매 step
%         result.tire.{FL,FR,RL,RR}.slipRatio 에 기록하는 값을 ctrlState 에 캐시하는 식으로
%         설계할 수 있음. 또는 ctrl_coordinator 에서 ABS 모듈레이션 (다른 설계 선택).
%       - 본 과제 시나리오 (B1) 는 vxRef 일정 — PID 속도 추종보다 ABS 가 핵심.
%
%   힌트:
%       - slip ratio κ = (ω·r_w - vx) / max(vx, 0.1)
%       - ABS 작동 조건: vehicle 감속 중 (ax < 0) AND |κ| > κ_target (≈0.12)
%       - Bang-bang ABS: brake_cmd = brake_cmd · 0.5 일 때 |κ| > κ_target

    %% TODO: 여기에 학생 구현
    %  (1) speed-tracking PI
    %  (2) ABS modulation (이번 함수에서 또는 ctrl_coordinator 에서)
    %  (3) jerk limit
    %  (4) anti-windup

    % 임시 baseline (반드시 본인 설계로 교체)
 %% 기본 출력값
    forceCmd.Fx_total   = 0;
    forceCmd.brakeRatio = 0;

    %% 방어 코드
    if nargin < 4 || isempty(ctrlState)
        ctrlState = struct();
    end
    if nargin < 8 || isempty(dt) || dt <= 0
        dt = 0.005;
    end

    %% 차량 질량 추정
    m = 1500;

    %% 속도 추종 PI 기본값
    % 현재 과제 runner는 vxRef로 scenario.vx0를 넣는 구조라서,
    % B1에서는 속도추종보다 ABS가 훨씬 중요하다.
    error_vx = vxRef - vx;

    if ~isfield(ctrlState, 'intError')
        ctrlState.intError = 0;
    end
    if ~isfield(ctrlState, 'prevForce')
        ctrlState.prevForce = 0;
    end

    % 아주 약한 속도 보정만 사용
    Kp = 0.5;
    Ki = 0.05;

    if isfield(CTRL, 'LON')
        if isfield(CTRL.LON, 'Kp'), Kp = CTRL.LON.Kp; end
        if isfield(CTRL.LON, 'Ki'), Ki = CTRL.LON.Ki; end
    end

    ctrlState.intError = ctrlState.intError + error_vx * dt;
    ctrlState.intError = max(min(ctrlState.intError, 20), -20);

    Fx_cmd = m * (Kp * error_vx + Ki * ctrlState.intError);

    % 과도한 추가 제동/가속 방지
    if isfield(LIM, 'MAX_AX')
        Fx_lim = LIM.MAX_AX * m;
    else
        Fx_lim = 8 * m;
    end

    Fx_cmd = max(min(Fx_cmd, Fx_lim), -Fx_lim);

    % jerk limit
    if isfield(LIM, 'MAX_JERK')
        dFmax = LIM.MAX_JERK * m * dt;
    else
        dFmax = 5000 * dt;
    end

    dF = Fx_cmd - ctrlState.prevForce;
    dF = max(min(dF, dFmax), -dFmax);
    Fx_cmd = ctrlState.prevForce + dF;
    ctrlState.prevForce = Fx_cmd;

    % 기본적으로는 추가 종방향 힘을 크게 쓰지 않는다.
    % B1은 scenario brake가 이미 존재하므로 ABS relief가 핵심.
    forceCmd.Fx_total = 0;

    %% ============================================================
    % ABS slip relief
    % slip ratio 목표: 약 0.12 근처
    % slip이 너무 커지면 brakeRatio를 음수로 만들어 기존 브레이크를 줄인다.
    %% ============================================================

    slipMax = 0;

    if isfield(ctrlState, 'wheelSlip') && ~isempty(ctrlState.wheelSlip)
        slip = abs(ctrlState.wheelSlip(:));
        slip = slip(isfinite(slip));

        if ~isempty(slip)
            slipMax = max(slip);
        end
    end

    reliefCmd = 0;

    % ax < 0이면 실제 제동 중일 가능성이 큼.
    % slip이 매우 큰 경우에도 ABS 개입.
    brakingLikely = (ax < -0.3) || (slipMax > 0.10);

    if brakingLikely
        if slipMax > 0.75
            reliefCmd = 0.65;
        elseif slipMax > 0.55
            reliefCmd = 0.50;
        elseif slipMax > 0.35
            reliefCmd = 0.35;
        elseif slipMax > 0.22
            reliefCmd = 0.22;
        elseif slipMax > 0.15
            reliefCmd = 0.10;
        else
            reliefCmd = 0.00;
        end
    end

    % ABS relief 필터
    if ~isfield(ctrlState, 'absReliefFilt')
        ctrlState.absReliefFilt = reliefCmd;
    end

    % slip이 커질 때는 빠르게 브레이크를 풀고,
    % slip이 작아졌을 때도 빠르게 브레이크를 다시 회복시킨다.
    if reliefCmd > ctrlState.absReliefFilt
        alpha = 0.75;
    else
        alpha = 0.55;
    end

    ctrlState.absReliefFilt = (1 - alpha) * ctrlState.absReliefFilt + alpha * reliefCmd;
    ctrlState.absReliefFilt = max(min(ctrlState.absReliefFilt, 0.95), 0);

    % 핵심: 음수 brakeRatio = 기존 시나리오 브레이크 감소 요청
    forceCmd.brakeRatio = -ctrlState.absReliefFilt;

end
