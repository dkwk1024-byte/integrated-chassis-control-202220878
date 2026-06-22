function [dampingCmd, ctrlState] = ctrl_vertical(suspState, ctrlState, CTRL, dt)
%CTRL_VERTICAL [학생 작성] CDC (Continuous Damping Control) — per-wheel 감쇠 명령
%
%   Body-bounce / wheel-hop 모드 분리 및 ride comfort 개선을 위한 가변 감쇠.
%
%   Inputs:
%       suspState - struct, 각 wheel 의 sprung/unsprung velocity 등
%           .zs_dot(4)     - sprung mass velocity (위쪽 양수) [m/s]
%           .zu_dot(4)     - unsprung mass velocity [m/s]
%           .zs(4), .zu(4) - 변위 [m]
%       ctrlState - 내부 상태
%       CTRL      - .VER.cMin (≈ 500), .cMax (≈ 5000), .skyGain (≈ 2500)
%       dt        - sample time
%
%   Output:
%       dampingCmd - 4×1 damping coefficient [Ns/m]
%
%   요구사항:
%       1. Skyhook 기본:  c_i = skyGain · sign(zs_dot_i · (zs_dot_i - zu_dot_i))
%          (또는 force form: F = skyGain · zs_dot, F = c · (zs_dot - zu_dot))
%       2. cMin ≤ c ≤ cMax 제한
%       3. (옵션) Hybrid skyhook + groundhook
%       4. (옵션) body-bounce/wheel-hop 빈도 분리
%
%   힌트:
%       - Skyhook 의 핵심 원리: sprung mass 가 절대 좌표에서 정지하길 원함 → relative
%         damping 을 변조해 sprung velocity 를 줄임.
%       - 간단 force version: 항상 c = c_nom 으로 두고, (zs_dot · (zs_dot - zu_dot)) > 0
%         일 때만 c = cMax, 아니면 c = cMin (semi-active 의 on-off skyhook).

    %% TODO: 학생 구현
    %  (1) skyhook (또는 변형)
    %  (2) per-wheel 적용
    %  (3) cMin/cMax 제한

    % 임시 baseline (반드시 교체) — passive 1500 Ns/m
% 상태 변수 읽기 (4개 바퀴 각각에 대한 배열, 크기: 4x1)
    zs_dot = suspState.zs_dot; % 차체(Sprung mass)의 수직 속도
    zu_dot = suspState.zu_dot; % 바퀴(Unsprung mass)의 수직 속도
    
    % 상대 속도 (서스펜션의 압축/인장 속도)
    v_rel = zs_dot - zu_dot;
    
    % 출력 변수 초기화
    dampingCmd = CTRL.VER.cMin * ones(4, 1);
    
    % 4개 바퀴에 대해 개별적으로 Skyhook 제어 적용
    for i = 1:4
        % 차체의 이동 방향과 서스펜션의 상대 이동 방향이 같을 때
        if (zs_dot(i) * v_rel(i)) > 0
            
            % [핵심 수정 사항] 0으로 나누기(Zero-division) 완벽 방어 로직
            denom = v_rel(i);
            if abs(denom) < 1e-6
                % 값이 0이거나 너무 작으면 강제로 1e-6으로 만들어 폭발 방지
                if denom >= 0
                    denom = 1e-6;
                else
                    denom = -1e-6;
                end
            end
            
            % Continuous Skyhook 요구 감쇠 계수 계산
            c_req = (CTRL.VER.skyGain * zs_dot(i)) / denom;
            
            % 계산된 요구 값이 물리적 한계(cMin ~ cMax)를 벗어나지 않도록 제한
            dampingCmd(i) = max(min(c_req, CTRL.VER.cMax), CTRL.VER.cMin);
            
        else
            dampingCmd(i) = CTRL.VER.cMin;
        end
    end
end