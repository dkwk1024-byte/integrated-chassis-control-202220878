# [202220878-권민수] ICC 제어기 설계 보고서

**과목**: 자동제어 — 2026 봄
**제출일**: 2026-06-22
**팀**: 2인 1팀 (팀원:202220067, 유정원)

---

## 1. 설계 개요 (1 페이지)

본 프로젝트의 목표는 BMW 5-series 기반 14DOF 차량 모델을 대상으로 6개의 표준 주행 시나리오에서 차량 안정성과 제동 성능을 개선하는 것이다. 특히 자동채점에서 사용되는 주요 KPI는 yaw rate 응답, side slip angle, load transfer ratio(LTR), lateral path deviation, stopping distance, ABS slip RMS 등이다. 따라서 본 설계에서는 모든 성능을 한 번에 최적화하기보다, 먼저 차량이 불안정해지지 않도록 yaw 안정성, sideslip 억제, wheel lock 방지를 우선 목표로 설정하였다.

제어기 설계는 복잡한 최적제어기보다는 강의에서 다룬 피드백 제어의 기본 개념을 바탕으로 한 rule-based feedback 구조로 구성하였다. LQR이나 SMC와 같은 방법도 적용 가능하지만, 본 프로젝트의 plant는 14DOF 비선형 차량 모델이고, 시나리오마다 속도, 조향, 제동 조건이 달라진다. 따라서 정확한 선형 상태공간 모델 하나로 전체 구간을 안정적으로 제어하기 어렵다고 판단하였다. 이에 따라 yaw rate 오차, side slip angle, wheel slip ratio와 같이 물리적 의미가 명확한 피드백 신호를 직접 사용하고, saturation과 filtering을 적용하여 시뮬레이션 중 발산이나 actuator command의 급격한 변화를 방지하는 방향으로 설계하였다.

전체 제어기는 네 개의 함수로 분리하였다. `ctrl_lateral.m`에서는 yaw rate tracking을 위한 AFS(active front steering) 보조 조향과 side slip angle을 제한하기 위한 ESC yaw moment를 생성하였다. `ctrl_longitudinal.m`에서는 약한 속도 추종 PI 구조를 포함하되, B1 straight braking 시나리오에서는 wheel slip ratio 기반 ABS relief가 핵심적으로 작동하도록 설계하였다. `ctrl_vertical.m`에서는 sprung mass와 unsprung mass의 상대속도를 이용한 continuous skyhook 방식의 CDC(continuous damping control)를 적용하였다. 마지막으로 `ctrl_coordinator.m`에서는 상위 제어기에서 생성한 steer angle, yaw moment, brake ratio, damping command를 실제 actuator 명령인 조향각, 4륜 brake torque, 4륜 damping coefficient로 변환하였다.

각 제어기 한 줄 요약은 다음과 같다.

* **ctrl_lateral**: yaw rate 오차 기반 AFS 보조 조향 + side slip angle threshold 기반 β-limiter ESC
* **ctrl_longitudinal**: 약한 PI 속도 보정 + wheel slip ratio 기반 ABS brake relief
* **ctrl_vertical**: continuous skyhook 기반 semi-active damping control
* **ctrl_coordinator**: yaw moment를 좌우 brake torque 차동으로 분배하고, ABS를 위해 음수 brake torque relief를 허용하는 actuator allocation

최종적으로 본 설계는 A3 Step Steer, A4 Steady-State Circular, A7 Brake-in-Turn 시나리오에서 만점을 달성하였고, B1 Straight Brake에서는 ABS slip RMS를 목표치 이하로 낮추는 데 성공하였다. 반면 stopping distance와 lateral path deviation은 완전히 개선하지 못했는데, 이는 본 제어기가 path error를 직접 입력으로 사용하지 않고 yaw rate 및 sideslip 안정성 중심으로 설계되었기 때문이라고 판단된다.

---

## 2. 수학적 모델링 (1-2 페이지)

### 2.1 사용한 plant 단순화

본 프로젝트에서 실제 시뮬레이션 및 채점은 BMW 5-series 기반의 14DOF 차량 모델에서 수행되었다. 14DOF 모델은 차량의 종방향, 횡방향, yaw motion뿐 아니라 roll, pitch, heave, 각 휠의 동역학과 서스펜션 거동까지 포함하므로 실제 차량 거동에 가까운 검증 환경이라고 볼 수 있다. 그러나 이러한 고차 비선형 모델을 그대로 사용하여 제어기를 설계하면 모델 파라미터가 많고, 시나리오별 속도와 조향 조건에 따라 동특성이 크게 달라지기 때문에 단순한 gain 선정이 어렵다.

따라서 본 설계에서는 제어기 구조를 정하기 위해 vehicle dynamics에서 가장 기본적으로 사용되는 단순화된 bicycle model을 기준으로 하였다. Bicycle model은 좌우 바퀴를 하나의 등가 앞바퀴와 뒷바퀴로 묶어서 표현하며, 횡방향 속도와 yaw rate를 주요 상태로 둔다. 이 모델을 사용하면 조향 입력이 yaw rate와 side slip angle에 어떤 영향을 주는지 직관적으로 파악할 수 있고, AFS와 ESC 설계에 필요한 yaw rate error 및 sideslip feedback 구조를 간단하게 만들 수 있다.

종방향 제어에서는 차량 전체 질량을 하나의 lumped mass로 보고, 종방향 힘과 가속도 사이의 관계를 단순히 (F_x = ma_x)로 근사하였다. 하지만 B1 straight braking 시나리오에서는 속도 추종보다 wheel lock 방지가 더 중요하므로, 최종 설계에서는 wheel slip ratio를 기준으로 brake torque를 줄이는 ABS relief 구조를 사용하였다. 수직방향 제어는 quarter-car model의 개념을 이용하여 sprung mass velocity와 unsprung mass velocity의 차이로 suspension relative velocity를 계산하고, 이를 기반으로 skyhook damping을 구현하였다.

즉 본 프로젝트에서는 실제 plant는 14DOF 모델을 사용하여 검증하되, 제어기 설계와 gain 선정은 bicycle model, lumped longitudinal model, quarter-car suspension model의 단순화된 물리 모델을 바탕으로 수행하였다. 이러한 단순화는 모델 정확도는 낮추지만, 제어 구조를 직관적으로 만들고 시뮬레이션 중 actuator saturation과 runtime error를 방지하는 데 유리하다.

### 2.2 State-space 표현


횡방향 제어기 설계를 위해 가장 기본적인 bicycle model을 사용하였다. 상태 변수는 차량의 횡방향 속도와 yaw rate로 두었고, 입력은 전륜 조향각으로 정의하였다.

$$
x_{lat} = [v_y,\ r]^T,\qquad u_{lat} = \delta
$$

여기서 (v_y)는 차량 무게중심의 횡방향 속도이고, (r)은 yaw rate이다. (\delta)는 전륜 조향각이다. 일정한 종방향 속도 (V_x)와 선형 타이어 모델을 가정하면, bicycle model은 다음과 같은 2차 상태방정식으로 표현할 수 있다.

$$
\dot{x}*{lat} = A*{lat}x_{lat} + B_{lat}u_{lat}
$$

이를 성분별로 쓰면 다음과 같다.

$$
\dot{v}_y =
-\frac{C_f+C_r}{mV_x}v_y
+\left(\frac{l_rC_r-l_fC_f}{mV_x}-V_x\right)r
+\frac{C_f}{m}\delta
$$

$$
\dot{r} =
\frac{l_rC_r-l_fC_f}{I_zV_x}v_y
-\frac{l_f^2C_f+l_r^2C_r}{I_zV_x}r
+\frac{l_fC_f}{I_z}\delta
$$

따라서 상태공간 행렬의 각 항은 다음과 같이 정의된다.

$$
A_{11} = -\frac{C_f+C_r}{mV_x}
$$

$$
A_{12} = \frac{l_rC_r-l_fC_f}{mV_x}-V_x
$$

$$
A_{21} = \frac{l_rC_r-l_fC_f}{I_zV_x}
$$

$$
A_{22} = -\frac{l_f^2C_f+l_r^2C_r}{I_zV_x}
$$

$$
B_1 = \frac{C_f}{m},\qquad B_2 = \frac{l_fC_f}{I_z}
$$

여기서 (m)은 차량 질량, (I_z)는 yaw moment of inertia, (l_f)와 (l_r)은 각각 차량 무게중심에서 전륜축과 후륜축까지의 거리이다. 또한 (C_f)와 (C_r)은 전륜 및 후륜 cornering stiffness이다. 이 모델을 통해 조향 입력이 yaw rate와 side slip angle에 미치는 영향을 확인할 수 있으며, 본 설계에서는 yaw rate tracking을 위한 AFS 보조 조향과 side slip angle 제한을 위한 ESC 설계에 이 관계를 사용하였다.

출력은 yaw rate와 side slip angle을 중심으로 보았다. Side slip angle은 소각도 조건에서 다음과 같이 근사할 수 있다.

$$
\beta \approx \frac{v_y}{V_x}
$$

따라서 출력은 다음과 같이 정의할 수 있다.

$$
y_1 = r
$$

$$
y_2 = \beta \approx \frac{v_y}{V_x}
$$

종방향 제어에서는 차량 전체를 하나의 질량으로 보는 lumped mass model을 사용하였다. 종방향 힘 (F_x)와 차량 속도 (v_x) 사이의 관계는 다음과 같이 근사하였다.

$$
m\dot{v}_x = F_x
$$

따라서 종방향 모델은 다음과 같이 단순화할 수 있다.

$$
x_{lon} = v_x,\qquad u_{lon} = F_x
$$

$$
\dot{x}*{lon} = \frac{1}{m}u*{lon}
$$

$$
y_{lon} = x_{lon}
$$

다만 실제 최종 코드에서는 B1 straight braking 시나리오에서 단순 속도 추종보다 wheel lock 방지가 더 중요하다고 판단하였다. 따라서 종방향 제어는 위의 (F_x = ma_x) 관계를 기본 개념으로 사용하되, 최종적으로는 wheel slip ratio를 이용한 ABS relief 구조를 적용하였다.

수직방향 제어에서는 각 바퀴를 독립적인 quarter-car suspension으로 근사하였다. Sprung mass velocity를 (\dot{z}_s), unsprung mass velocity를 (\dot{z}_u)라고 하면 suspension relative velocity는 다음과 같다.

$$
v_{rel} = \dot{z}_s - \dot{z}_u
$$

Skyhook damping의 기본 목표는 sprung mass의 절대 속도 (\dot{z}_s)를 줄이는 것이다. 본 설계에서는 다음 조건을 기준으로 damping coefficient를 조절하였다.

$$
\dot{z}_s(\dot{z}_s-\dot{z}_u) > 0
$$

위 조건이 만족되면 차체 운동을 줄이기 위해 높은 damping을 적용하고, 그렇지 않으면 낮은 damping을 적용한다. 실제 구현에서는 continuous skyhook 형태로 요구 감쇠계수를 계산한 뒤, (c_{min})과 (c_{max}) 사이로 제한하였다.

$$
c_{req} = \frac{K_{sky}\dot{z}_s}{\dot{z}_s-\dot{z}_u}
$$

$$
c_i = \mathrm{sat}(c_{req},\ c_{min},\ c_{max})
$$

정리하면, 본 설계에서는 횡방향 제어에는 bicycle model의 (v_y-r) 상태방정식을 사용하였고, 종방향 제어에는 lumped mass model, 수직방향 제어에는 quarter-car 기반 skyhook 개념을 사용하였다. 최종 검증은 14DOF plant에서 수행하였지만, 제어기 설계는 위와 같은 단순화된 모델을 바탕으로 진행하였다.



### 2.3 가정 + 한계

본 설계에서는 14DOF 차량 모델을 직접 제어기 설계에 모두 사용하지 않고, 제어 목적에 따라 단순화된 모델을 사용하였다. 이 과정에서 다음과 같은 가정을 두었다.

첫째, 횡방향 제어기 설계에서는 종방향 속도 (V_x)가 짧은 시간 동안 일정하다고 가정하였다. 실제 시뮬레이션에서는 제동이나 조향에 따라 속도가 변하지만, AFS와 ESC의 기본 구조를 설계할 때는 일정 속도에서의 bicycle model을 기준으로 yaw rate와 side slip angle의 관계를 해석하였다. 이 가정 덕분에 yaw rate error를 이용한 보조 조향과 side slip angle threshold 기반 ESC yaw moment를 단순하게 구현할 수 있었다.

둘째, 타이어는 선형 영역에서 동작한다고 가정하였다. Bicycle model에서는 전륜과 후륜 lateral force가 slip angle에 비례한다고 보고 cornering stiffness (C_f), (C_r)로 표현하였다. 그러나 실제 14DOF plant에서는 큰 조향각, 급제동, 복합 조향-제동 상황에서 타이어가 비선형 포화 영역에 들어갈 수 있다. 따라서 본 설계에서는 정밀한 타이어 force 예측보다는 side slip angle과 wheel slip ratio가 커질 때 제어입력을 제한하는 방식으로 안정성을 확보하였다.

셋째, 종방향 제어에서는 차량을 하나의 질량으로 보고 (F_x = ma_x) 관계를 사용하였다. 하지만 최종 코드에서는 단순 속도 추종보다 wheel lock 방지가 더 중요하다고 판단하여, `ctrlState.wheelSlip`에 저장되는 wheel slip ratio를 이용해 ABS relief를 구현하였다. 즉, wheel slip이 커지면 `brakeRatio`를 음수로 만들어 기존 시나리오 brake torque를 줄이는 방식으로 제동 안정성을 확보하였다. 이 구조는 B1 시나리오에서 `absSlipRMS`를 목표값 이하로 낮추는 데 효과적이었지만, brake torque를 보수적으로 줄이는 과정에서 stopping distance가 충분히 감소하지 못하는 한계가 있었다.

넷째, 수직방향 제어에서는 각 바퀴를 독립적인 quarter-car model로 보았다. 실제 차량에서는 roll, pitch, heave motion이 서로 결합되어 있고, 좌우 및 전후 suspension 거동이 완전히 독립적이지 않다. 하지만 본 설계에서는 각 바퀴의 sprung mass velocity와 unsprung mass velocity를 이용해 개별 damping coefficient를 계산하였다. 이 방식은 구현이 단순하고 안정적이지만, LTR을 직접 피드백하지 않기 때문에 A1과 D1 시나리오에서 load transfer ratio를 완전히 목표값 이하로 낮추는 데에는 한계가 있었다.

마지막으로, 본 제어기는 path error를 직접 입력으로 사용하지 않는다. `ctrl_lateral.m`은 yaw rate와 side slip angle만을 사용하여 AFS와 ESC 명령을 생성한다. 따라서 차량의 yaw 안정성과 sideslip 억제에는 효과적이었지만, A1과 D1 시나리오에서 lateral path deviation을 직접 줄이는 데에는 한계가 있었다. 최종 결과에서도 sideSlipMax는 대부분 목표를 만족했지만, lateralDevMax는 목표값에 도달하지 못하였다. 만약 더 개선한다면 path deviation 또는 heading error를 입력으로 사용하는 path-following 보상기를 추가하는 것이 필요하다.


## 3. 제어기 설계 (3-4 페이지)

### 3.1 ctrl_lateral — AFS + ESC

`ctrl_lateral.m`의 목표는 yaw rate tracking과 side slip angle 제한이다. A3 Step Steer 시나리오에서는 yaw rate overshoot, rise time, settling time이 주요 KPI로 사용되므로 목표 yaw rate를 빠르게 추종하는 것이 중요하다. 반면 A1, A7, D1과 같이 급조향 또는 제동이 포함된 시나리오에서는 side slip angle이 커지면 차량이 불안정해질 수 있으므로, 일정 threshold 이상에서 ESC yaw moment를 생성하도록 하였다.

본 설계에서는 복잡한 LQR이나 SMC 대신 yaw rate error 기반의 proportional feedback 구조를 사용하였다. LQR은 bicycle model의 정확한 파라미터와 속도별 gain scheduling이 필요하고, SMC는 chattering 및 actuator saturation 문제가 발생할 수 있다고 판단하였다. 따라서 구현 안정성과 튜닝 편의성을 우선하여 다음과 같은 단순한 feedback 구조를 선택하였다.

Yaw rate error는 다음과 같이 정의하였다.

$$
e_r = r_{ref} - r
$$

여기서 (r_{ref})는 목표 yaw rate이고, (r)은 실제 yaw rate이다. AFS 보조 조향각은 yaw rate error에 비례하도록 설계하였다.

$$
\delta_{target} = K_r e_r
$$

최종 코드에서는 (K_r = 0.5)를 사용하였다. 다만 yaw rate error가 순간적으로 변할 때 조향 명령이 급격하게 튀면 차량 응답이 불안정해질 수 있으므로, 1차 low-pass filter를 적용하였다.

$$
\delta_f(k) = 0.8\delta_f(k-1) + 0.2\delta_{target}(k)
$$

그 후 최종 AFS 보조 조향각은 조향각 제한을 넘지 않도록 saturation하였다.

$$
\delta_{AFS} = \mathrm{sat}(\delta_f,\ -\delta_{max},\ \delta_{max})
$$

이 방식은 PID 제어기 중 P 제어와 유사한 구조이며, integral term은 사용하지 않았다. Integral term은 steady-state error를 줄이는 데 유리하지만, 급격한 조향 시나리오에서는 wind-up으로 인해 overshoot가 증가할 수 있다고 판단하였다. 또한 derivative term은 noise에 민감할 수 있으므로 사용하지 않고, 대신 filtered steering command를 사용하여 급격한 입력 변화를 완화하였다.

Side slip angle 제한을 위해 ESC yaw moment를 추가하였다. Side slip angle threshold는 다음과 같이 설정하였다.

$$
\beta_{th} = 2.5^\circ
$$

차체 side slip angle의 절댓값이 threshold보다 작으면 ESC yaw moment를 발생시키지 않고, threshold를 초과하면 slip을 줄이는 방향으로 yaw moment를 생성하였다.

$$
M_z =
-K_{\beta}\ \mathrm{sign}(\beta)(|\beta|-\beta_{th})
$$

최종 설계에서는 (K_{\beta}=30000)을 사용하였다. 여기서 음의 부호는 side slip angle이 커지는 방향과 반대 방향의 yaw moment를 인가하기 위한 것이다. 이 구조는 차량이 급격히 미끄러지는 상황에서 yaw 안정성을 높이는 역할을 한다.

최종적으로 `ctrl_lateral.m`에서 사용한 주요 값은 다음과 같다.

```matlab
% yaw rate tracking
K_yaw = 0.5;
alpha_steer_prev = 0.8;
alpha_steer_new  = 0.2;

% beta limiter ESC
beta_th = deg2rad(2.5);
K_beta  = 30000;

% AFS command
error_yaw = yawRateRef - yawRate;
steer_target = K_yaw * error_yaw;
filteredSteer = 0.8 * filteredSteer + 0.2 * steer_target;
deltaAdd.steerAngle = sat(filteredSteer, -LIM.MAX_STEER_ANGLE, LIM.MAX_STEER_ANGLE);

% ESC yaw moment
if abs(slipAngle) > beta_th
    deltaAdd.yawMoment = -K_beta * sign(slipAngle) * (abs(slipAngle) - beta_th);
else
    deltaAdd.yawMoment = 0;
end
```

이 설계의 장점은 구조가 단순하고 actuator saturation 처리가 쉽다는 점이다. 또한 AFS는 yaw rate 응답을 빠르게 만들고, ESC는 side slip angle이 커지는 비상 상황에서만 개입하므로 두 제어 기능이 서로 다른 역할을 담당한다. 최종 결과에서 A3 Step Steer 시나리오는 yawRateOvershoot, yawRateRiseTime, yawRateSettling 세 KPI를 모두 만족하였고, A7 Brake-in-Turn 시나리오에서도 sideSlipMax와 LTR_max를 모두 목표값 이하로 유지하였다.


### 3.2 ctrl_longitudinal — 속도 + ABS

`ctrl_longitudinal.m`의 목표는 종방향 속도 제어와 ABS 기능을 함께 구현하는 것이다. 일반적인 종방향 제어에서는 목표 속도 (v_{x,ref})와 실제 속도 (v_x)의 차이를 이용하여 구동력 또는 제동력을 계산할 수 있다. 그러나 본 프로젝트의 B1 Straight Brake 시나리오에서는 단순한 속도 추종보다 wheel lock을 방지하는 것이 더 중요한 문제였다. 실제 baseline 결과에서도 `absSlipRMS`가 크게 나타났기 때문에, 최종 설계에서는 wheel slip ratio를 이용한 ABS relief 구조를 핵심으로 사용하였다.

종방향 속도 오차는 다음과 같이 정의하였다.

$$
e_v = v_{x,ref} - v_x
$$

기본적인 종방향 힘 명령은 PI 제어 형태로 계산할 수 있다.

$$
F_{x,cmd} = m(K_p e_v + K_i \int e_v dt)
$$

본 코드에서는 이 구조를 포함하되, 실제 최종 출력에서는 큰 추가 종방향 힘을 직접 사용하지 않았다. 그 이유는 B1 시나리오에서 이미 scenario brake command가 주어지고, 본 제어기는 그 제동 명령에 대한 보정값을 주는 구조로 동작하기 때문이다. 따라서 `forceCmd.Fx_total`은 최종적으로 0으로 두고, wheel slip ratio가 커지는 경우 `forceCmd.brakeRatio`를 음수로 만들어 기존 brake torque를 줄이는 방식으로 ABS를 구현하였다.

ABS 제어의 핵심 피드백 변수는 wheel slip ratio이다. Slip ratio는 일반적으로 다음과 같이 정의된다.

$$
\kappa = \frac{\omega r_w - v_x}{\max(v_x,\ 0.1)}
$$

여기서 (\omega)는 wheel angular speed이고, (r_w)는 wheel radius이다. 본 코드에서는 runner에서 `ctrlState.wheelSlip`에 저장되는 네 바퀴의 wheel slip ratio를 사용하였다. 네 바퀴 중 가장 큰 slip을 기준으로 ABS 개입 여부를 판단하였다.

$$
\kappa_{max} = \max(|\kappa_{FL}|,\ |\kappa_{FR}|,\ |\kappa_{RL}|,\ |\kappa_{RR}|)
$$

제동 중이거나 wheel slip이 일정 값 이상 커졌을 때 ABS가 작동하도록 하였고, slip이 커질수록 더 큰 brake relief를 적용하였다. 최종적으로 사용한 relief command는 다음과 같은 rule-based 구조이다.

```matlab id="tvv01a"
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
```

여기서 `reliefCmd`는 brake torque를 줄이는 정도를 의미한다. 값이 클수록 기존 제동 명령을 더 많이 줄인다. Slip이 갑자기 증가할 때 brake command가 급격히 변하면 차량 응답이 불안정해질 수 있으므로, relief command에도 1차 필터를 적용하였다.

$$
R_f(k) = (1-\alpha)R_f(k-1) + \alpha R(k)
$$

본 설계에서는 relief command가 증가할 때는 (\alpha = 0.75), 감소할 때는 (\alpha = 0.55)를 사용하였다. 즉, wheel lock 위험이 커질 때는 빠르게 brake를 줄이고, slip이 안정화되면 비교적 빠르게 brake를 회복하도록 하였다.

최종적으로 `ctrl_longitudinal.m`에서 출력하는 brake ratio는 다음과 같이 정의하였다.

$$
brakeRatio = -R_f
$$

음수 부호를 사용한 이유는 `ctrl_coordinator.m`에서 이 값을 기존 시나리오 brake torque를 줄이는 relief command로 해석하기 위해서이다. 실제 최종 brake torque는 runner 내부에서 scenario brake command와 coordinator의 brake torque correction이 합쳐져 계산된다. 따라서 `brakeRatio < 0`이면 기존 brake torque가 감소하고, wheel lock이 완화된다.

최종 코드의 핵심 구조는 다음과 같다.

```matlab id="y3ms4f"
forceCmd.Fx_total = 0;

slipMax = max(abs(ctrlState.wheelSlip));

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

ctrlState.absReliefFilt = (1-alpha)*ctrlState.absReliefFilt + alpha*reliefCmd;
forceCmd.brakeRatio = -ctrlState.absReliefFilt;
```

이 설계는 stopping distance를 최우선으로 줄이는 방식이 아니라, wheel lock을 방지하여 제동 안정성을 확보하는 방향으로 설계되었다. 최종 결과에서 B1 Straight Brake 시나리오의 `absSlipRMS`는 목표값 0.1 이하인 0.0899로 감소하여 해당 KPI에서 만점을 얻었다. 반면 brake relief가 보수적으로 작동하면서 stopping distance는 70.11 m로 남아 목표값 40 m에는 도달하지 못하였다. 따라서 본 종방향 제어기는 ABS slip 억제에는 효과적이었지만, 제동거리와 wheel slip 안정성 사이의 trade-off가 존재한다고 분석하였다.


### 3.3 ctrl_vertical — CDC (있다면)

`ctrl_vertical.m`의 목표는 각 바퀴의 suspension damping coefficient를 조절하여 차체의 수직 운동을 줄이는 것이다. 본 프로젝트에서는 CDC(Continuous Damping Control)를 구현하기 위해 skyhook damping 개념을 사용하였다. Skyhook 제어의 기본 아이디어는 차체가 절대좌표계에서 가능한 정지해 있도록 감쇠력을 조절하는 것이다. 즉, sprung mass velocity가 클 때 이를 줄이는 방향으로 damping coefficient를 증가시킨다.

수직방향 제어에서 사용한 주요 변수는 sprung mass velocity와 unsprung mass velocity이다.

$$
\dot{z}_s = \text{sprung mass velocity}
$$

$$
\dot{z}_u = \text{unsprung mass velocity}
$$

Suspension relative velocity는 다음과 같이 계산하였다.

$$
v_{rel} = \dot{z}_s - \dot{z}_u
$$

Skyhook 제어에서는 차체 속도와 suspension relative velocity의 곱을 이용하여 damping 증가 여부를 판단할 수 있다.

$$
\dot{z}*s v*{rel} > 0
$$

위 조건이 만족되면 suspension damping force가 차체 운동을 줄이는 방향으로 작용할 수 있으므로, 더 큰 damping coefficient를 적용한다. 반대로 조건이 만족되지 않으면 불필요한 감쇠력으로 ride comfort가 나빠질 수 있으므로 낮은 damping coefficient를 사용한다.

본 설계에서는 단순한 on-off skyhook이 아니라 continuous skyhook 형태로 요구 감쇠계수를 계산하였다. 요구 감쇠계수는 다음과 같이 계산하였다.

$$
c_{req} = \frac{K_{sky}\dot{z}*s}{v*{rel}}
$$

여기서 (K_{sky})는 skyhook gain이고, 실제 코드에서는 `CTRL.VER.skyGain` 값을 사용하였다. 계산된 감쇠계수는 물리적 actuator 제한을 고려하여 `cMin`과 `cMax` 사이로 제한하였다.

$$
c_i = \mathrm{sat}(c_{req},\ c_{min},\ c_{max})
$$

최종 코드에서는 각 바퀴에 대해 독립적으로 위 연산을 수행하였다. 따라서 출력 `dampingCmd`는 4x1 vector 형태이며, 순서는 `[FL; FR; RL; RR]`이다. 또한 `v_rel`이 0에 가까운 경우에는 나눗셈으로 인해 값이 발산할 수 있으므로, zero-division 방어 로직을 추가하였다. 이 부분은 시뮬레이션 중 NaN 또는 Inf가 발생하여 자동채점이 실패하는 것을 방지하기 위한 안정화 처리이다.

최종 구현의 핵심 구조는 다음과 같다.

```matlab id="axu7mb"
zs_dot = suspState.zs_dot;
zu_dot = suspState.zu_dot;

v_rel = zs_dot - zu_dot;

dampingCmd = CTRL.VER.cMin * ones(4, 1);

for i = 1:4
    if (zs_dot(i) * v_rel(i)) > 0

        denom = v_rel(i);

        if abs(denom) < 1e-6
            if denom >= 0
                denom = 1e-6;
            else
                denom = -1e-6;
            end
        end

        c_req = (CTRL.VER.skyGain * zs_dot(i)) / denom;

        dampingCmd(i) = max(min(c_req, CTRL.VER.cMax), CTRL.VER.cMin);

    else
        dampingCmd(i) = CTRL.VER.cMin;
    end
end
```

이 설계는 차체 수직속도가 suspension relative velocity와 같은 방향으로 움직일 때 높은 감쇠를 적용하여 차체 운동을 줄이고, 그렇지 않은 경우에는 최소 감쇠를 적용한다. 따라서 ride comfort와 handling 안정성 사이의 균형을 맞추는 semi-active damping 구조라고 볼 수 있다.

다만 본 vertical controller는 LTR을 직접 입력으로 사용하지 않는다. 즉, 좌우 하중이동 자체를 직접 피드백하는 roll control 구조는 아니며, 각 바퀴의 수직 속도 정보만으로 damping coefficient를 조절한다. 따라서 A1과 D1 시나리오에서 LTR_max를 일부 개선하는 데에는 한계가 있었다. 최종 결과에서도 A1과 D1의 LTR 항목은 부분 점수를 받았지만, 목표값 0.6 이하로 완전히 낮추지는 못하였다. 이 한계는 추후 roll angle, roll rate 또는 LTR 추정값을 feedback으로 사용하는 vertical control을 추가하면 개선할 수 있을 것으로 판단된다.


### 3.4 ctrl_coordinator — Actuator Allocation

`ctrl_coordinator.m`의 역할은 상위 제어기에서 계산한 횡방향, 종방향, 수직방향 명령을 실제 actuator command로 변환하는 것이다. 본 프로젝트에서 최종 actuator command는 조향각, 4륜 brake torque, 4륜 damping coefficient로 구성된다.

```matlab
actuatorCmd.steerAngle
actuatorCmd.brakeTorque
actuatorCmd.dampingCoeff
```

Coordinator는 `ctrl_lateral.m`에서 생성된 AFS 조향각과 ESC yaw moment, `ctrl_longitudinal.m`에서 생성된 brakeRatio 및 Fx_total, `ctrl_vertical.m`에서 생성된 damping command를 하나로 통합한다. 본 설계에서는 runtime error를 방지하기 위해 먼저 모든 출력값을 기본값으로 초기화하고, 입력 구조체에 필요한 field가 없는 경우 0 또는 기본값을 넣는 방어 로직을 추가하였다. 초기 코드에서는 `lonCmd.Fx_total` field가 없는 경우 runtime error가 발생하여 모든 시나리오가 0점 처리되었기 때문에, 최종 코드에서는 field checking을 가장 먼저 수행하였다.

조향 명령은 lateral controller에서 계산한 `latCmd.steerAngle`을 그대로 전달하되, actuator limit을 넘지 않도록 saturation하였다.

$$
\delta_{cmd} = \mathrm{sat}(\delta_{AFS},\ -\delta_{max},\ \delta_{max})
$$

종방향 brake command는 두 가지 역할을 하도록 설계하였다. 첫째, `brakeRatio > 0`인 경우에는 추가 제동을 의미하며, 전후 brake torque를 60:40 비율로 분배하였다. 둘째, `brakeRatio < 0`인 경우에는 ABS relief를 의미하며, 기존 시나리오 brake torque를 줄이기 위한 음수 brake torque correction을 생성하였다. 이 구조가 중요한 이유는 최종 brake torque가 runner 내부에서 scenario brake command와 coordinator output을 더해서 계산되기 때문이다. 따라서 음수 brake torque correction을 허용해야 wheel lock 상황에서 기존 brake를 줄일 수 있다.

최종 코드에서 brakeRatio에 따른 종방향 brake correction은 다음과 같이 구현하였다.

```matlab
if brakeRatio > 0
    T_lon = brakeRatio * maxBrake * [0.60; 0.60; 0.40; 0.40];

elseif brakeRatio < 0
    relief = abs(brakeRatio);
    T_lon = -relief * maxBrake * [1.00; 1.00; 0.85; 0.85];
end
```

위 식에서 `[FL; FR; RL; RR]` 순서를 사용하였다. 양수 brakeRatio에서는 전륜에 60%, 후륜에 40% 비율로 제동을 배분하였다. 반대로 ABS relief 상황에서는 네 바퀴 모두 brake torque를 줄이되, lock이 발생하기 쉬운 전륜의 brake torque를 조금 더 많이 줄이도록 하였다.

ESC yaw moment는 좌우 brake torque 차이를 이용해 구현하였다. Yaw moment (M_z)가 주어졌을 때, 좌우 brake torque 차이는 track width와 wheel radius를 이용해 근사할 수 있다. Brake force는 brake torque를 wheel radius로 나눈 값이므로, 좌우 제동력 차이에 의한 yaw moment는 다음과 같이 표현할 수 있다.

$$
M_z \approx \frac{\Delta T}{r_w}t
$$

따라서 필요한 brake torque 차이는 다음과 같이 근사된다.

$$
\Delta T \approx \frac{M_z r_w}{t}
$$

본 설계에서는 yaw moment를 전륜과 후륜에 70:30 비율로 나누어 분배하였다.

$$
\Delta T_f = 0.70\frac{M_z r_w}{t}
$$

$$
\Delta T_r = 0.30\frac{M_z r_w}{t}
$$

이를 4륜 brake torque correction으로 쓰면 다음과 같다.

```matlab
dT_f = 0.70 * Mz * r_w / track;
dT_r = 0.30 * Mz * r_w / track;

T_esc = [ dT_f;
         -dT_f;
          dT_r;
         -dT_r ];
```

즉 양의 yaw moment가 필요할 때는 좌측 brake torque를 증가시키고 우측 brake torque를 감소시키는 방식으로 좌우 비대칭 제동을 만든다. 이 차동 제동 방식은 별도의 yaw moment actuator 없이도 ESC 효과를 만들 수 있다는 장점이 있다.

최종 brake torque command는 종방향 brake correction, Fx_total 기반 추가 제동, ESC yaw moment 기반 차동 제동을 모두 더하여 계산하였다.

$$
T_{total} = T_{lon} + T_{Fx} + T_{ESC}
$$

최종적으로 command가 너무 커지는 것을 막기 위해 다음과 같이 제한하였다.

$$
T_{cmd} = \mathrm{sat}(T_{total},\ -T_{max},\ T_{max})
$$

여기서 중요한 점은 coordinator 내부에서는 brake torque를 0 이상으로만 제한하지 않았다는 것이다. `brakeRatio < 0`일 때 생성되는 음수 brake torque correction이 ABS relief 역할을 하기 때문이다. 실제 물리적인 최종 brake torque는 runner 내부에서 scenario brake command와 더해진 뒤 0과 `MAX_BRAKE_TRQ` 사이로 다시 제한된다.

수직방향 damping command는 `ctrl_vertical.m`에서 생성된 `verCmd`를 4x1 vector로 정리한 뒤, `cMin`과 `cMax` 사이로 제한하여 전달하였다. 따라서 coordinator는 lateral, longitudinal, vertical controller의 명령을 통합하면서 actuator saturation과 field checking을 함께 수행하는 안전 계층의 역할도 한다.

최종적으로 본 coordinator 설계는 두 가지 점에서 중요했다. 첫째, field checking과 기본값 초기화를 통해 runtime error를 제거하였다. 둘째, 음수 brake torque correction을 허용하여 ABS relief가 실제 brake command에 반영되도록 하였다. 이 수정 이후 B1 시나리오에서 `absSlipRMS`가 목표값 이하로 낮아져 해당 KPI에서 만점을 달성하였다.

## 4. 시뮬레이션 결과 (2-3 페이지)

### 4.1 P1 시나리오 benchmark — 베이스라인 vs 본인 설계

최종 제어기 성능은 `run('scripts/grade.m')`를 실행하여 확인하였다. 자동채점 결과, 본 설계의 정량 점수는 **56.95 / 70.00점**으로 계산되었다. 비율로는 약 **81.4%**에 해당한다. Runtime error나 deduction은 발생하지 않았으며, 모든 P1 시나리오가 정상적으로 실행되었다.

| 시나리오              | KPI                  |   OFF | ON (본인) |       목표값 |       점수 |
| ----------------- | -------------------- | ----: | ------: | --------: | -------: |
| A3 Step Steer     | yawRateOvershoot [%] |  2.81 |  2.4812 |    ≤ 10.0 |    4 / 4 |
| A3 Step Steer     | yawRateRiseTime [s]  |     - |  0.0650 |  ≤ 0.3000 |    4 / 4 |
| A3 Step Steer     | yawRateSettling [s]  |     - |  0.7250 |  ≤ 0.8000 |    4 / 4 |
| A1 DLC            | sideSlipMax [deg]    |  4.51 |  2.6749 |  ≤ 3.0000 |    6 / 6 |
| A1 DLC            | LTR_max              | 0.948 |  0.7761 |  ≤ 0.6000 | 3.53 / 5 |
| A1 DLC            | lateralDevMax [m]    |     - |  2.1668 |  ≤ 0.7000 |    0 / 4 |
| A4 SS Circular    | understeerGradient   |     - |  0.0007 |  0.003 기준 |    5 / 5 |
| A4 SS Circular    | sideSlipMax [deg]    |     - |  1.1791 |  ≤ 2.0000 |    5 / 5 |
| A7 Brake-in-Turn  | sideSlipMax [deg]    |  46.3 |  2.0452 |  ≤ 5.0000 |    8 / 8 |
| A7 Brake-in-Turn  | LTR_max              | 0.745 |  0.3318 |  ≤ 0.7000 |    7 / 7 |
| B1 Straight Brake | stoppingDistance [m] |  72.4 | 70.1145 | ≤ 40.0000 |    0 / 5 |
| B1 Straight Brake | absSlipRMS           |  0.73 |  0.0899 |  ≤ 0.1000 |    5 / 5 |
| D1 DLC + Brake    | sideSlipMax [deg]    |  7.65 |  3.4280 |  ≤ 4.0000 |    4 / 4 |
| D1 DLC + Brake    | LTR_max              |     - |  0.7760 |  ≤ 0.6000 | 1.41 / 2 |
| D1 DLC + Brake    | lateralDevMax [m]    |     - |  2.1668 |  ≤ 1.0000 |    0 / 2 |

최종 결과에서 A3, A4, A7 시나리오는 모든 KPI에서 만점을 받았다. 특히 A3 Step Steer에서는 yaw rate overshoot, rise time, settling time이 모두 목표 조건을 만족하였다. 이는 `ctrl_lateral.m`의 yaw rate error 기반 AFS 보조 조향이 step steer 응답을 안정적으로 만든 결과로 볼 수 있다.

A7 Brake-in-Turn 시나리오에서는 sideSlipMax가 2.0452 deg, LTR_max가 0.3318로 나타나 각각 목표값 5 deg와 0.7을 충분히 만족하였다. 이는 side slip angle이 threshold를 넘을 때 ESC yaw moment를 생성하고, coordinator에서 이를 좌우 brake torque 차동으로 분배한 효과로 판단된다.

B1 Straight Brake에서는 stoppingDistance가 70.1145 m로 목표값 40 m에는 도달하지 못하였다. 그러나 absSlipRMS는 0.0899로 목표값 0.1 이하를 만족하여 해당 KPI에서 만점을 받았다. 이는 `ctrl_longitudinal.m`에서 wheel slip ratio 기반 ABS relief를 적용하고, `ctrl_coordinator.m`에서 음수 brake torque correction을 허용하여 wheel lock을 줄인 결과이다. 다만 brake relief가 보수적으로 작동하면서 제동거리가 충분히 짧아지지 못하는 trade-off가 발생하였다.

A1과 D1 시나리오에서는 sideSlipMax는 목표를 만족했지만, lateralDevMax가 각각 2.1668 m로 나타나 목표값을 만족하지 못하였다. 본 제어기는 yaw rate와 side slip angle을 중심으로 설계되었고, path error나 heading error를 직접 입력으로 사용하지 않았다. 따라서 차량의 yaw 안정성은 확보되었지만, reference path를 정밀하게 따라가는 lateral path deviation 개선에는 한계가 있었다.

### 4.2 핵심 plot — A1 DLC

A1 DLC(Double Lane Change) 시나리오는 차량이 급격한 차선 변경을 수행하는 조건이므로, 횡방향 안정성과 path-following 성능을 동시에 확인할 수 있는 대표 시나리오이다. 본 설계에서는 A1 시나리오에서 sideSlipMax를 목표값 이하로 낮추는 데 성공했지만, lateralDevMax는 목표값을 만족하지 못하였다. 따라서 A1 DLC를 핵심 시나리오로 선정하여 trajectory와 yaw response를 함께 분석하였다.

![A1 trajectory comparison](figures/a1_trajectory.png)

*Figure 4.1 — A1 ISO 3888-1 DLC trajectory comparison. Controller off, controller on, reference path를 비교한다.*

![A1 yaw rate](figures/a1_yawrate.png)

*Figure 4.2 — A1 yaw rate response. Controller on/off 조건에서 yaw rate 응답을 비교한다.*

A1 DLC에서 최종 제어기 적용 후 sideSlipMax는 2.6749 deg로 나타났고, 목표값 3 deg 이하를 만족하였다. 이는 `ctrl_lateral.m`의 β-limiter가 side slip angle이 커질 때 ESC yaw moment를 발생시켜 차량의 yaw 안정성을 확보했기 때문으로 판단된다. 또한 `ctrl_coordinator.m`에서 해당 yaw moment를 좌우 brake torque 차동으로 변환하여 실제 actuator 명령으로 반영하였다.

반면 lateralDevMax는 2.1668 m로 목표값 0.7 m를 만족하지 못하였다. 이는 본 제어기가 yaw rate와 side slip angle을 중심으로 설계되었고, path error 또는 heading error를 직접 feedback으로 사용하지 않았기 때문이다. 즉, 차량의 자세 안정성은 개선되었지만 reference path를 정밀하게 따라가는 path-following 성능은 충분히 개선되지 않았다. 이 결과는 추후 개선 방향으로 path deviation feedback 또는 Stanley/Pure Pursuit 보상 항을 추가할 필요가 있음을 보여준다.

아래 코드는 A1 DLC 시나리오의 trajectory plot을 생성하기 위한 예시이다.

```matlab
if ~exist('docs/figures', 'dir')
    mkdir('docs/figures');
end

[r_off, k_off] = run_icc_scenario('A1','14dof','Controller','off','SavePlot',false);
[r_on,  k_on ] = run_icc_scenario('A1','14dof','Controller','on', 'SavePlot',false);

figure;
plot(r_off.x_pos, r_off.y_pos, 'r--', ...
     r_on.x_pos,  r_on.y_pos,  'b-', ...
     r_off.scenario.refPath(:,1), r_off.scenario.refPath(:,2), 'k:');

xlabel('x [m]');
ylabel('y [m]');
legend('controller off','controller on','reference path');
title('A1 DLC Trajectory Comparison');
axis equal;
grid on;
saveas(gcf, 'docs/figures/a1_trajectory.png');
```

Yaw rate 응답도 같은 방식으로 비교할 수 있다. 실제 변수명은 저장된 result 구조체에 따라 다를 수 있으므로, `fieldnames(r_on)`을 이용해 yaw rate 관련 field 이름을 확인한 뒤 plotting에 사용하였다.

```matlab
fieldnames(r_on)

figure;
plot(r_off.t, r_off.yawRate, 'r--'); hold on;
plot(r_on.t,  r_on.yawRate,  'b-');

xlabel('Time [s]');
ylabel('Yaw rate [rad/s]');
legend('controller off','controller on');
title('A1 DLC Yaw Rate Response');
grid on;
saveas(gcf, 'docs/figures/a1_yawrate.png');
```

정리하면, A1 DLC 결과는 본 제어기의 장점과 한계를 동시에 보여준다. Side slip angle은 목표값 이하로 억제되어 차량 안정성은 확보되었지만, path-following 오차는 충분히 줄이지 못하였다. 따라서 본 설계는 yaw stability 중심의 제어기로는 효과적이지만, reference path tracking을 개선하기 위해서는 lateral path error를 직접 사용하는 추가 제어기가 필요하다.

### 4.3 한 시나리오 deep dive — A7 (또는 본인이 가장 잘 푼 것)

A7 Brake-in-Turn 시나리오는 선회 중 제동이 동시에 발생하는 조건이다. 이 시나리오에서는 차량이 횡방향 가속을 받고 있는 상태에서 제동력이 추가되므로, 타이어가 사용할 수 있는 마찰 여유가 줄어든다. 따라서 yaw instability, side slip 증가, load transfer 증가가 동시에 발생하기 쉽다. Baseline에서는 sideSlipMax가 크게 증가하여 차량이 스핀아웃에 가까운 거동을 보였고, 본 설계에서는 이를 줄이는 것을 중요한 목표로 두었다.

최종 자동채점 결과에서 A7 시나리오의 주요 KPI는 다음과 같다.

| KPI               |   목표값 | 본인 설계 결과 |    점수 |
| ----------------- | ----: | -------: | ----: |
| sideSlipMax [deg] | ≤ 5.0 |   2.0452 | 8 / 8 |
| LTR_max           | ≤ 0.7 |   0.3318 | 7 / 7 |

A7에서 가장 중요한 성과는 sideSlipMax를 2.0452 deg로 낮춘 것이다. 목표값은 5 deg 이하였으므로 충분한 여유를 가지고 조건을 만족하였다. 또한 LTR_max도 0.3318로 나타나 목표값 0.7 이하를 만족하였다. 따라서 A7 Brake-in-Turn 시나리오에서는 총 15점 중 15점을 획득하였다.

이 성능 개선의 핵심 원인은 `ctrl_lateral.m`의 β-limiter와 `ctrl_coordinator.m`의 yaw moment allocation이다. `ctrl_lateral.m`에서는 side slip angle의 절댓값이 threshold를 초과하면 다음과 같이 slip을 줄이는 방향의 yaw moment를 생성하였다.

$$
M_z =
-K_{\beta}\ \mathrm{sign}(\beta)(|\beta|-\beta_{th})
$$

최종 설계에서는 threshold를 2.5 deg로 설정하였고, (K_{\beta}=30000)을 사용하였다. 이 제어 법칙은 side slip angle이 커질수록 더 큰 yaw moment를 요구하므로, 차량이 급격하게 미끄러지는 것을 억제한다.

생성된 yaw moment는 `ctrl_coordinator.m`에서 좌우 brake torque 차동으로 변환되었다. 본 설계에서는 yaw moment를 전륜과 후륜에 70:30 비율로 분배하고, 좌우 brake torque를 비대칭으로 만들어 ESC 효과를 구현하였다.

```matlab id="obpyn8"
dT_f = 0.70 * Mz * r_w / track;
dT_r = 0.30 * Mz * r_w / track;

T_esc = [ dT_f;
         -dT_f;
          dT_r;
         -dT_r ];
```

즉, 별도의 yaw moment actuator가 없더라도 좌우 제동력 차이를 이용해 차량의 yaw motion을 제어할 수 있다. A7 시나리오에서는 선회 중 제동으로 인해 차량이 불안정해지려는 순간 ESC yaw moment가 발생하고, coordinator가 이를 differential braking으로 변환하여 yaw instability를 억제하였다.

A7의 결과는 본 제어기가 yaw stability와 side slip suppression에는 효과적이라는 것을 보여준다. 특히 A1과 D1에서는 lateral path deviation이 남아 있었지만, A7에서는 path tracking보다 차량 자세 안정성이 더 중요한 KPI로 평가되었기 때문에 본 제어 구조가 좋은 성능을 보였다. 따라서 본 설계는 급제동과 선회가 결합된 상황에서 스핀아웃을 방지하는 안정화 제어기로는 충분히 효과적이었다고 판단된다.

다만 이 방식은 brake torque 차동을 이용하기 때문에, 제동 안정성과 경로 추종 성능을 동시에 최적화하는 데에는 한계가 있다. 향후 개선한다면 yaw moment allocation에 tire utilization이나 normal load를 함께 고려하는 WLS(weighted least squares) 기반 allocation을 적용하여, 각 바퀴의 마찰 한계를 더 정교하게 사용할 수 있을 것이다.


## 5. 분석 + 한계 (1-2 페이지)

### 5.1 가장 성공적이었던 시나리오

가장 성공적이었던 시나리오는 A7 Brake-in-Turn 시나리오이다. 이 시나리오는 선회 중 제동이 동시에 발생하는 조건이기 때문에 차량이 쉽게 불안정해질 수 있다. 특히 제동으로 인해 타이어의 종방향 힘이 커지면 횡방향 힘을 낼 수 있는 여유가 줄어들고, 그 결과 side slip angle 증가나 yaw instability가 발생할 가능성이 크다.

최종 결과에서 A7 시나리오는 sideSlipMax = 2.0452 deg, LTR_max = 0.3318로 나타났다. 두 값 모두 목표값인 sideSlipMax ≤ 5 deg, LTR_max ≤ 0.7을 충분히 만족하였고, 자동채점에서도 15점 만점을 받았다. 따라서 본 제어기 설계가 선회 중 제동 상황에서 차량 자세 안정성을 확보하는 데 효과적이었다고 판단할 수 있다.

A7에서 좋은 결과가 나온 가장 큰 이유는 `ctrl_lateral.m`의 β-limiter와 `ctrl_coordinator.m`의 differential braking이 잘 작동했기 때문이다. `ctrl_lateral.m`에서는 side slip angle이 threshold를 넘으면 slip을 줄이는 방향으로 yaw moment를 생성하였다. 이후 `ctrl_coordinator.m`에서는 이 yaw moment를 좌우 brake torque 차이로 변환하여 ESC 효과를 만들었다. 이 구조 덕분에 차량이 선회 중 제동으로 인해 미끄러지려는 상황에서도 side slip angle이 과도하게 증가하지 않았다.

또한 A7에서는 lateral path tracking보다 차량의 yaw stability와 sideslip suppression이 더 중요한 KPI로 평가되었다. 본 설계는 path error를 직접 제어하지는 않지만, yaw rate와 side slip angle을 이용해 차량 자세를 안정화하는 데 초점을 두었다. 따라서 A1이나 D1처럼 lateralDevMax가 중요한 시나리오보다, A7처럼 자세 안정성이 핵심인 시나리오에서 더 좋은 성능을 보였다.

결론적으로 A7 결과는 본 제어기의 강점을 가장 잘 보여준다. 본 설계는 복잡한 최적제어기 없이도 yaw rate feedback, side slip threshold 기반 ESC, brake torque allocation을 조합하여 선회 중 제동 상황에서 스핀아웃을 방지할 수 있었다. 따라서 A7은 본 프로젝트에서 가장 성공적으로 개선된 시나리오라고 판단된다.

### 5.2 가장 부족했던 시나리오

가장 부족했던 시나리오는 B1 Straight Brake와 A1/D1의 lateral path deviation 항목이다. 먼저 B1 Straight Brake에서는 `absSlipRMS`를 0.0899로 낮추어 목표값 0.1 이하를 만족하였지만, stoppingDistance는 70.1145 m로 나타나 목표값 40 m에 도달하지 못하였다. 즉, wheel lock을 방지하는 ABS 안정성은 확보했지만, 제동거리를 충분히 줄이지 못한 것이 가장 큰 한계였다.

B1에서 이러한 결과가 나온 이유는 본 종방향 제어기가 제동거리 최소화보다 wheel slip 억제를 우선하도록 설계되었기 때문이다. 최종 코드에서는 wheel slip ratio가 커질 때 `brakeRatio`를 음수로 만들어 기존 brake torque를 줄이는 ABS relief 구조를 사용하였다. 이 방식은 wheel lock을 방지하는 데 효과적이었고, 실제로 `absSlipRMS` 항목에서는 만점을 받았다. 그러나 brake relief가 보수적으로 작동하면서 제동력이 충분히 유지되지 못했고, 그 결과 stoppingDistance가 크게 줄어들지 않았다. 따라서 B1에서는 slip 안정성과 제동거리 사이의 trade-off가 발생했다고 볼 수 있다.

또 다른 부족한 부분은 A1 DLC와 D1 DLC + Brake에서의 lateralDevMax이다. A1의 lateralDevMax는 2.1668 m로 목표값 0.7 m를 만족하지 못했고, D1의 lateralDevMax도 2.1668 m로 목표값 1.0 m를 만족하지 못하였다. 반면 두 시나리오에서 sideSlipMax는 목표값 이하로 유지되었다. 이는 본 lateral controller가 reference path를 정확히 따라가는 것보다 yaw stability와 side slip suppression을 우선하도록 설계되었기 때문이다.

현재 `ctrl_lateral.m`은 yawRateRef와 yawRate의 차이를 이용해 AFS 보조 조향을 만들고, side slip angle이 threshold를 초과하면 ESC yaw moment를 생성한다. 하지만 lateral path error, heading error, look-ahead error와 같은 path-following 관련 상태는 직접 입력으로 사용하지 않는다. 따라서 차량의 자세 안정성은 개선되었지만, reference path와의 거리 오차를 직접 줄이는 기능은 부족했다.

정리하면, 본 설계의 한계는 두 가지로 볼 수 있다. 첫째, B1에서는 ABS relief가 wheel slip을 안정적으로 낮추었지만 제동거리를 줄이는 데에는 한계가 있었다. 둘째, A1과 D1에서는 yaw rate와 side slip angle 중심의 제어 구조 때문에 path deviation을 직접 제어하지 못하였다. 향후 개선을 위해서는 B1에서는 slip target 주변에서 brake torque를 더 적극적으로 회복하는 ABS modulation이 필요하고, A1/D1에서는 path error 또는 heading error를 feedback으로 사용하는 path-following 보상기가 추가되어야 한다.

### 5.3 만약 더 시간이 있었다면

만약 더 시간이 있었다면 가장 먼저 B1 Straight Brake의 stoppingDistance를 개선하고 싶다. 현재 설계에서는 wheel slip ratio가 커질 때 brake torque를 줄이는 ABS relief를 적용하여 `absSlipRMS`는 목표값 이하로 낮출 수 있었다. 그러나 brake relief가 보수적으로 작동하면서 stoppingDistance가 70.1145 m로 남았다. 향후에는 slip ratio가 목표값 0.1~0.12 근처에 있을 때 brake torque를 더 적극적으로 회복시키는 closed-loop ABS modulation을 적용할 것이다. 예를 들어 slip error를 다음과 같이 정의하고,

$$
e_{\kappa} = \kappa_{target} - |\kappa|
$$

이 slip error에 대해 PI 또는 bang-bang 제어를 적용하면 wheel lock을 방지하면서도 더 큰 제동력을 유지할 수 있을 것이다. 현재 설계는 slip이 커지면 brake를 줄이는 기능은 있지만, slip이 안정화되었을 때 최적 제동력까지 빠르게 회복하는 기능은 부족하였다.

두 번째로 개선하고 싶은 부분은 A1과 D1의 lateral path deviation이다. 본 설계에서는 yaw rate와 side slip angle을 중심으로 lateral controller를 구성하였기 때문에 차량 자세 안정성은 확보할 수 있었지만, reference path를 직접 따라가는 기능은 부족하였다. 만약 path error, heading error, look-ahead point 정보를 사용할 수 있다면 Stanley controller 또는 Pure Pursuit 기반의 path-following 보상항을 추가할 수 있다. 이 경우 AFS 보조 조향은 단순히 yaw rate error만 따르는 것이 아니라, 경로 오차를 줄이는 방향으로도 작동할 수 있으므로 A1과 D1의 lateralDevMax를 개선할 수 있을 것으로 예상된다.

세 번째로는 coordinator의 actuator allocation을 더 정교하게 만들고 싶다. 현재는 yaw moment를 전륜과 후륜에 70:30 비율로 나누고, 좌우 brake torque 차이를 이용해 ESC 효과를 만들었다. 이 방식은 단순하고 안정적이지만 각 바퀴의 normal load, tire utilization, friction limit을 직접 고려하지 않는다. 향후에는 WLS(weighted least squares) 기반 allocation을 적용하여 각 바퀴가 사용할 수 있는 마찰 여유를 고려한 brake torque 분배를 수행할 수 있다. 이를 통해 ESC yaw moment와 ABS relief가 동시에 작동할 때도 특정 바퀴에 과도한 제동력이 집중되는 것을 줄일 수 있을 것이다.

마지막으로 vertical controller에서는 LTR을 직접 고려하는 roll control을 추가하고 싶다. 현재 `ctrl_vertical.m`은 skyhook damping을 이용하여 각 바퀴의 수직 속도 기반으로 damping coefficient를 조절한다. 이 방식은 차체 수직 운동을 줄이는 데는 유용하지만, A1과 D1에서 나타난 LTR_max를 직접 줄이는 데에는 한계가 있었다. 향후에는 roll angle, roll rate, 또는 LTR 추정값을 feedback으로 사용하여 좌우 damping을 비대칭으로 조절하는 방식으로 load transfer를 더 적극적으로 줄일 수 있을 것이다.

정리하면, 추가 시간이 주어진다면 본 설계는 단순한 rule-based feedback 구조에서 path-following, optimal brake modulation, tire utilization 기반 allocation, roll stability feedback을 포함하는 통합 제어 구조로 발전시킬 수 있다. 이를 통해 현재 남아 있는 stoppingDistance, lateralDevMax, LTR_max 항목을 추가로 개선할 수 있을 것으로 판단된다.

## 6. 참고문헌

[1] ISO 3888-1:2018 — Passenger cars — Test track for a severe lane-change manoeuvre.
[2] ISO 4138:2021 — Steady-state circular driving behaviour.
[3] R. Rajamani, *Vehicle Dynamics and Control*, 2nd ed., Springer 2012. §2.5 (yaw rate response), §8 (ESC).
[4] J. Y. Wong, *Theory of Ground Vehicles*, 4th ed., Wiley 2008.
[5] S. Di Cairano, H. E. Tseng, D. Bernardini, and A. Bemporad, "Vehicle yaw stability control by coordinated active front steering and differential braking in the tire sideslip angles domain," *IEEE Transactions on Control Systems Technology*, vol. 21, no. 4, pp. 1236–1248, 2013.
[6] D. Karnopp, M. J. Crosby, and R. A. Harwood, "Vibration control using semi-active force generators," *Journal of Engineering for Industry*, vol. 96, no. 2, pp. 619–626, 1974. DOI: 10.1115/1.3438373.
---

## 부록 A — 사용한 AI 도구

info.ai_usage = 'ChatGPT and Gemini was used to understand the project requirements, identify which files to modify, and get suggestions for controller design and debugging.';

## 부록 B — 본인 sim_params.m 변경사항

본 프로젝트에서는 `sim_params.m`의 기본 파라미터를 수정하지 않았다. 제어기 성능 개선은 `scripts/control` 폴더의 네 개 제어 함수인 `ctrl_lateral.m`, `ctrl_longitudinal.m`, `ctrl_vertical.m`, `ctrl_coordinator.m` 내부 로직을 수정하여 수행하였다.

따라서 `CTRL.*`, `LIM.*`, `SIM.solver` 등의 설정값은 기본값을 유지하였다.