"""
Minimal MILP for Drone→Task (1 Round) using SciPy HiGHS
======================================================

목적함수 (Min):
  sum_{i,j} (α·d̄_{ij} + β·ē_{ij} − γ·Ā_j)·x_{ij}  +  sum_j ρ_j·y_j

제약:
 (1) ∀j: sum_i x_{ij} + y_j = 1                 # 작업 j는 '배정' 또는 '드롭' 중 정확히 하나
 (2) ∀i: sum_j x_{ij} ≤ Q_i                    # 드론 i 라운드당 처리량(용량) 제한
 (3) ∀i: sum_j e_{ij} x_{ij} ≤ (1−σ)·E_i^max   # 드론 i 배터리 한계
 (4) ∀i,j: x_{ij} ≤ w_{ij}                     # 거리 게이트(너무 먼 작업은 배정 금지)

의사결정변수:
  x_{ij} ∈ {0,1},  y_j ∈ {0,1}
"""

from __future__ import annotations
import numpy as np
from scipy.optimize import milp, Bounds, LinearConstraint

# -----------------------------
# 0) 하이퍼파라미터
# -----------------------------
alpha, beta, gamma = 1.0, 1.0, 1.0
sigma = 0.2                 # 안전 여유율
time_limit = 5.0            # 초
mip_rel_gap = 1e-3
np.set_printoptions(precision=3, suppress=True)

# -----------------------------
# 1) 아주 작은 예시 데이터 (I=3, J=6) — 바로 실행 가능
# -----------------------------
I, J = 3, 6

# 왕복 비행거리 d_{ij} [km]  (실무에선 좌표로 계산)
d = np.array([
    [ 6,  8, 12,  5,  9,  7],
    [ 4, 10,  3,  6, 11,  8],
    [ 9,  6,  7, 12, 13,  5],
], dtype=float)

# 간단한 에너지 모델: e_{ij} = κ_i·d_{ij} + e_hover_j  [Wh]
kappa = np.array([12.0, 14.0, 11.0])              # Wh/km (드론별 순항 계수)
e_hover = np.array([10, 12, 8, 9, 11, 7], float)   # 작업별 호버/픽업 비용
e = kappa[:, None] * d + e_hover[None, :]          # (I,J)

# 드론 i의 사용 가능 배터리 E_i^max [Wh]
Emax = np.array([200.0, 220.0, 180.0])

# 작업 우선도 A_j (정규화에만 사용)
A = np.array([0.9, 0.2, 0.4, 0.7, 0.3, 0.6])

# 거리 게이트 w_{ij} ∈ {0,1}  (여기선 임계 D_i^th로 단순화: d_{ij}/2 ≤ D_i^th 이면 1)
Dth = np.array([7.0, 8.0, 6.0])  # km (단방향 임계)
w = ( (d/2.0) <= Dth[:, None] ).astype(float)

# 라운드당 용량 Q_i
Q = np.ones(I, dtype=int)  # 각 드론 최대 1개

# 드롭 페널티 ρ_j (크면 드롭 억제)
rho = 0.6 * np.ones(J, float)

# -----------------------------
# 2) 정규화 (수치 안정)
# -----------------------------
Dmax = max(np.max(d), 1.0)         # d̄_{ij} = d_{ij} / Dmax
Amax = max(np.max(A), 1.0)         # Ā_j   = A_j   / Amax
Emax_safe = np.where(Emax > 0, Emax, 1.0)

d_bar = d / Dmax
e_bar = e / Emax_safe[:, None]
A_bar = A / Amax

# -----------------------------
# 3) 변수 인덱싱
# -----------------------------
# x_{ij} : I*J개 (0..n_x-1),  y_j : 그 뒤 J개
n_x = I * J
n_y = J
n = n_x + n_y

def idx_x(i, j):  return i*J + j
def idx_y(j):     return n_x + j

# -----------------------------
# 4) 목적함수 c
#     c_x(i,j) = α·d̄_{ij} + β·ē_{ij} − γ·Ā_j
# -----------------------------
c_x = alpha * d_bar + beta * e_bar - gamma * (A_bar[None, :])
c = np.zeros(n, float)
c[:n_x] = c_x.ravel(order="C")
c[n_x:] = rho

# -----------------------------
# 5) Bounds (이진 + 게이트/에너지 불가 페어 제거)
#     - 기본: 0 ≤ x,y ≤ 1
#     - (4) x_{ij} ≤ w_{ij} → ub_x = w
#     - e_{ij} > (1−σ)E_i^max 인 페어는 애초에 불가 → ub=0
# -----------------------------
lb = np.zeros(n)
ub = np.ones(n)
ub_x = ub[:n_x].reshape(I, J)
ub_x[:] = w

cap = (1.0 - sigma) * Emax
energy_infeasible = e > (cap[:, None] + 1e-12)
ub_x[energy_infeasible] = 0.0

ub[:n_x] = ub_x.ravel(order="C")
bounds = Bounds(lb, ub)

# SciPy에서 이진은 integrality=1 + Bounds[0,1]로 표현
integrality = np.ones(n, dtype=int)

# -----------------------------
# 6) 제약 구성
#   (1) ∀j: Σ_i x_{ij} + y_j = 1
#   (2) ∀i: Σ_j x_{ij} ≤ Q_i
#   (3) ∀i: Σ_j e_{ij} x_{ij} ≤ cap_i
# -----------------------------
# (1) 작업별 1-of-1 (dense로 간단히)
Aeq = np.zeros((J, n))
for j in range(J):
    for i in range(I):
        Aeq[j, idx_x(i, j)] = 1.0
    Aeq[j, idx_y(j)] = 1.0
con1 = LinearConstraint(Aeq, lb=np.ones(J), ub=np.ones(J))

# (2) 드론 용량
Aub_cap = np.zeros((I, n))
for i in range(I):
    for j in range(J):
        Aub_cap[i, idx_x(i, j)] = 1.0
con2 = LinearConstraint(Aub_cap, lb=-np.inf*np.ones(I), ub=Q.astype(float))

# (3) 드론 배터리
Aub_en = np.zeros((I, n))
for i in range(I):
    for j in range(J):
        if ub_x[i, j] > 0.0:       # 이미 ub=0(제거)된 변수는 생략해도 무방
            Aub_en[i, idx_x(i, j)] = e[i, j]
con3 = LinearConstraint(Aub_en, lb=-np.inf*np.ones(I), ub=cap.astype(float))

constraints = [con1, con2, con3]

# -----------------------------
# 7) 풀기
# -----------------------------
res = milp(
    c=c,
    integrality=integrality,
    bounds=bounds,
    constraints=constraints,
    options={"time_limit": time_limit, "mip_rel_gap": mip_rel_gap, "presolve": True},
)

# -----------------------------
# 8) 해 라운딩 및 간단 출력
# -----------------------------
x_bin = np.zeros((I, J), dtype=int)
y_bin = np.zeros(J, dtype=int)
if res.x is not None:
    x_raw = res.x[:n_x].reshape(I, J)
    y_raw = res.x[n_x:]
    print(x_raw)
    print(y_raw)
    x_bin = (x_raw > 0.5).astype(int)
    y_bin = (y_raw > 0.5).astype(int)

assigned = [(i, j) for i in range(I) for j in range(J) if x_bin[i, j] == 1]
dropped  = [j for j in range(J) if y_bin[j] == 1]

print("=== Solver ===")
print("Status:", res.status, res.message)
print(f"Objective: {float(res.fun):.6f}\n")

print("Assigned (i→j):", assigned)
print("Dropped  (j):  ", dropped)

# 간단 검증 (필요하면 주석 처리 가능)
assert np.all(x_bin.sum(axis=0) + y_bin == 1), "1-of-1 violated"
assert np.all(x_bin.sum(axis=1) <= Q), "Capacity violated"
assert np.all((e * x_bin).sum(axis=1) <= cap + 1e-6), "Energy cap violated"
assert np.all(x_bin <= w + 1e-9), "Gate violated"

# -----------------------------
# 9) 작업별 '왜 배정/드롭?' 간단 리포트
#     - min c_x(j) vs ρ_j 비교
#     - feasible 드론 목록
# -----------------------------
print("\n=== Per-Task quick check (min c_x vs rho) ===")
MASK = (ub_x <= 0.0)  # 게이트/에너지로 불가
c_full = alpha*(d/Dmax) + beta*(e/Emax[:,None]) - gamma*(A/Amax)[None,:]
c_masked = np.where(MASK, np.inf, c_full)
for j in range(J):
    feasible_i = [i for i in range(I) if not MASK[i, j]]
    best_c = float(np.min(c_masked[:, j])) if feasible_i else float('inf')
    dec = "ASSIGN" if y_bin[j] == 0 else ("DROP" if feasible_i and best_c >= rho[j] else ("DROP*INFEASIBLE" if not feasible_i else "DROP"))
    print(f"j={j}: min c_x={best_c:7.4f}, rho={rho[j]:.2f}, decision={dec}, feasible_i={feasible_i}")
