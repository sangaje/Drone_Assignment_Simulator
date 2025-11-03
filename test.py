"""Drone–Task 1-Round Assignment MILP (SciPy + HiGHS) with Detailed Diagnostics
-----------------------------------------------------------------------------
정식화(당신이 준 식과 동일):

집합
  • 드론 I, 작업 J

파라미터
  • d_{ij} ≥ 0     : (i,j) 총 비행거리(왕복)
  • e_{ij} ≥ 0     : (i,j) 총 에너지(Wh)  # 상수(사전 계산)
  • E_i^{max} > 0  : 드론 i 사용 가능 배터리(Wh)
  • σ ∈ [0,1)      : 안전 여유율 (cap_i = (1−σ)E_i^{max})
  • A_j ≥ 0        : 작업 j 우선도/가치
  • w_{ij} ∈ {0,1} : 거리 게이트 (base_i→j가 너무 멀지 않으면 1)
  • Q_i ∈ ℤ_+      : 라운드당 드론 i가 처리 가능한 최대 작업 수 (기본 1)
  • α,β,γ ≥ 0      : 거리·에너지·우선도 가중치
  • ρ_j ≥ 0        : 작업 j를 드롭할 때의 페널티
  • (정규화 권장) d̄_{ij}=d_{ij}/D_max, ē_{ij}=e_{ij}/E_i^{max}, Ā_j=A_j/A_max

의사결정변수
  • x_{ij} ∈ {0,1} : 드론 i가 작업 j를 수행하면 1
  • y_j   ∈ {0,1}  : 작업 j를 드롭(유예)하면 1

목적함수 (Minimize)
  •  Σ_{i∈I} Σ_{j∈J} ( α·d̄_{ij} + β·ē_{ij} − γ·Ā_j ) x_{ij}  +  Σ_{j∈J} ρ_j y_j

제약식
 (1) ∀j: Σ_i x_{ij} + y_j = 1                         # 작업별 '배정 또는 드롭' 정확히 하나
 (2) ∀i: Σ_j x_{ij} ≤ Q_i                            # 드론의 동시 처리 가능 작업 수 제한
 (3) ∀i: Σ_j e_{ij} x_{ij} ≤ (1−σ)·E_i^{max}         # 배터리(에너지) 한계
 (4) ∀i,j: x_{ij} ≤ w_{ij}                           # 거리 게이트 (너무 먼 작업 금지)

구현 포인트
  • (4)는 행 제약 대신 Bounds 상한으로 구현: w_{ij}=0 → ub(x_{ij})=0 (변수 자체 제거 효과)
  • 에너지로도 '애초에 불가능'한 (i,j) (e_{ij} > cap_i) 역시 ub=0으로 제거 → 모델 희소화/가속
  • 솔버: SciPy 1.16+ 의 scipy.optimize.milp (HiGHS MIP)
"""

from __future__ import annotations

import numpy as np
from scipy.optimize import Bounds, LinearConstraint, milp
from scipy.sparse import csr_matrix, dok_matrix


# =========================================================
# 1) 예시 데이터 생성 (물리적 일관성 유지)
# =========================================================
def make_example(I=6, J=12, seed=7):
    """- 드론 base, 작업 위치를 2D 평면에 난수 배치
    - d_{ij}: 왕복거리(km)
    - e_{ij}: κ_i·d_{ij} + e_hover_j  (Wh)  # 선형 근사
    - Emax_i: 드론 i의 가용 배터리(Wh)
    - w_{ij}: 단방향 거리 ≤ D_i^{th} 이면 1 (접근 가능)
    - A_j: 작업 우선도
    - Q_i: 라운드당 용량(기본 1)
    """
    rng = np.random.default_rng(seed)

    bases = rng.uniform([0,0],[10,10], size=(I,2))
    tasks = rng.uniform([-2,-2],[12,12], size=(J,2))

    single = np.linalg.norm(bases[:,None,:] - tasks[None,:,:], axis=-1)  # (I,J)
    d_km = 2.0 * single  # 왕복(km)
    d = d_km

    kappa = rng.uniform(10.0, 25.0, size=I)     # Wh/km
    e_hover = rng.uniform(5.0, 20.0, size=J)    # Wh
    e = (kappa[:,None] * d) + e_hover[None,:]

    Emax = rng.uniform(200.0, 400.0, size=I)    # Wh

    Dth = rng.uniform(4.0, 7.0, size=I)         # km (단방향)
    w = (single <= Dth[:,None]).astype(float)

    A = rng.uniform(0.1, 1.0, size=J)
    Q = np.ones(I, dtype=int)

    return d, e, Emax, w, A, Q, bases, tasks, Dth


# =========================================================
# 2) MILP 빌드 (수식→행렬, 경계, 정수성 매핑)
# =========================================================
def build_and_solve_milp(
    d: np.ndarray, e: np.ndarray, Emax: np.ndarray, w: np.ndarray,
    A: np.ndarray, Q: np.ndarray | int,
    alpha=1.0, beta=1.0, gamma=1.0, rho=None,
    sigma=0.2, normalize=True,
    time_limit=10.0, mip_rel_gap=1e-3, presolve=True
):
    """입력:
      • d,e,Emax,w,A,Q,alpha,beta,gamma,rho,sigma (위에 정의된 의미와 동일)
    출력:
      • (res, dict) : SciPy 결과(res)와 진단을 위한 모든 중간물(diag) 딕셔너리
    """
    I, J = d.shape
    e = np.asarray(e, float); Emax = np.asarray(Emax, float).reshape(-1)
    w = np.asarray(w, float); A = np.asarray(A, float).reshape(-1)

    if np.isscalar(Q): Q = np.full(I, int(Q), dtype=int)
    else: Q = np.asarray(Q, int).reshape(-1)

    # ---------- 정규화 구성(권장) ----------
    if normalize:
        Dmax = max(np.max(d), 1.0)
        d_bar = d / Dmax
        Emax_safe = np.where(Emax > 0, Emax, 1.0)
        e_bar = e / Emax_safe[:,None]
        Amax = max(np.max(A), 1.0)
        A_bar = A / Amax
    else:
        Dmax = 1.0; Amax = 1.0
        Emax_safe = np.where(Emax > 0, Emax, 1.0)
        d_bar, e_bar, A_bar = d, e, A

    # ---------- 드롭 페널티 ----------
    if rho is None:
        rho = 0.6 * np.ones(J, float)
    else:
        rho = np.asarray(rho, float).reshape(-1)

    # ---------- 변수 인덱싱 ----------
    # x_{ij}: I*J개 (0..n_x-1),  y_j: 그 뒤 J개 (n_x..n_x+n_y-1)
    n_x = I * J
    n_y = J
    n = n_x + n_y

    def idx_x(i,j): return i*J + j
    def idx_y(j):   return n_x + j

    # ---------- 목적함수 c ----------
    # c_x(i,j) = α d̄_{ij} + β ē_{ij} − γ Ā_j  # (식 그대로)
    c_x = alpha * d_bar + beta * e_bar - gamma * (A_bar[None,:])
    c = np.zeros(n, float)
    c[:n_x] = c_x.ravel(order="C")
    c[n_x:] = rho

    # ---------- Bounds(이진 + 게이트/에너지 상한) ----------
    lb = np.zeros(n); ub = np.ones(n)
    # 거리 게이트 (4): x_{ij} ≤ w_{ij} → 상한에 바로 반영
    ub_x = ub[:n_x].reshape(I,J); ub_x[:] = w
    # 에너지 '애초에 불가' (단일 페어 관점): e_{ij} > cap_i → ub=0
    cap = (1.0 - sigma) * Emax
    energy_infeasible = e > (cap[:,None] + 1e-12)
    ub_x[energy_infeasible] = 0.0
    ub[:n_x] = ub_x.ravel(order="C")
    bounds = Bounds(lb, ub)

    # ---------- 정수성 ----------
    integrality = np.ones(n, dtype=int)  # binary는 [0,1] 경계 + integrality=1

    # ---------- 제약 (1) 작업별 1-of-1 ----------
    Aeq = dok_matrix((J, n), float)
    for j in range(J):
        for i in range(I): Aeq[j, idx_x(i,j)] = 1.0
        Aeq[j, idx_y(j)] = 1.0
    Aeq = csr_matrix(Aeq)
    cons = [LinearConstraint(Aeq, lb=np.ones(J), ub=np.ones(J))]

    # ---------- 제약 (2) 드론 용량 ----------
    Aub_cap = dok_matrix((I, n), float)
    for i in range(I):
        for j in range(J):
            Aub_cap[i, idx_x(i,j)] = 1.0
    Aub_cap = csr_matrix(Aub_cap)
    cons.append(LinearConstraint(Aub_cap, lb=-np.inf*np.ones(I), ub=Q.astype(float)))

    # ---------- 제약 (3) 드론 배터리 ----------
    Aub_en = dok_matrix((I, n), float)
    for i in range(I):
        for j in range(J):
            if ub_x[i,j] > 0.0:  # 이미 ub=0(제거)된 변수엔 계수 안 넣어 희소성↑
                Aub_en[i, idx_x(i,j)] = e[i,j]
    Aub_en = csr_matrix(Aub_en)
    cons.append(LinearConstraint(Aub_en, lb=-np.inf*np.ones(I), ub=cap))

    # ---------- 풀기 ----------
    options = {"time_limit": time_limit, "mip_rel_gap": mip_rel_gap, "presolve": bool(presolve)}
    res = milp(c=c, integrality=integrality, bounds=bounds, constraints=cons, options=options)

    # ---------- 결과/진단 패키징 ----------
    diag = dict(
        I=I, J=J, n=n, n_x=n_x, n_y=n_y,
        alpha=alpha, beta=beta, gamma=gamma, rho=rho,
        d=d, e=e, Emax=Emax, w=w, Q=Q, A=A,
        d_bar=d_bar, e_bar=e_bar, A_bar=A_bar,
        Dmax=Dmax, Amax=Amax, cap=cap, ub_x=ub_x, energy_infeasible=energy_infeasible,
        Aeq=Aeq, Aub_cap=Aub_cap, Aub_en=Aub_en, bounds=bounds, c_x=c_x, c=c
    )
    return res, diag


# =========================================================
# 3) 상세 진단/설명 출력
# =========================================================
def diagnostics(res, diag, topk=3):
    I, J = diag["I"], diag["J"]
    n_x = diag["n_x"]; Q = diag["Q"]
    d, e, A = diag["d"], diag["e"], diag["A"]
    d_bar, e_bar, A_bar = diag["d_bar"], diag["e_bar"], diag["A_bar"]
    alpha, beta, gamma, rho = diag["alpha"], diag["beta"], diag["gamma"], diag["rho"]
    cap = diag["cap"]; ub_x = diag["ub_x"]; c_x = diag["c_x"]
    Dmax, Amax = diag["Dmax"], diag["Amax"]
    w = diag["w"]

    # 해 추출 및 이진화(라운딩)
    x_bin = np.zeros((I,J), int); y_bin = np.zeros(J, int)
    if res.x is not None:
        x_raw = res.x[:n_x].reshape(I,J)
        y_raw = res.x[n_x:]
        x_bin = (x_raw > 0.5).astype(int)
        y_bin = (y_raw > 0.5).astype(int)

    # 요약
    print("=== Solver ===")
    print("Status:", res.status, res.message)
    print(f"Objective: {float(res.fun):.6f}   (reported gap: {getattr(res,'mip_rel_gap',None)})")
    print()

    assigned = [(i,j) for i in range(I) for j in range(J) if x_bin[i,j]==1]
    dropped  = [j for j in range(J) if y_bin[j]==1]
    print("Assigned pairs (i→j):", assigned)
    print("Dropped tasks:", dropped)
    print()

    # 사후 검증
    assert np.all(x_bin.sum(axis=0) + y_bin == 1), "1-of-1 violated"
    assert np.all(x_bin.sum(axis=1) <= Q), "Capacity violated"
    assert np.all((e*x_bin).sum(axis=1) <= cap + 1e-6), "Energy cap violated"
    assert np.all(x_bin <= w + 1e-9), "Gate violated"

    # 드론별 사용량/여유
    assigned_per_i = x_bin.sum(axis=1)
    energy_used_i = (e*x_bin).sum(axis=1)
    slack_energy_i = cap - energy_used_i
    slack_cap_i = Q - assigned_per_i

    print("Per-drone assigned:", assigned_per_i.tolist())
    print("Per-drone energy used (Wh):", np.round(energy_used_i,3).tolist())
    print("Per-drone energy slack (Wh):", np.round(slack_energy_i,3).tolist())
    print("Per-drone capacity slack:", slack_cap_i.tolist())
    print("Feasible x-vars kept:", int((ub_x>0).sum()), "/", I*J)
    print()

    # ------------ 유틸: 후보/비용/이유 ------------
    # 마스크: ub_x<=0 (게이트/에너지로 애초 불가)
    MASK = (ub_x <= 0.0)
    # 불가능 페어는 +inf로 마스킹하여 min 계산시 제외
    c_masked = np.where(MASK, np.inf, (alpha*(d/Dmax) + beta*(e/diag["Emax"][:,None]) - gamma*(A/Amax)[None,:]))

    def top_k_candidates_for_task(j, k=3):
        # (i, c_ij, reason_str)
        rows = []
        for i in range(I):
            if ub_x[i,j] <= 0.0:
                reason = []
                if w[i,j] <= 0.5: reason.append("GATE")
                if e[i,j] > cap[i] + 1e-12: reason.append("ENERGY>cap")
                rows.append((i, np.inf, "|".join(reason) if reason else "INFEASIBLE"))
            else:
                comp = alpha*(d[i,j]/Dmax) + beta*(e[i,j]/diag["Emax"][i]) - gamma*(A[j]/Amax)
                # ex-post 관점: 현 해에서 이 드론에 더 배정 가능?
                can_add_capacity = (slack_cap_i[i] > 0)
                can_add_energy   = (e[i,j] <= slack_energy_i[i] + 1e-12)
                explain = []
                if not can_add_capacity: explain.append("CAP=0")
                if not can_add_energy:   explain.append("ENERGY_SLACK<e_ij")
                rows.append((i, comp, "|".join(explain) if explain else "OK"))
        # 비용 기준 오름차순 정렬
        rows_sorted = sorted(rows, key=lambda t: t[1])
        return rows_sorted[:k], rows_sorted

    # ------------ 작업별 상세 리포트 ------------
    print("=== Per-Task Analysis ===")
    header = ("j  assigned  reason/notes                              "
              " best_c_x    rho_j   decision   comps(αd̄, βē, −γĀ) for assigned or best")
    print(header)
    print("-"*len(header))

    for j in range(J):
        is_drop = (y_bin[j]==1)
        topk_list, all_list = top_k_candidates_for_task(j, k=topk)

        if not is_drop:
            # 배정된 드론 i*
            i_star = np.where(x_bin[:,j]==1)[0][0]
            comp_d = alpha * d_bar[i_star,j]
            comp_e = beta * e_bar[i_star,j]
            comp_a = -gamma * A_bar[j]
            c_star = comp_d + comp_e + comp_a

            # 보조 정보: 대안 후보 top-1 (배정된 것과 동일할 수도)
            best_i, best_c, best_explain = topk_list[0]
            explain = []
            if ub_x[i_star,j] <= 0.0:
                explain.append("(!) unexpected ub=0 for assigned")
            # ex-post 여유
            if slack_cap_i[i_star] < 0: explain.append("CAP<0?!")
            if slack_energy_i[i_star] < -1e-9: explain.append("ENERGY<0?!")
            note = "OK"
            if explain: note = ",".join(explain)

            print(f"{j:2d}  {i_star:8d}  {note:35s}  {c_star:8.4f}  {rho[j]:6.2f}   ASSIGN    "
                  f"({comp_d:5.3f}, {comp_e:5.3f}, {comp_a:5.3f})")

            # 상위 대안 후보들도 표시
            print(f"     → Top-{topk} candidates by c_x (i, c_x, status): {[(i, float(c), s) for (i,c,s) in topk_list]}")

        else:
            # 드롭: 왜 드롭되었는지
            feasible_ijs = [i for i in range(I) if ub_x[i,j] > 0.0]
            if len(feasible_ijs)==0:
                # 순수 불가(게이트/에너지)
                gate_fail = int((w[:,j]<=0.5).sum())
                en_fail   = int((e[:,j] > cap + 1e-12).sum())
                print(f"{j:2d}  {'-':8s}  INFEASIBLE: no drone passes gate/energy   "
                      f"{np.inf:8}  {rho[j]:6.2f}   DROP      (---, ---, ---)")
                print(f"     · gate_fail={gate_fail}, energy_fail={en_fail}")
            else:
                best_i = min(feasible_ijs, key=lambda i: c_masked[i,j])
                best_c = c_masked[best_i, j]

                # ex-post 관점: 현 해에서 추가 배정 가능한 드론 있는지?
                cap_ok = [i for i in feasible_ijs if (diag['Q'][i]-x_bin[i,:].sum())>0]
                en_ok  = [i for i in feasible_ijs if e[i,j] <= (cap[i] - (e*x_bin).sum(axis=1)[i] + 1e-12)]

                # 의사결정 사유 분기
                if best_c >= rho[j]:
                    decision = "DROP (CHEAPER_THAN_ASSIGN)"  # ρ가 더 싸다
                    reason = "local economics: min c_x >= rho"
                else:
                    # best_c < rho인데도 드롭된 경우 → 전역 제약 결합
                    if len(cap_ok)==0 and len(en_ok)==0:
                        decision = "DROP (NO_CAP & NO_ENERGY_SLACK)"
                        reason = "all feasible drones saturated (cap and/or energy)"
                    elif len(cap_ok)==0:
                        decision = "DROP (NO_CAP)"
                        reason = "all feasible drones at capacity"
                    elif len(en_ok)==0:
                        decision = "DROP (NO_ENERGY_SLACK)"
                        reason = "all feasible drones lack energy slack"
                    else:
                        decision = "DROP (GLOBAL_TRADEOFF)"
                        reason = "exists feasible+cheap candidate, but worse globally"

                # 보고
                # best 후보의 비용성분도 같이 출력
                comp_d = alpha * d_bar[best_i, j] if np.isfinite(best_c) else np.nan
                comp_e = beta * e_bar[best_i, j] if np.isfinite(best_c) else np.nan
                comp_a = -gamma * A_bar[j]        if np.isfinite(best_c) else np.nan
                print(f"{j:2d}  {'-':8s}  {reason:35s}  {best_c:8.4f}  {rho[j]:6.2f}   {decision:10s} "
                      f"({comp_d if np.isfinite(best_c) else np.nan:5.3f}, "
                      f"{comp_e if np.isfinite(best_c) else np.nan:5.3f}, "
                      f"{comp_a if np.isfinite(best_c) else np.nan:5.3f})")

                # 상위 후보 나열
                topk_list, _ = top_k_candidates_for_task(j, k=topk)
                print(f"     → Top-{topk} candidates by c_x (i, c_x, status): {[(i, float(c), s) for (i,c,s) in topk_list]}")

    print()

    # ------------ 배정된 페어 상세(비용 분해, 제약 여유) ------------
    print("=== Assigned-Pair Details (cost breakdown & constraints) ===")
    for (i,j) in assigned:
        comp_d = alpha * d_bar[i,j]
        comp_e = beta * e_bar[i,j]
        comp_a = -gamma * A_bar[j]
        c_ij = comp_d + comp_e + comp_a
        print(f"(i={i}, j={j})  c_x={c_ij:7.4f}  comps(αd̄,βē,−γĀ)=({comp_d:5.3f},{comp_e:5.3f},{comp_a:5.3f})  "
              f"e_ij={e[i,j]:6.2f}Wh  cap_i={(cap[i]):6.2f}Wh  used_i={(e*x_bin).sum(axis=1)[i]:6.2f}Wh  "
              f"slack_i={(cap[i]-(e*x_bin).sum(axis=1)[i]):6.2f}Wh  "
              f"used_count={int((x_bin[i,:]).sum())}/{int(Q[i])}")

    # ------------ 목적함수 항 기여도 확인 ------------
    obj_x = (alpha*(d/Dmax) + beta*(e/diag["Emax"][:,None]) - gamma*(A/Amax)[None,:]) * x_bin
    obj_y = rho * y_bin
    print()
    print(f"Obj from x-block: {float(obj_x.sum()):.6f}")
    print(f"Obj from y-block: {float(obj_y.sum()):.6f}")
    print(f"Total (≈ SciPy fun): {float(obj_x.sum()+obj_y.sum()):.6f}")


# =========================================================
# 4) 엔트리 포인트: 예시 데이터로 즉시 실행
# =========================================================
if __name__ == "__main__":
    # 하이퍼파라미터: 필요하면 바꾸어 민감도 확인
    alpha, beta, gamma = 1.0, 1.0, 1.0
    sigma = 0.2
    rho_default = None        # None → 0.6로 설정
    time_limit = 10.0
    mip_rel_gap = 1e-3

    # 예시 인스턴스 생성
    d, e, Emax, w, A, Q, *_ = make_example(I=6, J=12, seed=42)

    # MILP 빌드 & 풀이
    res, diag = build_and_solve_milp(
        d, e, Emax, w, A, Q,
        alpha=alpha, beta=beta, gamma=gamma, rho=rho_default,
        sigma=sigma, normalize=True,
        time_limit=time_limit, mip_rel_gap=mip_rel_gap, presolve=True
    )

    # 진단 출력
    diagnostics(res, diag, topk=3)
