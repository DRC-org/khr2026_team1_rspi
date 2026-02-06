#!/usr/bin/env python3
"""
Corrected Task Optimizer for 関西春ロボコン2026 Rulebook ver.1.0.1
Model: Auto Robot
Strategy: 60s Conquest (V-Goal)
"""

import sys

try:
    from ortools.sat.python import cp_model
except ImportError:
    print("Error: ortools not installed. Please run: pip install ortools")
    sys.exit(1)

class TaskOptimizer:
    def __init__(self):
        # 自動ロボット前提: 王手条件「3つの櫓にリング2個以上」
        # 陣取りスポットの設定を「リング2個消費」に変更
        self.spots = {
            # Start (右下隅)
            0: {"name": "Start", "x": 0.3, "y": 0.5, "type": "start", "action_time": 0},
            
            # --- ターゲット (陣取りゾーン) ---
            # 自動ロボット王手条件: 各櫓にリング2個以上
            # 実機制約: ハンド2つ×2個 = 計4個保持、投下は2個単位（ハンド単位）
            # req_r: 2 (ルール要件＋実機のハンド投下単位)
            # score: 58点 = 櫓10 + リング40(20×2) + IN8(4×2)
            1: {"name": "陣取り_1", "x": 3.15, "y": 2.0, "type": "target", "action_time": 13, "req_y": 1, "req_r": 2, "score": 58, "required_for_vgoal": True},
            2: {"name": "陣取り_2", "x": 3.15, "y": 3.2, "type": "target", "action_time": 13, "req_y": 1, "req_r": 2, "score": 58, "required_for_vgoal": True},
            3: {"name": "陣取り_3", "x": 3.15, "y": 4.4, "type": "target", "action_time": 13, "req_y": 1, "req_r": 2, "score": 58, "required_for_vgoal": True},
            # 最速攻略のため陣取り4は除外（行かないようにする）
            # 4: {"name": "陣取り_4", "x": 3.15, "y": 5.6, "type": "target", "action_time": 13, "req_y": 1, "req_r": 2, "score": 58, "required_for_vgoal": False},
            
            # --- ターゲット (本丸) ---
            # 本丸はリング1個でOK（ただし実機は2個単位なので余る）
            5: {"name": "本丸", "x": 3.25, "y": 1.40, "type": "honmaru", "action_time": 4, "req_y": 0, "req_r": 1, "score": 120, "required_for_vgoal": True},
            
            # --- 補給ポイント (仮想ノード) ---
            # 複数回補給に行けるように仮想ノードを確保（4個ずつに削減して探索効率化）
            101: {"name": "櫓補給_A", "x": 2.96, "y": 0.6, "type": "supply", "action_time": 4, "gain_y": 2, "gain_r": 0, "score": 20, "required_for_vgoal": False},
            102: {"name": "櫓補給_B", "x": 2.96, "y": 0.6, "type": "supply", "action_time": 4, "gain_y": 2, "gain_r": 0, "score": 0, "required_for_vgoal": False},
            103: {"name": "櫓補給_C", "x": 2.96, "y": 0.6, "type": "supply", "action_time": 4, "gain_y": 2, "gain_r": 0, "score": 0, "required_for_vgoal": False},
            104: {"name": "櫓補給_D", "x": 2.96, "y": 0.6, "type": "supply", "action_time": 4, "gain_y": 2, "gain_r": 0, "score": 0, "required_for_vgoal": False},
            
            # リング取得: 右奥 (リングゾーン)
            201: {"name": "リング補給_A", "x": 0.7, "y": 6.2, "type": "supply", "action_time": 8, "gain_y": 0, "gain_r": 4, "score": 0, "required_for_vgoal": False},
            202: {"name": "リング補給_B", "x": 0.7, "y": 6.2, "type": "supply", "action_time": 8, "gain_y": 0, "gain_r": 4, "score": 0, "required_for_vgoal": False},
            203: {"name": "リング補給_C", "x": 0.7, "y": 6.2, "type": "supply", "action_time": 8, "gain_y": 0, "gain_r": 4, "score": 0, "required_for_vgoal": False},
            204: {"name": "リング補給_D", "x": 0.7, "y": 6.2, "type": "supply", "action_time": 8, "gain_y": 0, "gain_r": 4, "score": 0, "required_for_vgoal": False},
        }
        
        self.params = {
            "robot_speed": 2.1,         # m/s
            "max_yagura": 2,            # 櫓同時保持数（実機制約）
            "max_ring": 4,              # リング同時保持数（実機制約: 2ハンド×2個）
            "jintori_bonus": 30,        # 陣取り達成ボーナス/個
            "vgoal_bonus": 1000,        # 攻略達成ボーナス（評価用）
        }

    def dist(self, id1, id2):
        s1 = self.spots[id1]
        s2 = self.spots[id2]
        return ((s1["x"] - s2["x"])**2 + (s1["y"] - s2["y"])**2)**0.5

    def solve(self, start_node=0, time_limit=180):
        model = cp_model.CpModel()
        ids = list(self.spots.keys())
        
        # 変数定義
        # x[i,j]: i -> j 移動
        x = {(i, j): model.NewBoolVar(f'x_{i}_{j}') for i in ids for j in ids}
        visit = {i: model.NewBoolVar(f'visit_{i}') for i in ids}
        order = {i: model.NewIntVar(0, len(ids), f'order_{i}') for i in ids}
        
        # 在庫変数 (移動後の保持数)
        # y_hold[i]: i 訪問直後の櫓保持数
        y_hold = {i: model.NewIntVar(0, self.params["max_yagura"], f'y_{i}') for i in ids}
        r_hold = {i: model.NewIntVar(0, self.params["max_ring"], f'r_{i}') for i in ids}

        # 1. 経路制約
        # スタート地点
        model.Add(visit[start_node] == 1)
        model.Add(order[start_node] == 0)
        model.Add(y_hold[start_node] == 0)
        model.Add(r_hold[start_node] == 0)
        
        # フロー保存
        for i in ids:
            model.Add(sum(x[i, j] for j in ids) == 1).OnlyEnforceIf(visit[i])
            model.Add(sum(x[k, i] for k in ids) == 1).OnlyEnforceIf(visit[i])
            model.Add(sum(x[i, j] for j in ids) == 0).OnlyEnforceIf(visit[i].Not())
            # 自己ループの制御
            if i == start_node:
                model.Add(x[i, i] == 0) # スタートは必ず訪問
            else:
                model.Add(x[i, i] == 0).OnlyEnforceIf(visit[i])
                model.Add(x[i, i] == 1).OnlyEnforceIf(visit[i].Not())
        
        # サブツアー排除 (MTZ方式簡易版) - 緩和: 訪問する場合のみ順序制約
        for i in ids:
            for j in ids:
                if i != j and j != start_node:  # スタートに戻るエッジは除外
                    # x[i,j] -> order[j] > order[i] (差は1以上)
                    model.Add(order[j] >= order[i] + 1).OnlyEnforceIf(x[i, j])

        # 2. 在庫推移制約
        for i in ids:
            for j in ids:
                if i == j: continue
                
                # Startに戻るエッジ(j==start_node)は、在庫制約を免除する
                # (ミッション終了時に在庫が残っていてもOK)
                if j == start_node:
                    continue
                
                # i -> j 移動時
                spot_j = self.spots[j]
                
                # 櫓の増減
                req_y = spot_j.get("req_y", 0)
                gain_y = spot_j.get("gain_y", 0)
                
                # リングの増減
                req_r = spot_j.get("req_r", 0)
                gain_r = spot_j.get("gain_r", 0)
                
                # 移動条件: jで必要なリソースを持っていること
                # 持っている量 >= 必要量
                model.Add(y_hold[i] >= req_y).OnlyEnforceIf(x[i, j])
                model.Add(r_hold[i] >= req_r).OnlyEnforceIf(x[i, j])
                
                # 在庫更新ロジック修正: 上限(=max_hold)でクリップする
                # j後の在庫 = min(i後の在庫 - 消費 + 獲得, max_hold)
                
                # 櫓
                calc_y = model.NewIntVar(-10, 100, f'calc_y_{i}_{j}')
                model.Add(calc_y == y_hold[i] - req_y + gain_y).OnlyEnforceIf(x[i, j])
                
                max_y_var = model.NewConstant(self.params["max_yagura"])
                model.AddMinEquality(y_hold[j], [calc_y, max_y_var]).OnlyEnforceIf(x[i, j])

                # リング
                calc_r = model.NewIntVar(-10, 100, f'calc_r_{i}_{j}')
                model.Add(calc_r == r_hold[i] - req_r + gain_r).OnlyEnforceIf(x[i, j])
                
                max_r_var = model.NewConstant(self.params["max_ring"])
                model.AddMinEquality(r_hold[j], [calc_r, max_r_var]).OnlyEnforceIf(x[i, j])

        # 3. 攻略条件 (本丸)
        honmaru_id = 5
        targets = [1, 2, 3]
        
        
        # 王手条件: 3つ以上の陣取り
        # V-Goal狙いなので、ここでは「全ターゲット制圧」を条件とする
        # 手動: 3つ以上、自動: 3つ以上(2個入れ)
        # 簡易的に、required_for_vgoal=Trueのスポットは全て訪問必須とする
        pass_vgoal_spots = [visit[i] for i in ids if self.spots[i].get("required_for_vgoal", False)]
        if pass_vgoal_spots:
             model.Add(sum(pass_vgoal_spots) == len(pass_vgoal_spots))
        # V-Goal前提条件となるターゲット（必須ターゲットのみ）
        # 陣取り4は必須ではないので除外する
        vgoal_required_targets = [t for t in targets if self.spots[t].get("required_for_vgoal", False)]
        
        for t in vgoal_required_targets:
            model.Add(visit[t] == 1).OnlyEnforceIf(visit[honmaru_id])
            model.Add(order[t] < order[honmaru_id]).OnlyEnforceIf(visit[honmaru_id])

        # 4. 目的関数 (得点最大化 + 時間最小化)
        SCALE = 10
        # ミッション遂行時間の上限は競技時間(3分=180秒) + マージン
        # time_limit引数は「計算時間」の制限に使うため、ここでは定数を使う
        MISSION_MAX_TIME = 240
        total_time = model.NewIntVar(0, MISSION_MAX_TIME * SCALE, 'total_time')
        
        edge_costs = []
        for i in ids:
            for j in ids:
                if i != j:
                    t_move = self.dist(i, j) / self.params["robot_speed"]
                    t_action = self.spots[j]["action_time"]
                    cost = int((t_move + t_action) * SCALE)
                    edge_costs.append(x[i, j] * cost)
        
        model.Add(total_time == sum(edge_costs))
        
        # 得点計算
        # 各ターゲット訪問に大きな基礎点を与える (補給ポイントには基礎点を与えない)
        score_base = sum(visit[i] * (self.spots[i].get("score", 0) + 100) for i in ids if self.spots[i]["type"] in ["target", "honmaru"])
        
        # 補給ポイントの得点（例：アプローチ点）のみ加算
        score_supply = sum(visit[i] * self.spots[i].get("score", 0) for i in ids if self.spots[i]["type"] == "supply")
        
        score_jintori_bonus = sum(visit[t] for t in targets) * self.params["jintori_bonus"]
        score_vgoal = visit[honmaru_id] * self.params["vgoal_bonus"]
        
        # 訪問数を強く推奨 -> 削除（無駄な補給立ち寄りを防ぐため）
        # score_count = sum(visit[i] for i in ids) * 10 
        
        # スコアを大きく重み付けして決定
        # 最速王手優先: 時間ペナルティを大きくし、余分なアクション（陣取り4など）を防ぐ
        # V-Goal達成(必須) >>> 時間短縮 >> 余分なスコア
        
        # total_time (秒 * 10)
        # 1秒短縮 = 10ポイント改善
        # 陣取り1回 = 58ポイント
        # 時間ペナルティを重くして、58点稼ぐのに6秒以上かかるなら行かないようにする
        
        # 修正:
        # V-Goalボーナス: 1000 * 100 = 100000 (絶対優先)
        # 時間ペナルティ: total_time * 20 (1秒=200点相当の重み) 
        # -> スコア58点稼ぐために0.3秒しか使えない設定 -> 実質寄り道禁止
        
        model.Maximize(score_vgoal * 100 + score_jintori_bonus + score_base - total_time * 20)

        # Solve
        solver = cp_model.CpSolver()
        solver.parameters.max_time_in_seconds = float(time_limit)
        # solver.parameters.log_search_progress = True
        status = solver.Solve(model)
        
        if status in [cp_model.OPTIMAL, cp_model.FEASIBLE]:
            print(f"Solution Found: {solver.StatusName(status)}")
            path = []
            curr = start_node
            
            while True:
                path.append(curr)
                next_node = None
                for j in ids:
                    if j != curr and solver.Value(x[curr, j]):
                        next_node = j
                        break
                if next_node is None or next_node in path:
                    break
                curr = next_node
            
            # 結果表示
            print("\n=== 最適化ルート ===")
            t_sum = 0
            total_points = 0
            
            for k in range(len(path)-1):
                i, j = path[k], path[k+1]
                t_move = self.dist(i, j) / self.params["robot_speed"]
                t_act = self.spots[j]["action_time"]
                t_sum += t_move + t_act
                
                s_name = self.spots[j]["name"]
                s_point = self.spots[j].get("score", 0)
                
                # ボーナス計算 (簡易)
                if self.spots[j]["type"] == "target":
                    s_point += 30 # 陣取りボーナス
                
                total_points += s_point
                
                inv_y = solver.Value(y_hold[j])
                inv_r = solver.Value(r_hold[j])
                
                print(f" -> {s_name} (累計{t_sum:.1f}s) [+ {s_point}点] [残 櫓:{inv_y} 輪:{inv_r}]")
            
            print(f"\n合計予想得点: {total_points}点 (V-Goalボーナス除く)")
            return path
        else:
            print("No solution found")
            return []
