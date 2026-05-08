
---

## 1. 当前 Portfolio 分析

### 架构总览

当前 portfolio 的执行流程：

```
Client.searchWithFallback() [留5s安全缓冲]
  → PortfolioController.search()
      → LevelAnalyzer.analyze() → LevelFeatures
      → 独立分组检测 (BFS + union-find, 50%时间上限)
          → 若>1组: 各组独立投影State/Level, 独立buildStrategySequence, 独立solve
          → 全组成功 → 合并返回 | 部分成功 → 保存为bestPartial, 继续
      → System.gc()
      → buildStrategySequence() → 4分支:
          ① SINGLE_AGENT (≤8 box goals): SA(w=1, 40%) → SA(w=5, 60%)
          ② SINGLE_AGENT (>8 box goals): PP/GREEDY(40%) → PP/TOPO(35%) → PP/RAND(25%)
          ③ Cyclic + spiral (corridorRatio>0.5): FARTHEST(40%) → GREEDY(30%) → RAND#42(15%) → RAND#137(15%)
          ④ Cyclic + open (corridorRatio≤0.5): TOPO(35%) → GREEDY(35%) → RAND#42(15%) → RAND#137(15%)
          ⑤ No-cycle: TOPO(40%) → GREEDY(30%) → RAND#42(30%)
      → 顺序尝试, 第一个完整解直接返回
      → 如无完整解, 返回最长部分解
```

### 与标准实践的对比

| 标准portfolio算法特征 | 当前状态 | 评估 |
|---|---|---|
| **独立分组分解 (Independence Detection)** | ✅ 已实现 | BFS+union-find, 正确投影State/Level, 有50%时间限制 |
| **特征驱动策略选择** | ⚠️ 部分 | 只用了 `corridorRatio` 和 `hasCircularDependency`，缺少密度/agent数量/box数量维度 |
| **策略多样性** | ⚠️ 不足 | 所有多agent分支都只用 PP+不同排序，没有本质不同的算法；CBS/JointA* 已从顶层排除 |
| **Anytime 特性** | ⚠️ 部分 | 有 partial plan 保存 + early-exit，但**不会在已有部分解基础上继续改进** |
| **时间预算管理** | ✅ 合理 | 固定比例分配，budget fraction 制度，5s安全缓冲 |
| **资源自适应** | ⚠️ 部分 | 有 adaptive BSP budget (大地图 25K vs 小地图 80K)，但 `GC()` 是粗糙手段 |
| **重启策略 (restart-based)** | ❌ 缺失 | 不同排序模式只是换 ordering，PP 核心的 frozen/regress/trap 机制没变 |
| **任务分配优化** | ⚠️ 部分 | Hungarian 算法 + greedy 回退，但**只分配 1 个 box per goal**，不处理同色多box选择策略 |

### 关键缺陷

**缺陷1：所有策略本质相同 — 都是 PP + 不同排序**

`buildStrategySequence()` 的4个多agent分支的**唯一区别是 `OrderingMode`**。但 PP 的核心问题不是排序：
- BSP (BoxSearchPlanner) 一次只移一个 box，**不考虑其他agent的未来动作**
- ConflictResolver 只做**当步**冲突检测，不做**多步预测**
- AgentCoordinator 的 clearing 是贪心的，可能创造新的死锁

当一个 PP 排序因为 agent 互堵而失败时，换一个排序**大概率遇到同样的互堵**，只不过在不同的 subgoal 上。对于高密度、多agent紧耦合的level（TBSTANS1、DECrunchy、help），**排序多样性几乎没用**。

**缺陷2：没有 multi-agent path planning 能力**

CBS 和 JointA* 被排除在顶层策略外（理由是 CBS 不支持 push/pull）。但：
- CBS 的**高层框架**（constraint tree + conflict detection + split）对多agent场景是标准答案
- 真正的问题是**低层规划器** (`SpaceTimeAStar`) 只支持 agent 移动，不支持 box push/pull
- 正确做法：CBS 高层不变，低层替换为 "push/pull-aware space-time A*"

**缺陷3：没有增量式解决方案**

PP 成功解决 N 个 subgoal 后，在第 N+1 个失败时：
- `early-exit` 返回整个部分解
- 下一个 portfolio 尝试**从头开始**，之前解决的 N 个 subgoal 的成功经验全部丢失
- 标准实践：anytime portfolio 应该可以**在部分解基础上继续**

**缺陷4：immovable box 处理不完整**

TaskFilter 正确检测了 "无同色agent" 的 box 为 immovable，但：
- 这些 immovable box 被传入 PP 作为 `immovableBoxes` (treated as walls)
- 然而 BSP 的 `searchForSubgoal` 在生成后继状态时仍然会尝试 push/pull 这些不可移动的 box
- 在 pacMAn、donkeyK、EpicfAIl 等**大量 immovable box 构成迷宫**的关卡，这会浪费搜索预算

---

## 2. 解决剩余关卡需要什么

### 按难度分类的15个未解关卡

**第一类：Immovable-box 迷宫 (3关)**
| Level | 特征 | 当前结果 | 根因 |
|---|---|---|---|
| pacMAn | ~70 个 Z box (orange, 无orange agent) 构成 pac-man 迷宫, 4 agent 各推1 box | 574 partial | immovable box 当wall处理，但BSP不理解迷宫寻路 |
| donkeyK | H/S/L/D immovable box 构成平台游戏, 9 agent | 0 | 同上 + 5个brown agent同色竞争 |
| EpicfAIl | I/J/E immovable, 4 agent 在 14×20 紧凑空间 | 0 | 4个narrow corridor内需要精确多agent协调 |

**需要的改进**：将 immovable box **真正合并入 Level.walls**（在 `projectLevel` 或 `search()` 入口处），而不是作为运行时软约束。这样 BFS/A* 的 successor generation 天然不会尝试移动它们，搜索效率大幅提升。

**第二类：极端密度 / 紧耦合 (4关)**
| Level | 特征 | 当前结果 | 根因 |
|---|---|---|---|
| TBSTANS1 | 6 agent 在 15×15 螺旋迷宫 | 260 partial | agent 密度极高，PP 的单agent规划+冲突解决无法处理 |
| DECrunchy | 3 agent 在 14×15，18 goals，1-wide 中央通道 | 217 partial | agent 必须穿过1格宽通道交换位置 |
| help | 4 agent 在 ++柱阵迷宫，15 boxes, 1-wide 通道 | 114 partial | box 必须在1-wide通道中交换位置 |
| ISO | 10 agent 10种颜色, 35×45 复杂迷宫 | 1 partial | 10个独特颜色=无同色竞争但需大规模协调 |

**需要的改进**：
1. **Push/Pull-aware CBS**：高层 CBS 框架 + 低层用 BSP 代替 SpaceTimeAStar。这是解决 agent 互堵的唯一标准方法。
2. **Corridor reasoning**：当两个 agent/box 在 1-wide 通道中面对面时，需要专门的 "让路" 逻辑（谁退、退到哪）。

**第三类：大规模 (4关)**
| Level | 特征 | 当前结果 | 根因 |
|---|---|---|---|
| Lily | 50×50, 10 agent, 200+ boxes | 0 | 太大，BSP budget 25K 不够，PP 来不及做任何事 |
| Minchia | 50×50, 10 agent, 200+ P-box 边框 | 0 | P-box 边框是immovable，内部需精确协调 |
| PokeNOM | 50×50, 9 agent, 200+ pixel-art boxes | 0 | 同 Lily |
| GroupEZ | 51×50, 10 agent, F/U 100+ boxes, 3 blue agent | 415 partial | 大规模 + 同色竞争 |

**需要的改进**：
1. **immovable box → wall 融合**：pixel-art、边框等大量 box 一旦识别为满足条件或不可移动，直接吸收到 wall 中，搜索空间能缩小 10 倍
2. **更积极的 task 过滤**：如已满足的goal上的box不参与后续搜索
3. **分区策略**：对 50×50 地图做空间分区（上半/下半/左右），每个分区独立解，合并

**第四类：复杂协调 (4关)**
| Level | 特征 | 当前结果 | 根因 |
|---|---|---|---|
| TheGate | 49×47, 10 agent, 底部30行纯走廊迷宫 | 2097 partial | 底部走廊极深，agent 需穿越全深度 |
| BigSplit | 26×38, 10 agent, W-box 分隔左右 | 216 partial | 左螺旋 + 右货架，W-barrier 须破 |
| MArtians | 50×50, 10 agent, 5+2+2+1 分组 | 1625 partial | 5 agent 大组规划不出来 |
| Medibots | 29×39, 10 agent, 3对同色 | 0 | 同色 agent 对需任务分配 |
| NameHere | 48×48, 10 agent, 16 pink box 类型给2 pink agent | 0 | 2 pink agent 需处理16种box=高协调 |

**需要的改进**：
1. **多 agent 同色任务分配**：当 N 个同色 agent 需要处理 M 个同色 box 时，需要明确分工（哪个agent负责哪个区域的box）
2. **Barrier 识别 + 破拆计划**：BigSplit 的 W-box barrier 需要先清除再开始正常规划

### 推荐优先级

| 优先级 | 改进 | 影响关卡 | 工作量 | 预期收益 |
|---|---|---|---|---|
| **P0** | **Immovable box → wall 融合** | pacMAn, donkeyK, EpicfAIl, Lily, Minchia, PokeNOM, GroupEZ (7关) | 中 | 高：搜索空间缩小数倍 |
| **P1** | **同色 agent 任务分区** | donkeyK, BigSplit, GroupEZ, Medibots, NameHere (5关) | 中 | 中高：消除同色竞争 |
| **P2** | **Corridor reasoning** | TBSTANS1, DECrunchy, help, ISO (4关) | 高 | 中：需要重写部分 conflict resolution |
| **P3** | **Push/Pull-aware CBS** | 所有紧耦合 (4-6关) | 很高 | 高但风险高：需改造整个CBS低层 |
| **P4** | **Anytime portfolio (增量)** | 所有 partial 解 (8关) | 中 | 中：已有2097步的TheGate可能改进 |

**建议先做 P0 (immovable box → wall 融合)**，这是投入产出比最高的改进，影响 7 个未解关卡。