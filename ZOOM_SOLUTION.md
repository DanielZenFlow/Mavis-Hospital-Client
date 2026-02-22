# ZOOM 关卡解法详细记录

## 目录

1. [关卡概述与拓扑结构](#1-关卡概述与拓扑结构)
2. [核心难点分析](#2-核心难点分析)
3. [解题流程总览](#3-解题流程总览)
4. [跨色障碍检测系统（CrossColorBarrierAnalyzer）](#4-跨色障碍检测系统)
5. [全视野停车预分配系统（Pre-Assignment）](#5-全视野停车预分配系统)
6. [BSP 重试与替代目标机制](#6-bsp-重试与替代目标机制)
7. [走廊瓶颈检测（Corridor Bypass Check）](#7-走廊瓶颈检测)
8. [通行保护机制（Transit Protection）](#8-通行保护机制)
9. [障碍延伸候选点生成（Barrier Extension Candidates）](#9-障碍延伸候选点生成)
10. [问题迭代修复过程](#10-问题迭代修复过程)
11. [最终执行流程](#11-最终执行流程)
12. [回归测试结果](#12-回归测试结果)
13. [代码变更清单](#13-代码变更清单)

---

## 1. 关卡概述与拓扑结构

### 1.1 关卡基本信息

- **关卡名称**: ZOOM
- **尺寸**: 25 行 × 19 列
- **智能体**: 3 个
  - Agent 0 (RED): 初始位置 (18, 14)，负责搬运 Z/O/M/H/E/R/W/I/N 等红色箱子
  - Agent 1 (CYAN): 初始位置 (15, 12)，负责搬运 O/A 等青色箱子
  - Agent 2 (CYAN): 初始位置 (15, 13)，负责搬运 O/A 等青色箱子
- **箱子**: 48 个 A 箱子（CYAN）+ 多个红色/蓝色字母箱子
- **目标**: 在上方区域拼写 "HERE"（第8行）和 "ZOOM"（第19行）

### 1.2 地图结构（ASCII 表示）

```
行号  地图内容
 0      +++++++++++++++
 1    +++++++++++++++++++
 2    +ZOOM+++++AAAAAAAA+    ← A箱子墙（cols 10-17）
 3    +MOOZ+++++AAAAAAAA+
 4    ++++++++++AAAAAAAA+
 5    +         AAAAAAAA+    ← 第5行：关键的水平走廊
 6    +    +++++AAAAAAAA+
 7    +HERE+++++AAAAAAAA+    ← HERE字母（初始位置）
 8    +    +++++AAAAAAAA+    ← HERE目标位置
 9    +++++++++++++ +++++    ← ★ GAP在(9,13)：唯一南北通道
10    +        +        +    ← 第10行：右侧迷宫的水平连接器
11    + WINNER + ++++++ +    ← (11,10)和(11,17)是仅有的自由格
12    +        +  + + + +    ← 手指走廊开始
13    +        +  + + + +
14    +        +  +     +    ← (14,12)是墙，分隔col10-11和col13-17
15    ++++++++++  12+++++    ← Agent 1&2 在此
16    +        +        +
17    +        +++++++  +
18    +        +     0  +    ← Agent 0 在此
19    +        +   ZOOM +    ← ZOOM目标位置
20    ++++     +  +++++++
21    +GG+     +        +
22    +EZ+     Z        +
23    ++++     +        +
24    +++++++++++++++++++
```

### 1.3 关键拓扑特征

**GAP（缺口）**: 全关卡最关键的位置 — `(9, 13)`，这是上半部分和下半部分之间**唯一的通道**。48个 A 箱子（第2-8行，第10-17列）完全封堵了这个通道。

**右侧迷宫结构** (行10-15, 列10-17): 约31个自由格，由多条"手指走廊"构成：
- **第10行**: 完全开放（列10-17），是所有走廊的**唯一水平连接器**
- **第11行**: 仅 `(11,10)` 和 `(11,17)` 自由 — 关键的垂直连接点
- **第12-13行**: 交替格局 — `(x,10)`, `(x,11)` 自由, `(x,12)` 墙, `(x,13)` 自由, `(x,14)` 墙, `(x,15)` 自由, `(x,16)` 墙, `(x,17)` 自由
- **第14行**: `(14,10-11)` 自由, `(14,12)` **墙**（将 col 10-11 区域与 col 13-17 区域在第10行以下完全隔断）
- **第15行**: `(15,10-11)` 自由，Agent 1&2 在 `(15,12)` 和 `(15,13)`

**第5行走廊** (列5-9): 一条无旁路走廊 — 第4行（列5-9）和第6行（列5-9）均为墙壁，任何被停放在此处的箱子都会永久封锁通道。但列1-4区域在第6行有南向旁路。

**col 10-11 走廊** 和 **col 13-17 走廊**：这两个走廊群**仅通过第10行相连**。如果第10行被箱子阻塞，Agent 0 将无法从下方到达 gap。

---

## 2. 核心难点分析

### 2.1 跨色障碍问题

Agent 0（RED）需要通过 gap `(9,13)` 到达上方区域完成 HERE 和 ZOOM 目标。但 gap 被 7 个 CYAN 颜色的 A 箱子阻塞（沿 L 形路径排列）:

```
L形障碍清除顺序（从gap入口向外）：
(8,13) → (7,13) → (6,13) → (5,13) → (5,12) → (5,11) → (5,10)
```

**颜色约束**: Agent 0 是 RED，**不能**操作 CYAN 的 A 箱子。必须由 CYAN 的 Agent 1 或 Agent 2 来清除。

### 2.2 停车空间极度有限

右侧迷宫（行10-15，列10-17）是清除后 A 箱子的唯一停放区域，但该区域：
- 总共仅约31个自由格
- 被手指走廊分割成多个狭窄区域
- 每条走廊仅1格宽，放置一个箱子即完全堵死
- 需要同时停放7个箱子 + 保留 Agent 0 的通行路径

### 2.3 Agent 0 被困风险

这是**最关键也最难解决的问题**：清除 A 箱子后，7个箱子被停放到右侧迷宫。如果停放位置选择不当，会堵死 Agent 0 从下方（第18行）经由右侧迷宫到达 gap `(9,13)` 的所有路径。

Agent 0 从`(18,14)` 到 gap 的必经路径：
```
(18,14) → ... → (15,10) → (14,10) → (13,10) → (12,10) → (11,10) → (10,10)
→ (10,11) → (10,12) → (10,13) → (9,13) [GAP]
```

任何堵在 `(13,10)`, `(11,10)`, `(12,11)` 等关键位置的箱子都会切断这条路径。

---

## 3. 解题流程总览

ZOOM 关卡的完整求解分为 **四个阶段**：

```
阶段1: 跨色障碍检测
  CrossColorBarrierAnalyzer.analyzeBarriers() → 发现7个A箱子构成的L形障碍

阶段2: 障碍清除（Barrier Clearing）
  2a. 全视野停车预分配 → 为7个箱子各指定一个安全停车位
  2b. 逐个清除（BSP规划） → 用Agent 1/2 将箱子拉/推出barrier路径
  2c. BSP重试（第7个箱子） → 预分配目标失败，使用替代目标机制
  2d. 二次清除 → 将停在gap上的箱子移到迷宫深处

阶段3: 子目标规划（Subgoal Planning）
  PriorityPlanning 按拓扑序逐个规划子目标：
  - Agent 0 通过gap到达上方
  - 完成 HERE 目标（行8，列1-4）
  - 完成 ZOOM 目标（行19，列1-4）
  - A箱子回填（将之前清出的A箱子推回目标位置）

阶段4: 联合行动输出
  将所有行动合并为 joint-action 序列发送给服务器
```

**最终结果**: 711 步，86.5 秒求解

---

## 4. 跨色障碍检测系统

### 4.1 CrossColorBarrierAnalyzer

`CrossColorBarrierAnalyzer` 是一个静态分析器，在规划开始前检测跨色障碍。

**核心方法**: `analyzeBarriers(State, Level, List<Subgoal>)`

**检测逻辑**:
1. 对每个智能体，BFS 计算其可达区域（将其他颜色的箱子视为不可通过的障碍）
2. 检查哪些目标位置不在可达区域内
3. 如果存在不可达目标，回溯找到阻塞路径上的**最小箱子集合**
4. 构建 `Barrier` 对象，包含清除顺序

**Barrier 数据结构**:
```java
public static class Barrier {
    public final int blockedAgentId;        // 被阻塞的智能体ID (Agent 0)
    public final Color blockedAgentColor;    // 被阻塞智能体的颜色 (RED)
    public final List<Position> unreachableGoals;  // 不可达的目标位置
    public final List<Character> goalBoxTypes;      // 目标位置上的箱子类型
    public final List<Position> clearingOrder;      // 清除顺序（从gap入口开始）
    public final char blockingBoxType;       // 阻塞箱子类型 ('A')
    public final Color blockingColor;        // 阻塞箱子颜色 (CYAN)
}
```

### 4.2 清除智能体选择

`findAllClearingAgents()` 方法：
1. 遍历所有智能体，筛选与阻塞箱子**同色**的
2. 用 BFS 计算每个候选智能体到第一个障碍箱子的距离
3. 按距离排序返回（最近优先）
4. 跳过 `permanentlyFailedBarriers` 中记录的永久失败智能体

在 ZOOM 中：Agent 1 (距离近) 和 Agent 2 都是 CYAN，均可清除 A 箱子。

### 4.3 提取可行性预检查

`isBoxExtractable()` 方法在清除开始前验证第一个障碍箱子是否**物理上可以被拉出**。检查条件：
- 箱子至少有一个空闲的相邻格可供智能体站位
- 从该站位格方向拉出后，箱子有路径可离开障碍区域
- 不是死端走廊中的不可移动箱子

如果不可提取 → 将障碍缓存入 `skippedBarrierClearingOrders`，跳过。

---

## 5. 全视野停车预分配系统

### 5.1 问题背景

在之前的实现中，用的是**即时分配**策略：每清除一个箱子时才选择停车位。这导致：
- 前面的箱子占据了好位置
- 后面的箱子无处可停
- 清除智能体可能在清除中途失去与后续箱子的连通性

### 5.2 预分配算法

`preAssignParkingSlots()` 方法在清除**任何**箱子之前，为**所有** 7 个箱子预先分配停车位。

**算法流程**:

```
输入：clearingOrder = [(8,13), (7,13), (6,13), (5,13), (5,12), (5,11), (5,10)]
      candidates = 按距离排序的停车候选位置列表
      blockedAgentPos = Agent 0 的位置 (18,14)
      gapExitCell = gap出口格 (10,13)

递归回溯搜索：
  对于每个箱子 i = 0..6:
    对于每个候选位置 c:
      1. 模拟: 箱子从 clearingOrder[i] 移到 c
      2. 连通性检查: 智能体从 c 能否到达下一个箱子 clearingOrder[i+1]?
      3. 如果连通:
         递归分配下一个箱子 (i+1)
         如果全部分配成功 → 返回
         否则回溯，尝试下一个候选
      4. 如果全部 7 个箱子分配完毕:
         ★ 通行验证: BFS 检查 Agent 0 (18,14) → gapExitCell (10,13) 是否可达
         如果可达 → 返回成功
         否则 → 回溯
```

### 5.3 连通性验证

`bfsConnectedAfterParking()` 方法：
- 模拟箱子已停放到候选位置
- 将**后续障碍箱子**从障碍集中移除（因为它们还在原位但即将被清除）
- BFS 从候选位置的邻格出发，检查是否能到达下一个需清除的箱子

### 5.4 通行保护（Transit Protection）

这是**本次修复新增**的关键检查。在 `doPreAssignRecursive` 的终止条件中：

```java
if (idx >= clearingOrder.size()) {
    // 所有箱子分配完毕 — 验证被阻塞的智能体仍可到达gap
    if (blockedAgentPos != null && gapExitCell != null) {
        Set<Position> effectiveObs = new HashSet<>(obstacles);
        for (Position bp : clearingOrder) {
            effectiveObs.remove(bp); // 障碍箱子已被清除
        }
        return bfsCanReach(blockedAgentPos, gapExitCell, effectiveObs, level);
    }
    return true;
}
```

**作用**: 即使预分配的 7 个位置每一步都保持了清除智能体的连通性，但如果组合起来恰好堵死了 Agent 0 从下方经由右侧迷宫到 gap 的路径，就必须回溯寻找其他分配方案。

### 5.5 退化策略

如果无法为全部 7 个箱子找到安全分配：
- 逐步减少 `maxBoxes` (6, 5, 4...)
- 返回最大可行前缀的分配方案
- 如果连 1 个都分配不了 → 跳过此障碍

---

## 6. BSP 重试与替代目标机制

### 6.1 问题背景

预分配为第 7 个箱子（`(5,10)`，barrier 的最远端）指定的停车位是 `(13,15)` — 这个位置在右侧迷宫深处的一条手指走廊中。

BSP（BoxSearchPlanner）尝试规划从 `(5,10)` 到 `(13,15)` 的推箱路径时**失败**了：
- 路径需要经过 gap `(9,13)` → 进入第10行 → 沿手指走廊下行
- 需要绕过已停放的 6 个 A 箱子
- 搜索空间在 256,000 个状态内无法找到解

原始代码的问题：**BSP 失败后没有重试机制**，直接放弃清除。

### 6.2 重试机制设计

当 BSP 第一次失败后（包括 full-unfreeze 重试也失败），启动**替代目标搜索**：

```
BSP失败 [预分配目标(13,15)]
    ↓
全解冻重试失败 [unfreeze所有barrier位置]
    ↓
★ 替代目标搜索 ★
    ↓
1. 构建宽松的避免集（retryAvoid）
2. 获取新的候选位置列表（最多30个）
3. 追加障碍延伸候选点
4. 对每个候选位置：
   a. 走廊瓶颈检查（noBypassCells）
   b. 通行验证（bfsCanReach → gapExitCell）
   c. 如果通过 → BSP尝试规划
   d. 成功 → 使用此目标，退出循环
5. 最多尝试8个通过检查的候选
```

### 6.3 宽松避免集

对于**最后一个箱子**（`isLastBox = true`），避免集从严格放松为最小限度：

```java
Set<Position> retryAvoid = new HashSet<>();
retryAvoid.addAll(barrier.clearingOrder);  // 障碍位置始终排除
retryAvoid.addAll(usedParkings);           // 已使用的停车位排除
if (!isLastBox) {
    retryAvoid.addAll(baseAvoidPositions); // 非最后箱子：保持完整保护
}
// 最后一个箱子: 不再保护gap出口行和接近走廊
// 因为后面没有更多箱子需要清除了
```

**关键洞察**: 对最后一个箱子，gap 出口行第10行不再需要保护（没有后续箱子需要经过它到达），接近走廊也不需要保护。这大幅扩展了候选空间。

### 6.4 候选位置排序

```java
List<Position> retryCandidates = CrossColorBarrierAnalyzer.findParkingPositions(
        30, updReach, currentState, level, retryAvoid, boxPos);
```

候选按**到箱子当前位置的距离**排序（最近优先），而非到 gap 的距离。这确保 BSP 路径尽可能短。

---

## 7. 走廊瓶颈检测

### 7.1 问题定义

在 ZOOM 的第5行（列5-9），走廊两侧（第4行和第6行）都是墙：

```
行4: ++++++++++AAAA...
行5: +         AAAA...   ← 走廊
行6: +    +++++AAAA...
```

列5-9的每个格子的南北方向都被墙封锁，这意味着：
- 如果一个 A 箱子被推到 `(5,7)` 之类的位置
- Agent 0 无法从该格的南北方向绕过
- Agent 0 被永久阻塞在一侧

### 7.2 检测算法

```java
Set<Position> noBypassCells = new HashSet<>();
if (barrier.clearingOrder.size() >= 2) {
    Position lastBox = barrier.clearingOrder.get(barrier.clearingOrder.size() - 1);
    Position prevBox = barrier.clearingOrder.get(barrier.clearingOrder.size() - 2);
    int dr = lastBox.row - prevBox.row;  // 障碍延伸方向（行增量）
    int dc = lastBox.col - prevBox.col;  // 障碍延伸方向（列增量）
    Position ext = lastBox;
    for (int step = 0; step < 20; step++) {
        ext = Position.of(ext.row + dr, ext.col + dc);
        if (level.isWall(ext)) break;
        // 检查垂直方向是否有旁路
        boolean hasBypass;
        if (dr == 0) { // 水平延伸 → 检查南北
            hasBypass = !level.isWall(Position.of(ext.row - 1, ext.col))
                     || !level.isWall(Position.of(ext.row + 1, ext.col));
        } else { // 垂直延伸 → 检查东西
            hasBypass = !level.isWall(Position.of(ext.row, ext.col - 1))
                     || !level.isWall(Position.of(ext.row, ext.col + 1));
        }
        if (!hasBypass) {
            noBypassCells.add(ext); // 标记为不可绕过的瓶颈
        } else {
            break; // 遇到第一个有旁路的格子 → 走廊打开了
        }
    }
}
```

**在 ZOOM 中的效果**:

障碍最后两个箱子是 `(5,11)` 和 `(5,10)`，延伸方向为`(0, -1)`（向西）。

从 `(5,10)` 继续向西检查：
- `(5,9)`: 北 `(4,9)` 是墙, 南 `(6,9)` 是墙 → **noBypass** ✓
- `(5,8)`: 北 `(4,8)` 是墙, 南 `(6,8)` 是墙 → **noBypass** ✓
- `(5,7)`: 北 `(4,7)` 是墙, 南 `(6,7)` 是墙 → **noBypass** ✓
- `(5,6)`: 北 `(4,6)` 是墙, 南 `(6,6)` 是墙 → **noBypass** ✓
- `(5,5)`: 北 `(4,5)` 是墙, 南 `(6,5)` 是墙 → **noBypass** ✓
- `(5,4)`: 北 `(4,4)` 是墙, 南 `(6,4)` 不是墙 → **hasBypass** → 停止

所以 `noBypassCells = {(5,9), (5,8), (5,7), (5,6), (5,5)}`。

候选位置如果落在这些格子上，会被立即拒绝，不需要浪费 BSP 搜索预算。

---

## 8. 通行保护机制

### 8.1 GAP 出口通行检查

对每个通过走廊瓶颈检查的候选位置，执行**通行验证**：

```java
if (blockedPos != null && gapExitCell != null) {
    Set<Position> simObs = new HashSet<>(baseRetryObs);
    simObs.add(altTarget);    // 箱子停放在此处
    simObs.remove(boxPos);     // 箱子离开当前位置
    if (!bfsCanReach(blockedPos, gapExitCell, simObs, level)) {
        continue; // 拒绝此候选：会堵死Agent 0
    }
}
```

**模拟逻辑**:
1. `baseRetryObs` = 当前所有箱子的位置 - 所有 barrier 位置（barrier 箱子已被清除）
2. 加入 `altTarget`（模拟箱子停放到此处）
3. 移除 `boxPos`（箱子已从此处离开）
4. BFS 检查 Agent 0 从 `(18,14)` 到 `gapExitCell (10,13)` 是否可达

### 8.2 bfsCanReach 辅助方法

```java
private boolean bfsCanReach(Position start, Position target,
                             Set<Position> obstacles, Level level) {
    if (start.equals(target)) return true;
    Set<Position> visited = new HashSet<>();
    Queue<Position> queue = new LinkedList<>();
    visited.add(start);
    queue.add(start);
    while (!queue.isEmpty()) {
        Position current = queue.poll();
        for (Direction dir : Direction.values()) {
            Position next = current.move(dir);
            if (next.equals(target)) return true;
            // 也检查邻接（agent需要在gap旁边，不一定在gap上）
            for (Direction d2 : Direction.values()) {
                if (next.move(d2).equals(target)) {
                    if (!level.isWall(next) && !obstacles.contains(next)) return true;
                }
            }
            if (visited.contains(next)) continue;
            if (level.isWall(next)) continue;
            if (obstacles.contains(next)) continue;
            visited.add(next);
            queue.add(next);
        }
    }
    return false;
}
```

**特殊处理**: 不仅检查 `next == target`，还检查 `next` 的邻格是否等于 `target`。这是因为 Agent 0 不需要**站在** gap 上，只需要能到达 gap 的**相邻格**就能继续规划。

### 8.3 为什么不检查目标可达性（之前失败的方案）

在第三次迭代中，我们尝试了直接检查 Agent 0 到所有 `unreachableGoals`（HERE 的目标 `(8,1-4)`）的可达性。但这个检查**过于严格**：

- HERE 目标在 `(8, 1-4)`
- 但 `(7, 1-4)` 处有可推动的 HERE 箱子
- Walk-only BFS 将箱子视为不可通过的障碍
- 因此 BFS 认为 `(8,1-4)` 不可达
- **所有 30+ 个候选位置全部被拒绝**

教训：通行检查应该检查**到 gap 出口的可达性**（纯空间连通），而非到最终目标的可达性（需要考虑推箱操作）。

---

## 9. 障碍延伸候选点生成

### 9.1 问题背景

`bfsReachableForClearing()` 使用 walk-only BFS——如果一个箱子堵在走廊里，BFS 无法穿过箱子到达另一侧的空格。

在 ZOOM 中，`(5,10)` 处的 A 箱子阻塞了向西的走廊。Walk-BFS 从智能体位置出发，无法到达 `(5,9)`, `(5,8)` 等格子。因此这些格子不会出现在 `findParkingPositions` 返回的候选列表中。

但实际上，通过 **BSP 推箱规划**，智能体可以将箱子从 `(5,10)` 向西推到 `(5,4)` — 这些格子是 "推可达但不可步行到达" 的。

### 9.2 延伸候选生成算法

```java
if (barrier.clearingOrder.size() >= 2) {
    Position lastBox = barrier.clearingOrder.get(barrier.clearingOrder.size() - 1);
    Position prevBox = barrier.clearingOrder.get(barrier.clearingOrder.size() - 2);
    int bdr = lastBox.row - prevBox.row;  // 延伸方向
    int bdc = lastBox.col - prevBox.col;
    Position ext = lastBox;
    for (int step = 0; step < 20; step++) {
        ext = Position.of(ext.row + bdr, ext.col + bdc);
        if (level.isWall(ext)) break;
        if (retryAvoid.contains(ext)) continue;
        if (retryCandidates.contains(ext)) continue;
        if (level.getBoxGoal(ext) != '\0') continue;       // 不占目标格
        if (level.getAgentGoal(ext.row, ext.col) != -1) continue;  // 不占agent目标
        if (!currentState.hasBoxAt(ext) || ext.equals(boxPos)) {
            retryCandidates.add(ext);
        }
    }
}
```

**在 ZOOM 中的效果**:
- `lastBox = (5,10)`, `prevBox = (5,11)`
- 延伸方向: `(0, -1)` (向西)
- 生成候选: `(5,9)`, `(5,8)`, `(5,7)`, `(5,6)`, `(5,5)`, `(5,4)`, `(5,3)`, `(5,2)`, `(5,1)`
- 但 `(5,9)` 到 `(5,5)` 被走廊瓶颈检查拒绝 (noBypassCells)
- `(5,4)` 有旁路 → 可以通过走廊检查

### 9.3 与走廊检查的协同

候选生成和走廊检查**紧密配合**：

1. 延伸候选提供了 walk-BFS 无法发现的目标
2. 走廊检查过滤掉了那些会造成永久阻塞的位置
3. 通行检查再次确认 Agent 0 的路径不被切断
4. 最终结果：只有真正安全且 BSP 可达的目标被尝试

---

## 10. 问题迭代修复过程

整个 ZOOM 求解经历了 **4 轮迭代修复**，每轮都揭示了更深层的问题：

### 第一轮：BSP 无重试机制

**现象**: 前6个箱子成功清除，第7个箱子 `(5,10)` BSP 规划到 `(13,15)` 失败。代码直接放弃。

**根因**: 预分配目标 `(13,15)` 在手指走廊深处，被6个已停放的箱子包围，BSP 在 256K 状态内找不到路径。没有备选方案。

**修复**: 添加 BSP 重试机制——失败后尝试 `findParkingPositions()` 返回的备选目标。

**结果**: 第7个A箱子成功清除！重试目标 `(10,10)` 仅需 25 步。6/7 → **7/7 清除**。

**但新问题出现**: 关卡未解决——所有子目标规划失败。

### 第二轮：Agent 0 被停放箱子困住

**现象**: 7/7 清除成功，但子目标规划全部失败。

**根因分析**:
- 第7个箱子最初清除到 `(10,10)`
- 但 `(10,10)` 在第10行，需要二次清除（secondary clearing）
- 二次清除将箱子从 `(10,10)` 移到 `(12,11)`
- `(12,11)` 堵死了 col 11 的垂直通道
- 之前的箱子已堵死 `(13,10)` (col 10通道) 和 `(11,17)` (col 17通道)
- Agent 0 在第15行以下，**所有**通往第10行的路径都被堵死
- Agent 0 无法到达 gap → 无法到达上方区域 → 所有目标失败

**修复**: 在 `preAssignParkingSlots` 中添加**通行保护**——递归终止时 BFS 验证 Agent 0 能否到达 gapExitCell。

**结果限制**: 在预分配时，col 11 路径 `(12,11)` 仍然是自由的（二次清除还没发生），所以预分配的通行检查合法通过。问题出在**二次清除**选择了不当的目标。需要在 BSP 重试阶段也加入通行检查。

### 第三轮：通行检查过于严格（全部候选被拒绝）

**现象**: BSP 重试阶段添加了通行检查，但检查的是 Agent 0 到 `unreachableGoals`（`(8,1-4)` HERE目标）的可达性。

**根因**: HERE 目标 `(8,1-4)` 前方有 HERE 箱子 `(7,1-4)` 阻挡。Walk-only BFS 无法穿过箱子。所以 BFS 认为所有目标均不可达。结果：30+ 个候选位置**全部被拒绝**。

**教训**: Walk-BFS 不能用来判断"推箱操作后才可达"的目标。应该只检查到 gap 出口的**空间连通性**。

### 第四轮：走廊旁路 + GAP 出口通行 + 障碍延伸（最终方案）

**修复组合**:

1. **走廊旁路检查**: 只排除确实无法绕过的瓶颈格子（noBypassCells），不在这些位置尝试 BSP
2. **GAP 出口通行检查**: 改为检查 Agent 0 到 `gapExitCell (10,13)` 的可达性（不是到最终目标），这是纯空间连通检查，不涉及推箱
3. **障碍延伸候选**: 将障碍方向延伸产生的"推可达"格子加入候选池

**结果**: 
- 箱子 `(5,10)` 的 BSP 重试目标变为 `(9,13)` — 就是 gap 本身！
- `(9,13)` 通过走廊检查（不在 noBypassCells 中）
- `(9,13)` 通过通行检查（Agent 0 仍可经由 col 11 路径到达 `(10,13)`）
- BSP 成功规划 `(5,10) → (9,13)`，仅21步
- 二次清除将箱子从 `(9,13)` 移到 `(15,13)` — 在迷宫深处，不影响 Agent 0 通行
- **ZOOM 关卡首次成功求解！711 步！**

---

## 11. 最终执行流程

### 11.1 障碍清除阶段（204 步）

| 顺序 | 箱子位置 | 停车位置 | 步数 | 备注 |
|------|---------|---------|------|------|
| 1 | (8,13) | (13,17) | 22 | 预分配目标 |
| 2 | (7,13) | (12,17) | 25 | 预分配目标 |
| 3 | (6,13) | (11,17) | 23 | 预分配目标 |
| 4 | (5,13) | (13,15) | 37 | 预分配目标 |
| 5 | (5,12) | (12,15) | 33 | 预分配目标 |
| 6 | (5,11) | (12,13) | 43 | 预分配目标 |
| 7 | (5,10) | **(9,13)** | **21** | ★ BSP重试成功（预分配目标失败） |

第7个箱子是关键突破：预分配目标 `(13,15)` BSP 失败，重试到 `(9,13)` 成功。

### 11.2 二次清除（19 步）

箱子从 gap `(9,13)` 被 Agent 1 移动到 `(15,13)`（迷宫深处），打通 gap 通道。

### 11.3 子目标规划阶段（~488 步）

Agent 0 经由清通的 gap 到达上方：
- 路径: `(18,14) → ... → (15,10) → (14,10) → (13,10) → (12,10) → (11,10) → (10,10) → (10,13) → (9,13) [GAP]`
- 完成 HERE 目标: 将 H/E/R/E 推到 `(8,1-4)`
- 完成 ZOOM 目标: 回到下方将 Z/O/O/M 推到 `(19,1-4)`
- A 箱子回填: 将之前清出的 A 箱子推回到第5-8行的 A 目标上

### 11.4 最终结果

```
Level solved: Yes
Actions used: 711
Time to solve: 86.476 seconds
Strategy: STRICT_ORDER (RANDOM#42 ordering)
```

---

## 12. 回归测试结果

| 关卡 | 基线步数 | 修改后步数 | 状态 |
|------|---------|-----------|------|
| SAbotbot | 63 | 63 | ✅ 无回归 |
| ClosedAI | 447 | 447 | ✅ 无回归 |
| MADS | 51 | 51 | ✅ 无回归 |
| MAGiC | 607 | 607 | ✅ 无回归 |
| **ZOOM** | **306 (未解)** | **711 (已解)** | 🎉 **新增求解** |

所有回归测试均通过，步数完全一致。ZOOM 从未解变为已解。

---

## 13. 代码变更清单

### 13.1 修改文件

**唯一修改文件**: `src/main/java/mapf/planning/strategy/PriorityPlanningStrategy.java`

| 变更 | 行号范围 | 增/删 | 描述 |
|------|---------|-------|------|
| BSP 重试循环 | ~753-895 | +143 行 | BSP 失败后的替代目标搜索、过滤、重试 |
| 通行保护（预分配） | ~618-647, ~1175-1190 | +30 行 | `blockedAgentPos`/`gapExitCell` 参数传递 + 终止条件中的 BFS 验证 |
| `bfsCanReach()` | ~1303-1333 | +31 行 | BFS 可达性辅助方法 |
| 方法签名变更 | ~1131, ~1175 | +4 行 | `preAssignParkingSlots` 和 `doPreAssignRecursive` 新增参数 |
| 调用点参数计算 | ~625-628 | +4 行 | 从 state/approachPath 计算 blockedAgentPos 和 gapExitCell |

**总计**: +220 行, -9 行

### 13.2 新增组件/机制

| 组件 | 类型 | 作用 |
|------|------|------|
| BSP 替代目标搜索 | 重试循环 | BSP 失败后搜索更近/更简单的停车目标 |
| 走廊瓶颈检测 (noBypassCells) | 过滤器 | 排除两侧都是墙的无旁路走廊格子 |
| GAP 出口通行检查 | 过滤器 | 验证候选目标不会切断 Agent 0 到 gap 的路径 |
| 障碍延伸候选生成 | 候选扩展 | 将障碍方向延伸的推可达格子加入候选池 |
| 预分配通行保护 | 回溯约束 | 预分配终止时验证 Agent 0 通行不被切断 |
| `bfsCanReach()` | 辅助方法 | 通用的 BFS 可达性检查（支持邻接判定） |

### 13.3 修复的不合理部分

| 原始问题 | 位置 | 修复方式 |
|----------|------|---------|
| BSP 失败无重试 | 清除循环 | 添加最多8次的替代目标重试 |
| 预分配不考虑被阻塞智能体通行 | `doPreAssignRecursive` | 终止条件增加 BFS 通行验证 |
| 候选列表缺少推可达位置 | 重试候选生成 | 障碍延伸候选点补充 |
| 无走廊瓶颈感知 | 重试过滤 | noBypassCells 集合预计算 + 快速过滤 |
| 最后一个箱子过度保护 | retryAvoid 构建 | 最后一个箱子放松gap出口行和接近走廊的保护 |

### 13.4 相关 Git 提交

```
3fb5ac5 BSP retry with transit-aware alternative targets + corridor bypass
        + barrier extension — ZOOM SOLVED (711 actions)
3c11c28 Full-horizon parking pre-assignment + multi-agent clearing fallback
        （前置提交：预分配系统基础实现）
```
