# 对话接力文档 - Hungarian Algorithm Fix Session

**日期**: 2026-02-11  
**分支**: `fix/hungarian-algorithm-trigger`  
**Commits**: `d3856ab` (trigger fix), `0be0908` (3-layer defense)  
**状态**: ⚠️ ClosedAI.lvl 仍未解决

---

## 一、原始问题

### 用户背景
- 研究生，Java基础弱，无数据结构背景，但有MAPF/Sokoban概念知识
- 正在通过"以修代学"方式学习这个Hospital MAPF项目
- 前一个对话分析了Hungarian算法的心智模型差异，并建议修改trigger condition

### 触发问题
过去能解决的关卡 **ClosedAI.lvl** 在引入Hungarian算法后失败了。用户问：
1. 支持Pull的Sokoban中Hungarian算法标准实践 vs 我们实现的差距
2. ClosedAI回归的代码错误在哪

---

## 二、已完成的修改

### Commit 1: `d3856ab` - Hungarian Trigger修复
```
fix: always compute Hungarian assignment regardless of goal dependencies
```

**旧逻辑**:
```java
if (goalDependsOn.isEmpty()) {
    subgoalManager.computeHungarianAssignment(state, level, completedBoxGoals);
} else {
    subgoalManager.invalidateHungarianCache();
}
```

**新逻辑**:
```java
// 始终计算Hungarian，无论是否有依赖关系
subgoalManager.computeHungarianAssignment(state, level, completedBoxGoals);
if (!goalDependsOn.isEmpty()) {
    logNormal("[PP] Goal dependencies detected — Hungarian computed (order enforced separately)");
}
```

**理由**: Hungarian负责Box-to-Goal ASSIGNMENT，LevelAnalyzer的topological sort负责EXECUTION ORDER。两者正交，不应相互禁用。

---

### Commit 2: `0be0908` - 三层防御体系

#### 问题根因
Hungarian的Cost矩阵使用 `BFS(box→goal)` 忽略动态障碍物（movable boxes），导致：
- 被其他箱子包围的stuck箱子仍被分配（如ClosedAI row12的`H(12,14)`）
- Agent物理上无法走到箱子旁边，但Hungarian仍认为"可达"
- BSP失败后没有重试机制，导致整个goal卡死

#### 修改详情

**L1: Cost矩阵过滤** ([HungarianBoxAssigner.java](src/main/java/mapf/planning/strategy/HungarianBoxAssigner.java#L102-L106))
```java
// 在computeAllAssignments中增加过滤
if (!isBoxMovable(boxPos, state, level)) continue;

private static boolean isBoxMovable(Position boxPos, State state, Level level) {
    for (Direction dir : Direction.values()) {
        Position neighbor = boxPos.move(dir);
        if (!level.isWall(neighbor) && !state.hasBoxAt(neighbor)) {
            return true; // 至少1个邻居是空的→pull可行
        }
    }
    return false; // 四面被墙/箱子包围→stuck
}
```
**防御**: 排除4邻居全被占用的stuck箱子，避免浪费assignment slot。

**L2: 物理可达性检查** ([SubgoalManager.java](src/main/java/mapf/planning/strategy/SubgoalManager.java#L347-L380))
```java
// 在getHungarianCandidate中增加BFS检查
if (!isAgentPhysicallyAdjacentReachable(agentPos, assignedBox, state, level)) {
    return null; // 拒绝Hungarian pick，fallback到greedy
}

private boolean isAgentPhysicallyAdjacentReachable(Position agentPos, Position boxPos, ...) {
    // BFS from agent，treating ALL boxes as obstacles (not just immovable)
    // 检查能否走到boxPos的任意相邻格子
}
```
**防御**: Hungarian的BFS忽略动态障碍，这里用物理BFS double-check。

**L3: BSP失败重试** ([PriorityPlanningStrategy.java](src/main/java/mapf/planning/strategy/PriorityPlanningStrategy.java#L1377-L1401))
```java
// Round 4: 如果Rounds 1-3全失败，尝试不同的箱子
Position retryBoxPos = subgoalManager.findBestBoxForGoalExcluding(
    subgoal, state, level, allSubgoals, completedBoxGoals, boxPos);
if (retryBoxPos != null && !retryBoxPos.equals(boxPos)) {
    // 用retryBox重试所有BSP rounds
}
```
**防御**: 即使L1/L2没拦住，BSP失败时还有第二次机会。

---

## 三、ClosedAI.lvl 关卡分析

### 关卡特征
- **Row 12关键区域**: 目标`HHIIJJ`(cols 1-6)在左死胡同，箱子`JHIIJH`(cols 9-14)在右侧，中间拥挤
- **Right-side cells**: cols 26/28/30是三条1格宽垂直死胡同，仅在row3连通，有严格的extraction/fill顺序依赖
- **多color交叉**: Cyan(H/I/J/K)有4种box type，分布在上区、左区、下区，Hungarian需协调跨区分配

### 为什么Hungarian会选错
1. H(12,14)虽然BFS距离短（被墙和箱子包围但BFS忽略箱子），Hungarian可能把它分配给goal(12,2)
2. H(12,10)（唯一物理可达的）被分配给goal(12,1)
3. 当topological order先执行goal(12,2)时，Hungarian返回H(12,14) → L2拦住 → greedy接管
4. **但greedy可能仍选不到最优box**（因为其他box被Hungarian"预留"），BSP失败
5. L3重试，但可能选到的也是次优box

---

## 四、当前状态

### ✅ 已验证
- 代码编译通过（`mvn compile -q` 成功）
- Git已提交两个commits在`fix/hungarian-algorithm-trigger`分支
- claude.md已更新（Design Decisions Log）

### ❌ 未验证
- **ClosedAI.lvl实际运行测试** - 用户报告"关卡还是没有解决"
- 没有运行regression test（`levels/SAbotbot.lvl`）
- 没有verbose log分析Hungarian选box的过程

---

## 五、可能的问题 & 下一步建议

### 可能残留的问题

#### 1. Greedy fallback的局限性
**问题**: L2拒绝Hungarian后，greedy按 `agentToBox + boxToGoal` 排序，但：
- 仍然用的是BFS距离（忽略动态障碍）
- 不考虑global assignment的残留影响（其他box被Hungarian"预留"导致的连锁反应）

**建议**: 在`findBestBoxForGoal`的greedy层增加物理可达性pre-filter，类似L2的BFS：
```java
// 在greedy candidate loop中
if (!isAgentPhysicallyAdjacentReachable(agentPos, boxPos, state, level)) continue;
```

#### 2. isAllocationFeasible的盲区
**问题**: `isAllocationFeasible`检查bipartite matching，但obstacles假设是`immovableBoxes + currentGoal`，忽略了：
- 其他同色movable boxes的blocking效应
- 窄走廊中的box-box blocking（e.g., row12的JHIIJH队列）

**建议**: 增强obstacles集合，或在Reachability BFS中动态考虑box density。

#### 3. Round 4重试的贪心性
**问题**: `findBestBoxForGoalExcluding`仅排除1个failed box，如果有3个boxes {B1, B2, B3}都次优，Round 4只尝试B2，没有尝试B3。

**建议**: Round 4可以loop多次，每次排除之前失败的所有boxes。

#### 4. Hungarian的Cost未考虑agent-box距离
**问题**: 当前Cost仅用 `BFS(box→goal)`，完全忽略agent→box的距离。这导致Hungarian可能选一个"离goal近但离agent远"的box。

**建议**: Cost矩阵改为 `agentToBox + boxToGoal`（但这需要per-agent Hungarian，不是per-boxType）。或者在validation层（L2）增加agent-box距离的penalty。

---

## 六、Debug建议

### 立即执行
```bash
# 1. 运行ClosedAI.lvl，开启verbose log
java -jar server.jar -l complevels/ClosedAI.lvl \
  -c "java -Xmx4g -cp target/classes mapf.client.Client" \
  -g -s 500 -t 180

# 设置环境变量启用verbose
$env:LOG_LEVEL="VERBOSE"
```

### 关键log分析
1. **Hungarian assignment**: 看`[SubgoalManager] Hungarian pre-assignment`和`Using Hungarian assignment for X -> Y: box at Z`
2. **L2拦截**: 看`physically unreachable by agent — falling back to greedy`
3. **Round 4触发**: 看`[PP] Round 4 (retry with different box)`
4. **BSP失败pattern**: 看`All rounds exhausted for X -> Y result=null`

### 如果仍失败
考虑回退策略：
- 试试`OrderingMode.DISTANCE_GREEDY`（绕过topological sort）
- 试试完全禁用Hungarian（用环境变量或config flag）验证是否Hungarian引入了regression

---

## 七、代码架构提醒

```
Hungarian Assignment Flow:
1. computeAndCacheSubgoals (PP)
   └─> computeHungarianAssignment (SubgoalManager)
       └─> computeAllAssignments (HungarianBoxAssigner) ← L1在这里
           └─> isBoxMovable filter

2. planSubgoal (PP)
   └─> findBestBoxForGoal (SubgoalManager)
       └─> getHungarianCandidate ← L2在这里
           └─> isAgentPhysicallyAdjacentReachable
       └─> greedy fallback (if L2 rejects)
   
   └─> BSP Rounds 1-3 (frozen variants)
   
   └─> Round 4 retry ← L3在这里
       └─> findBestBoxForGoalExcluding
```

---

## 八、文件清单

### 修改的文件
1. `src/main/java/mapf/planning/strategy/PriorityPlanningStrategy.java`
   - Line ~389: `computeAndCacheSubgoals` - 移除conditional guard
   - Line 1377-1401: `planSubgoal` - 增加Round 4 retry

2. `src/main/java/mapf/planning/strategy/SubgoalManager.java`
   - Line 200-268: `findBestBoxForGoal` - unchanged logic
   - Line 280-315: `findBestBoxForGoalExcluding` - 新增方法
   - Line 317-365: `getHungarianCandidate` - 增加L2 physical accessibility check
   - Line 367-406: `isAgentPhysicallyAdjacentReachable` - 新增BFS方法

3. `src/main/java/mapf/planning/strategy/HungarianBoxAssigner.java`
   - Line 102-106: `computeAllAssignments` - 增加`isBoxMovable`过滤
   - Line 206-220: `isBoxMovable` - 新增helper方法

4. `claude.md`
   - Line 240-260: Design Decisions Log - 更新Hungarian always-on理由和3-layer defense

### 关键依赖
- `ImmovableBoxDetector.getDistanceWithImmovableBoxes()` - BFS距离（忽略movable boxes）
- `LevelAnalyzer.analyze()` - 提供topological order和goalDependsOn
- `BoxSearchPlanner.searchForSubgoal()` - BSP主搜索

---

## 九、理论背景

### Hungarian Algorithm标准实践 vs 我们的实现

| 维度 | 标准实践 | 我们（修复前） | 我们（修复后） |
|------|---------|--------------|--------------|
| Cost计算 | BFS + agent距离 + penalty | 仅BFS(box→goal) | 仅BFS(box→goal) ✅L1过滤stuck |
| Candidate过滤 | Agent可达 + box movable | 仅immovable过滤 | ✅isBoxMovable + ✅L2 physical BFS |
| 失败重试 | 换box重试 | 无 | ✅Round 4 retry |
| 动态重算 | 每完成goal | ✅有 | ✅保持 |

### 心智模型对比
- **外卖派单** (Hungarian理想): 骑手→订单的assignment，骑手可自由移动
- **华容道** (Pull-Sokoban现实): 滑块之间互相blocking，assignment受当前layout严格约束

我们的修复尝试在"外卖派单"模型上叠加"华容道"约束（L1/L2/L3），但可能还不够。

---

## 结论

**当前进度**: 70%  
**核心理念**: Hungarian做transport optimization，topological sort做execution sequencing，三层防御避免错误assignment  
**待验证**: ClosedAI.lvl实际运行  
**如果仍失败**: 考虑greedy层增强、Cost公式改进、或Hungarian vs Greedy的权衡策略

**下一步接手者请**:
1. 先运行ClosedAI.lvl + verbose log
2. 分析Hungarian选box的pattern
3. 如果L2频繁拒绝，考虑在greedy层也加物理BFS pre-filter
4. 如果BSP普遍失败，考虑Cost公式引入agent距离

---

**Git状态**:
```bash
git log --oneline -2
# 0be0908 fix: 3-layer defense against Hungarian misassignment (ClosedAI regression)
# d3856ab fix: always compute Hungarian assignment regardless of goal dependencies
```
