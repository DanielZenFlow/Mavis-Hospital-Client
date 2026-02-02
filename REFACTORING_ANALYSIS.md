# PriorityPlanningStrategy SRP 违反分析

## 当前状态
- **代码行数**: 3662行 (从4166行减少到3260行，然后又增加回来)
- **方法数量**: 72个私有方法 + 公共接口方法
- **已创建的Helper类**: 7个

## 严重违反SRP的职责分析

### 1. **主规划循环职责** (~150行)
- `search()` - 入口方法
- `planWithSubgoals()` - 主循环（包含超时检查、依赖分析、CBS切换等）
- **职责**: 整体规划流程控制
- **应保留**: 这是核心orchestrator职责

### 2. **搜索算法职责** (~800行) ⚠️ **可提取**
- `searchForSubgoal()` - A*搜索box目标
- `searchForAgentGoal()` - A*搜索agent目标
- `searchForDisplacement()` - 搜索位移路径
- `planBoxDisplacement()` - 规划box位移
- `reconstructPath()` - 重建路径
- `computeSubgoalHeuristic()` - 计算启发式
- **建议**: 创建 `BoxSearchPlanner.java`

### 3. **死锁检测与解决职责** (~300行) ⚠️ **可提取**
- `attemptCycleBreaking()` - 尝试打破循环依赖
- `findSafeDisplacementPosition()` - 寻找安全位移位置
- **建议**: 创建 `DeadlockBreaker.java` (已有DeadlockResolver但功能不同)

### 4. **Agent协调与让步职责** (~500行) ⚠️ **可提取**
- `tryIdleAgentClearing()` - 清理空闲agent
- `clearBlockingAgent()` - 清除阻塞agent
- `performProactiveYielding()` - 主动让步
- `findBestYieldPosition()` - 寻找最佳让步位置
- `setAgentYielding()` - 设置让步状态
- `clearYieldingForBeneficiary()` - 清除让步状态
- `forceYieldingAgentToMove()` - 强制让步agent移动
- **建议**: 创建 `AgentCoordinationManager.java`（增强现有的AgentYieldingManager）

### 5. **Greedy步进与行动选择职责** (~200行) ⚠️ **可提取**
- `tryGreedyStep()` - 尝试贪心步
- `tryGreedyStepWithMerging()` - 带合并的贪心步
- `findBestGreedyAction()` - 寻找最佳贪心动作
- `estimateAgentCost()` - 估计agent成本
- `tryRandomEscapeMove()` - 尝试随机逃逸
- **建议**: 创建 `GreedyPlanner.java`

### 6. **路径与位置分析职责** (~400行) ⚠️ **可提取**
- `findCriticalPositions()` - 寻找关键位置
- `findCriticalPositionsForAgentGoal()` - 寻找agent目标关键位置
- `findPathIgnoringDynamicObstacles()` - 忽略动态障碍物寻路
- `findParkingPosition()` - 寻找停车位置
- `isValidParkingPosition()` - 验证停车位置
- `planAgentPath()` - 规划agent路径
- `countPassableNeighbors()` - 计算可通行邻居
- `countFreeNeighbors()` - 计算自由邻居
- **建议**: 创建 `PathAnalyzer.java`

### 7. **Plan合并与冲突处理职责** (~250行) ⚠️ **可提取**
- `createJointActionWithMerging()` - 创建带合并的联合动作
- `addOtherAgentMoves()` - 添加其他agent动作
- `tryPreemptivePathClearing()` - 预防性路径清理
- `tryPushBoxOutOfWay()` - 推开阻挡box
- `applyJointAction()` - 应用联合动作
- **建议**: 创建 `PlanMerger.java`

### 8. **拓扑分析职责** (~300行) ⚠️ **可提取**
- `ensureTopologicalDepthsComputed()` - 确保拓扑深度已计算
- `computeTopologicalDepths()` - 计算拓扑深度
- `countBlockingBoxes()` - 计算阻塞box数量
- `computeBlockingScore()` - 计算阻塞分数
- `ensureReverseOrderComputed()` - 确保反向顺序已计算
- `computeReverseExecutionOrder()` - 计算反向执行顺序
- `computeCorridorDepth()` - 计算走廊深度
- `getReverseExecutionPriority()` - 获取反向执行优先级
- **建议**: 增强现有的 `TopologicalAnalyzer.java`

### 9. **Agent目标优先级判断职责** (~100行) ⚠️ **可提取**
- `wouldBlockHigherPriorityAgentGoal()` - 是否会阻塞更高优先级agent目标
- `getPositionsThatWouldBlockHigherPriority()` - 获取会阻塞更高优先级的位置
- **建议**: 创建 `AgentPriorityManager.java`

### 10. **辅助工具方法** (~200行) - 部分已提取
- `getDistance()` - 计算距离
- `wouldDisturbSatisfiedGoal()` - 是否会干扰已满足目标
- `isBoxOnPath()` - box是否在路径上
- `findBoxPosition()` - 寻找box位置
- `findTargetBoxPosition()` - 寻找目标box位置
- `findCorridorExitMove()` - 寻找走廊出口移动
- **部分已在PlanningUtils中，但还有遗漏**

## 重构优先级建议

### 🔴 高优先级（核心功能解耦）
1. **BoxSearchPlanner** (~800行)
   - 所有A*搜索相关方法
   - 与SubgoalSearcher合并/重构

2. **AgentCoordinationManager** (~500行)
   - 所有agent让步、清理、协调相关方法
   - 增强现有AgentYieldingManager

3. **PathAnalyzer** (~400行)
   - 所有路径分析、位置查找相关方法

### 🟡 中优先级（辅助功能提取）
4. **DeadlockBreaker** (~300行)
   - 死锁检测和打破相关方法

5. **PlanMerger** (~250行)
   - 计划合并、动作协调相关方法

6. **GreedyPlanner** (~200行)
   - 贪心步进和动作选择相关方法

### 🟢 低优先级（优化现有）
7. 增强 **TopologicalAnalyzer** (补充~300行)
8. 创建 **AgentPriorityManager** (~100行)
9. 补充 **PlanningUtils** (补充遗漏的工具方法)

## 预期重构结果
- **目标**: PriorityPlanningStrategy减少到 ~800-1000行
- **职责**: 仅保留orchestration逻辑（主循环、超时检查、策略切换、依赖分析触发）
- **新增Helper类**: 6-9个
- **总代码量**: 预计总行数不变或略增，但每个类职责单一清晰

## 当前问题总结
PriorityPlanningStrategy承担了至少**10个主要职责**：
1. 主规划流程控制 ✅ (保留)
2. A*搜索算法 ❌ (应提取)
3. 死锁打破 ❌ (应提取)
4. Agent协调 ❌ (应提取)
5. 贪心规划 ❌ (应提取)
6. 路径分析 ❌ (应提取)
7. 计划合并 ❌ (应提取)
8. 拓扑分析 ❌ (部分提取，需增强)
9. 优先级管理 ❌ (应提取)
10. 工具方法 ❌ (部分提取，需完善)

**结论**: 是的，严重违反SRP！至少还需要提取6-9个helper类。
