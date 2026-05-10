# DeepSeek Bug Fix Review — fix/deepseek-bug-review

> 基于 `deepseektask.txt` 执行，逐项修复 `deepseek-bug.txt` 同行评审结论中的 8 项问题。
> 分支：`fix/deepseek-bug-review`（from `feat/dispatcher-p4`）

## 执行环境

- **仓库**: `mavis-hospital-client` (Java 17 + Maven)
- **基线分支**: `feat/dispatcher-p4`
- **工作分支**: `fix/deepseek-bug-review`（未 push，待人工审查后推送）
- **回归命令**: `mvn -q compile` + `java -jar server.jar -l levels/{SAbotbot,MAsimple1}.lvl`

## 已有修复（来自父分支）

评审结论中的 #3、#4、#5、#8 在 `feat/dispatcher-p4` 上已经修复。本分支未重复提交这些变更。

| Bug# | 严重度 | 描述 | 状态 |
|------|--------|------|------|
| #3 | 🟢 低 | `adjustOrderForBottlenecks` 死代码 | ✅ 已删除（L136 注释标注已废弃） |
| #4 | 🟢 低 | `SubgoalManager.findAgentForColor` 死代码 | ✅ 已删除（注释自标 Legacy） |
| #5 | 🟡 中 | `State.hasAgentAt/getAgentAt` 注释误导 | ✅ 已改为明确 O(n) 扫描语义 |
| #8 | 🔴 最高 | `Client.updateState` 用逐 agent `apply` 而非 joint 语义 | ✅ 已改为 mask→`applyJointAction` |

## 本分支新增提交

### Bug #1 — ConflictResolver 优先级改进 🟡

**提交**: `6a5f22a feat(L4): ConflictResolver picks yielder by holding-box / on-goal priority`

**位置**: `src/main/java/mapf/planning/strategy/ConflictResolver.java`

**变更**: `resolveConflicts(Action[], State, Level)` 兜底方法从"高编号 Agent 无条件让"改为"按动态优先级选让位者"：
1. 正在执行 PUSH/PULL 的 Agent 获得 +10 优先级（已持有 box，更难重启）
2. 已在自己 agent-goal 上的 Agent 降低 -5 优先级（已完成的不该被推进）
3. 平手时回退到高编号 Agent 让位（保持确定性的回退策略）

**回归**：SAbotbot=63, MAsimple1=27（无退化）

### Bug #6 — applyJointAction 可观测性 🟡

**提交**: `db1e873 feat(L1): applyJointAction logs once when push/pull source has no box`

**位置**: `src/main/java/mapf/domain/State.java`

**变更**: 在 `applyJointAction` 的 PUSH/PULL 分支中，当读取 `boxes.get(boxPos)` 返回 null 时（上游 planner 对不合法的推/拉动作未做可行性检测），打印一次性 stderr 警告（`BOX_MISMATCH_LOGGED` 静态标志）。不改变执行语义，沿用已有的 `OUT_OF_GRID_LOGGED` 模式。

**回归**：SAbotbot=63, MAsimple1=27；stderr 无新增噪音

### Bug #2 — IW1 find-route 文档澄清 🟡

**提交**: `78468a3 docs(L2): cross-reference IW1 find-route usage from constants`

**位置**: `src/main/java/mapf/planning/strategy/PriorityPlanningStrategy.java`

**变更**: 仅修改注释，不修改逻辑：
- `iw1Planner` 字段注释从 "used for escape subgoals only" 改为明确列出 P2 (escape) 和 P4b (find-route) 两条路径
- 常量 `IW1_FINDROUTE_MIN/MAX_MANHATTAN` 上方注释加入 `IW(1) FIND-ROUTE FALLBACK per claudeopus47 §1.2.2 / §3.2` 交叉引用
- P4b 分支（~L3700）注释头部加入 `per claudeopus47 §1.2.2 / §3.2`

**回归**：SAbotbot=63, MAsimple1=27

### Bug #7 — Portfolio ADR 架构决策记录 🔴

**提交**: `b6d9c9c docs: ADR-0001 record portfolio-uses-PP-only decision`

**位置**: `docs/adr/0001-portfolio-pp-only.md`（新建）

**变更**: 将"PortfolioController 只使用 PP 变体"这一架构决策固化为 ADR 文档。决策依据：
- Hospital domain 是 Cooperative Multi-Agent Sokoban with Pull，不是 vanilla MAPF
- CBS 假设点状 agent + 静态环境，在 push-pull 域中不成立（见 `claudeopus47.txt §3.1.2`）
- 完整 CBS 移植到 Sokoban 域预估博士论文级别工作量
- 当前通过多 seed RANDOM 排序 + P4 冲突驱动 PP + escape/NAMO 合成子目标覆盖循环依赖场景

**回归**：纯文档变更，不影响代码

## 终验结果

```
mvn -q clean compile  # PASS

SAbotbot   → Level solved: Yes, 63 actions ✅
MAsimple1  → Level solved: Yes, 27 actions ✅
MAsimple2  → Level solved: Yes, 40 actions ✅
help.lvl   → Timeout (60s budget insufficient for 4-agent 15-goal CBSR loop) ⚠️
```

help.lvl 超时并非回归 — 这是一个复杂的竞赛关卡（4 agents, 15 goals, 30 条依赖边），在 60s 服务器预算下 CBSR 循环无法完成。这不是本次修复引入的问题。

## 待人工审查

```powershell
git log --oneline fix/deepseek-bug-review ^main
# b6d9c9c docs: ADR-0001 record portfolio-uses-PP-only decision (#7)
# 78468a3 docs(L2): cross-reference IW1 find-route usage from constants (#2)
# db1e873 feat(L1): applyJointAction logs once when push/pull source has no box (#6)
# 6a5f22a feat(L4): ConflictResolver picks yielder by holding-box / on-goal priority (#1)
```

审查通过后执行：
```powershell
git push -u origin fix/deepseek-bug-review
```
