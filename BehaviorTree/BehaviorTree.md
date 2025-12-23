# **行为树（Behavior Tree, BT）**
- 参考资料
- http://dev.nav2.fishros.com/doc/behavior_trees/trees/nav_through_poses_recovery.html
- https://www.behaviortree.dev/docs/3.8/learn-the-basics/bt_basics/
  
# 1. 是什么

## 1.1 BT 的定义

行为树（Behavior Tree, BT）是一种用树状结构组织智能体行为逻辑的决策模型。在游戏领域，行为树已经比较流行了。主要用于维护游戏角色的各种动作和状态。
它具有以下核心特点：

* **层次化结构**：使用树节点表达行为逻辑。
* **Tick 驱动执行模型**：根节点周期性被 tick，状态沿树传播，节点返回三种状态：

  * `SUCCESS`
  * `FAILURE`
  * `RUNNING`
  
* **还会有两种特殊状态:**

  * `IDLE`:未被激活 / 已重置，库强烈建议自定义节点不要主动返回 IDLE
  * `SKIPPED`:既不是激活状态，也不是完成状态，是一种‘被逻辑跳过’的特殊返回。
  
* **逻辑与行为解耦**：高层（控制策略）与底层（动作实现）分离，提高可复用性。
* **组件化与可编排性**：通过组合节点可以构建复杂任务流程，易调试、易拓展。
  
---

## 1.2 BT 与 FSM（有限状态机）对比

| 维度          | BT（行为树）                      | FSM（有限状态机）                      |
| ----------- | ---------------------------- | ------------------------------- |
| **结构复杂度**   | 层级结构天然支持组合、复用，复杂度随树深度线性增长    | 状态和状态间迁移呈指数级膨胀（State Explosion） |
| **逻辑修改成本**  | 可独立修改子树，影响范围可控               | 改一个状态可能影响多个转移，维护成本高             |
| **并行与中断能力** | 支持并行节点、优先级节点、守护节点（Decorator） | 中断通常需显式建状态，结构复杂                 |
| **可读性与可视化** | 可直接图形化表达                     | 状态机复杂后难理解                       |
| **重复行为复用**  | 子树可复用                        | 需重复建状态或使用嵌套子状态（复杂）              |
| **适用场景**    | 有层级结构、流程多变、需容错的场景（机器人）       | 逻辑简单、状态少、确定性强的场景（协议、硬件）         |

### 总结

* **FSM 适合“确定流程、离散事件少”的系统**。
* **BT 适合“复杂流程、混合行为、需要容错”的机器人系统**。
  无人叉车属于后者，因此 BT 更合适。

---

# 2. 干什么

## 2.1 BT 在无人叉车系统中能干什么

无人叉车（AGF / laser AGV）典型特征：

* 多任务调度
* 多阶段作业（导航 / 对位 / 穿叉 / 升降 / 放置）
* 同时需兼容安全检测、导航、避障、重规划、故障处理

**BT 在无人叉车系统中的典型功能包括：**

###  任务级决策

* 接收任务（取货、卸货、其他调度指令）
* 根据黑板（Blackboard）中的任务参数自动分解流程
* 不同任务类型通过复用子树完成（取货、卸货等）

###  动作与子任务组织

* 导航 -> 停靠 -> 对位 -> 升降 -> 抓取/放置
* 使用 Sequence、Selector 做条件判断与子流程切换
* 支持失败回退（Fallback）提高鲁棒性

###  安全逻辑

* 随时检测急停信号
* 障碍物检测、避障失败重规划
* 货物状态检查
* 信号连接情况

###  异常恢复

* 导航失败 -> 重规划
* 抓取失败 -> 再次对位/重新插叉
* 放置失败 -> 重试或报警
* 故障 -> 集成 fallback 节点进入安全模式
* 货物状态异常 -> ????

---

## 2.2 无人叉车典型作业场景 & BT 能做什么

###  场景 1：取货 -> 运送 -> 卸货

流程：

1. 导航到托盘 A
2. 自动对位
3. 穿叉 + 升起
4. 导航到目标点
5. 放下托盘并退出

对应 BT 功能：

* 用一个 Sequence 表达完整作业流程
* 在导航失败时，Selector 自动执行 “ReplanRoute”
* 在取货失败时进入 “重新对位” 子树
* parallel repeat执行安全状态监控

---

###  场景 2：充电流程 / 低电回库

流程：

* 检查电量 -> 低电 -> 停工 -> 前往充电站 -> 对位 -> 进入充电模式
  这是典型 “Selector + Sequence” 的 BT 结构。

---

# **3. 怎么用**

行为树的“语法”并不是某种编程语言，而是一套**结构化的逻辑规则**。就像搭积木一样，通过固定类型的节点进行组合，构成机器人完整的决策过程。

# **3.1 行为树语法总体概念**

行为树的基础语法由三部分组成：

1. **树状结构**（父节点管理子节点）
2. **节点执行规则**（不同类型节点有不同逻辑）
3. **节点返回状态**（Success / Failure / Running）

---

# **3.2 行为树的五大节点类型**

### **（1）控制节点（Control）**

控制节点决定子节点的执行顺序，是 BT 的“语法结构骨架”。

通用执行模式：

1. 接收 tick  
2. 按自身策略依次（或并行）tick 子节点  
3. 根据子节点返回值综合出自己的状态（Success / Failure / Running / Skipped）

#### 控制节点的语法类比

* `Sequence` → **顺序语句**（先做 A 再做 B）
* `Fallback/Selector` → **条件/备选语句**（A 不行就试 B）
* `Parallel` → **并行语句**（A 和 B 同时推进）

#### 常见控制节点语义：

| 控制节点 | 语义解释 | Brief | 执行逻辑 |
|---------|----------------------------------------|-------------------|-------------------|
| **Sequence** | 从第一个子节点开始依次执行：遇到 `RUNNING` 立即返回 `RUNNING`；遇到 `FAILURE` 立即返回 `FAILURE`，并调用 `resetChildren()` 将所有子节点（包括可能处于 `RUNNING` 的后续子节点）`halt + reset` 为 `IDLE`；全部子节点依次返回 `SUCCESS` 时，本节点返回 `SUCCESS`。 | The SequenceNode is used to tick children in an ordered sequence. If any child returns RUNNING, previous children will NOT be ticked again. | - If all the children return SUCCESS, this node returns SUCCESS.<br>- If a child returns RUNNING, this node returns RUNNING. Loop is NOT restarted, the same running child will be ticked again.<br>- If a child returns FAILURE, stop the loop and return FAILURE. Restart the loop only if (reset_on_failure == true) |
| **SequenceWithMemory** | 与 Sequence 类似，但**记住当前子节点索引**：失败时不会把 `current_child_idx_` 归零，只会 `halt` 当前之后的子节点，下次 tick 仍从当前索引继续；成功则向后推进，到所有子节点都 `SUCCESS` 时返回 `SUCCESS`；适合"长链条流程，中途失败后从中间继续"的场景。 | The SequenceWithMemory is used to tick children in an ordered sequence. If any child returns RUNNING, previous children are not ticked again. | - If all the children return SUCCESS, this node returns SUCCESS.<br>- If a child returns RUNNING, this node returns RUNNING. Loop is NOT restarted, the same running child will be ticked again.<br>- If a child returns FAILURE, stop the loop and return FAILURE. Loop is NOT restarted, the same running child will be ticked again. |
| **Fallback / Selector** | 按顺序尝试多个子节点：如果某个子节点 `SUCCESS`，立即返回 `SUCCESS`；如果子节点 `RUNNING`，返回 `RUNNING`；只有所有子节点都返回 `FAILURE`（或被 `SKIPPED` 且没有成功）时才返回 `FAILURE`。 | The FallbackNode is used to try different strategies, until one succeeds. If any child returns RUNNING, previous children will NOT be ticked again. | - If all the children return FAILURE, this node returns FAILURE.<br>- If a child returns RUNNING, this node returns RUNNING.<br>- If a child returns SUCCESS, stop the loop and return SUCCESS. |
| **ReactiveSequence** | 每个 tick 都**从第一个子节点重新开始**：对子节点 0..N-1 依次 `executeTick()`；如果某个子节点 `RUNNING`，会 `haltChild()` 其他兄弟，使得"同一时刻只有一个子节点在 RUNNING"，然后返回 `RUNNING`；如果某个子节点 `FAILURE`，立刻 `resetChildren()` 并返回 `FAILURE`；全部 `SUCCESS` 则返回 `SUCCESS`。适合"持续重评估条件 + 单一活动行为"的场景。 | The ReactiveSequence is similar to a ParallelNode. All the children are ticked from first to last: | - If a child returns RUNNING, halt the remaining siblings in the sequence and return RUNNING.<br>- If a child returns SUCCESS, tick the next sibling.<br>- If a child returns FAILURE, stop and return FAILURE.<br>- If all the children return SUCCESS, this node returns SUCCESS. |
| **ReactiveFallback** | 与 ReactiveSequence 对偶：每 tick 从头依次执行；`SUCCESS` 时立即 `resetChildren()` 并返回 `SUCCESS`；`RUNNING` 时也会 `halt` 其它兄弟，保证只有一个 RUNNING；所有子节点都 `FAILURE`（可能夹杂 `SKIPPED`）时才返回 `FAILURE`。适合"优先级条件 + 备选行为"的场景。 | The ReactiveFallback is similar to a ParallelNode. All the children are ticked from first to last: | - If a child returns RUNNING, continue to the next sibling.<br>- If a child returns FAILURE, continue to the next sibling.<br>- If a child returns SUCCESS, stop and return SUCCESS.<br>- If all the children fail, than this node returns FAILURE. |
| **Parallel** | 多个子节点在一个 tick 内都被执行：对尚未完成的子节点调用 `executeTick()`，统计 `SUCCESS` / `FAILURE` 个数；当成功数达到 `success_threshold` （或在负阈值语义下"至少多少个成功"）时，`resetChildren()` 并返回 `SUCCESS`；当失败数达到 `failure_threshold` 或"已经不可能再满足成功阈值"时，`resetChildren()` 并返回 `FAILURE`；否则返回 `RUNNING`；所有子节点都被 `SKIPPED` 时返回 `SKIPPED`。 | The ParallelNode execute all its children __concurrently__, but not in separate threads! Even if this may look similar to ReactiveSequence, this Control Node is the __only__ one that can have multiple children RUNNING at the same time. | - The Node is completed either when the THRESHOLD_SUCCESS or THRESHOLD_FAILURE number is reached (both configured using ports).<br>- If any of the thresholds is reached, and other children are still running, they will be halted. |
| **ParallelAll** | 要求**全部子节点都完成**（成功或失败）才返回最终状态；只有当失败数达到 `max_failures` 阈值时，才返回 `FAILURE`，否则返回 `SUCCESS`；适合"所有子任务都必须完成，但允许一定失败数"的协同场景。 | The ParallelAllNode execute all its children __concurrently__, but not in separate threads! It differs in the way ParallelNode works because the latter may stop and halt other children if a certain number of SUCCESS/FAILURES is reached, whilst this one will always complete the execution of ALL its children. | - Always complete the execution of ALL its children.<br>- If failure_count >= max_failures, return FAILURE.<br>- Otherwise, return SUCCESS when all children complete. |
| **Switch** | 读取黑板中的 `variable`，与若干 `case_i` 字符串比较（支持枚举映射）；匹配到某个 case 则只 tick 对应子节点，否则执行默认子节点；如果当前运行中的子节点在本 tick 中不再匹配，会先被 `haltChild()`，然后转到新的分支。语义等价于 `switch-case-default`。 | The SwitchNode is equivalent to a switch statement, where a certain branch (child) is executed according to the value of a blackboard entry. | - Read "variable" from blackboard.<br>- Compare with case_1, case_2, ... case_N.<br>- If match case_i, execute child i.<br>- If no match, execute default child (last).<br>- If current running child no longer matches, halt it and switch to new branch. |
| **IfThenElse** | 第 1 个子节点是条件：若返回 `SUCCESS`，执行第 2 个子节点；若返回 `FAILURE`，执行第 3 个子节点（如存在）；只有 2 个子节点时，相当于"条件失败时整个节点返回 `FAILURE`"；本节点本身不是"Reactive"的。 | IfThenElseNode must have exactly 2 or 3 children. This node is NOT reactive. | - The first child is the "statement" of the if.<br>- If that return SUCCESS, then the second child is executed.<br>- Instead, if it returned FAILURE, the third child is executed.<br>- If you have only 2 children, this node will return FAILURE whenever the statement returns FAILURE. |
| **WhileDoElse** | 第一个子节点作为条件，**每个 tick 都重新评估条件**（Reactive 特性）：条件 `SUCCESS` 时执行第 2 个子节点（Do 分支），条件 `FAILURE` 时执行第 3 个子节点（Else 分支，如存在）；如果正在执行的分支变为 RUNNING，而条件状态改变，会先 `haltChild()` 当前分支，然后切换到新分支。适合"条件满足时循环执行，不满足时走尾处理"的结构。 | WhileDoElse must have exactly 2 or 3 children. It is a REACTIVE node of IfThenElseNode. | - The first child is the "statement" that is executed at each tick.<br>- If result is SUCCESS, the second child is executed.<br>- If result is FAILURE, the third child is executed.<br>- If the 2nd or 3d child is RUNNING and the statement changes, the RUNNING child will be stopped before starting the sibling. |
| **ManualSelector** | 通过外部接口选择当前要执行的子节点索引；BT 本身只负责按该索引 tick 对应子节点，其余由"外部控制逻辑"决定，适合调试 / 人工介入。 | Use a Terminal User Interface (ncurses) to select a certain child manually. | - Use TUI (ncurses) to select child index.<br>- Execute the selected child.<br>- Return the status of the selected child. |



## **（2）装饰节点（Decorator）**

装饰节点本身不定义具体业务行为，只负责**在“一个子节点”外层包一层控制逻辑**，修改其执行时机或返回值。

| 装饰器 | 语义解释 | Brief | 执行逻辑 |
|--------|---------|-------------------|-------------------|
| **Inverter** | 取反：子节点 `SUCCESS` → 返回 `FAILURE`，子节点 `FAILURE` → 返回 `SUCCESS`；`RUNNING` 原样透传，`SKIPPED` 一般也按原样传递。 | The InverterNode returns SUCCESS if child fails or FAILURE if child succeeds. RUNNING status is propagated. | - If child returns SUCCESS, this node returns FAILURE.<br>- If child returns FAILURE, this node returns SUCCESS.<br>- If child returns RUNNING, this node returns RUNNING. |
| **Retry** | 当子节点 `FAILURE` 时，按配置的最大次数自动重新执行；一旦某次执行 `SUCCESS`，立刻返回 `SUCCESS`；在重试期间如果子节点返回 `RUNNING`，则本节点返回 `RUNNING`；如果子节点返回 `SKIPPED`，会直接返回 `SKIPPED`（不重试）；超过最大尝试次数仍失败，则本节点返回 `FAILURE`。支持 `-1` 表示无限重试。 | The RetryNode is used to execute a child several times if it fails. | - If the child returns SUCCESS, the loop is stopped and this node returns SUCCESS.<br>- If the child returns FAILURE, this node will try again up to N times (N is read from port "num_attempts").<br>- If the child returns RUNNING, this node returns RUNNING. |
| **Repeat** | 子节点 `SUCCESS` 时增加计数；未达 `num_cycles` 时重置子节点，继续下一轮；达到 `num_cycles`（或 `-1` 无限循环）后返回 `SUCCESS`；子节点 `FAILURE` 时立即重置并返回 `FAILURE`；子节点 `SKIPPED` 时会直接返回 `SKIPPED`（不计数，不继续循环）；子节点 `RUNNING` 时本节点也返回 `RUNNING`。 | The RepeatNode is used to execute a child several times, as long as it succeed. | - To succeed, the child must return SUCCESS N times (port "num_cycles").<br>- If the child returns FAILURE, the loop is stopped and this node returns FAILURE. |
| **ForceSuccess** | 每次先 tick 子节点：如果子节点返回 `SUCCESS` 或 `FAILURE`，都会 `resetChild()` 并强制返回 `SUCCESS`；如果子节点是 `RUNNING` 或 `SKIPPED`，则原样透传，不做修改。 | The ForceSuccessNode returns always SUCCESS or RUNNING. | - If child returns SUCCESS or FAILURE (isStatusCompleted), reset child and return SUCCESS.<br>- If child returns RUNNING or SKIPPED, return child status unchanged. |
| **ForceFailure** | 与 ForceSuccess 相反：子节点返回 `SUCCESS` / `FAILURE` 时都被强制映射为 `FAILURE`（并重置子节点）；子节点 `RUNNING` / `SKIPPED` 时保持不变。 | The ForceFailureNode returns always FAILURE or RUNNING. | - If child returns SUCCESS or FAILURE (isStatusCompleted), reset child and return FAILURE.<br>- If child returns RUNNING or SKIPPED, return child status unchanged. |
| **Delay** | 第一次 tick 时启动一个定时器并立即返回 `RUNNING`；只有当延时结束后，才真正开始 tick 子节点，并返回子节点的状态；期间如果被 halt，会取消定时器并重置。 | The delay node will introduce a delay and then tick the child returning the status of the child as it is upon completion. | - First tick: start timer, return RUNNING.<br>- During delay: return RUNNING.<br>- After delay: tick child and return child status.<br>- If halted during delay: cancel timer and reset. |
| **Timeout** | 在子节点外包一个超时：启动计时器并 tick 子节点；如果在超时时间内子节点完成，则返回子节点状态；如果子节点在超时点仍为 `RUNNING`，装饰器会 `haltChild()` 并返回 `FAILURE`。 | The TimeoutNode will halt() a running child if the latter has been RUNNING longer than a given time. | - Start timer and tick child.<br>- If child completes within timeout, return child status.<br>- If timeout is reached and child is still RUNNING, halt child and return FAILURE. |
| **SubTree** | `SubTreeNode` 是一个特殊 Decorator，用来在当前树中实例化并执行另一个已注册的行为子树，并处理**端口重映射**（当前树的黑板键 ↔ 子树内部端口/黑板）。从语义上看，它是"调用子流程"的节点。 | The SubTreeNode is a way to wrap an entire Subtree, creating a separated BlackBoard. | - Instantiate the subtree with ID.<br>- Remap ports from parent blackboard to subtree blackboard.<br>- Execute subtree root node.<br>- Return subtree root status. |
| **RunOnce** | 子节点第一次跑出 `SUCCESS` 或 `FAILURE` 后，会缓存结果；后续每次 tick 都直接返回缓存的最终状态，不再真正 tick 子节点；保证子行为只执行一次。 | The RunOnceNode is used when you want to execute the child only once. | - First execution: tick child until SUCCESS or FAILURE, cache result.<br>- If "then_skip" is TRUE (default): return SKIPPED in future ticks.<br>- If "then_skip" is FALSE: return cached status synchronously forever. |
| **Loop** | 从黑板或静态配置的队列（`std::deque`）中依次取出元素，将元素值写入 `value` 端口，然后执行子节点；只要队列非空且子节点未返回 `FAILURE`，就继续循环；子节点 `SUCCESS` 时重置子节点，继续处理下一个队列元素；子节点 `FAILURE` 时停止循环并返回 `FAILURE`；队列为空时返回 `if_empty` 端口配置的状态（默认 `SUCCESS`）。适合"遍历一组数据并依次处理"的场景。 | The LoopNode class is used to pop_front elements from a std::deque. | - Pop element from queue, copy to "value" port.<br>- Execute child with element value.<br>- If child SUCCESS: reset child, continue with next element.<br>- If child FAILURE: stop and return FAILURE.<br>- If queue empty: return "if_empty" status (default SUCCESS). |
| **KeepRunningUntilFailure** | 持续执行子节点：子节点 `SUCCESS` 时重置子节点，但本节点返回 `RUNNING`（继续循环）；子节点 `RUNNING` 时本节点也返回 `RUNNING`；一旦子节点返回 `FAILURE`，本节点才返回 `FAILURE`，循环结束。适合"持续监控直到失败"的场景。 | The KeepRunningUntilFailureNode returns always FAILURE or RUNNING. | - If child returns FAILURE: reset child and return FAILURE.<br>- If child returns SUCCESS: reset child and return RUNNING (continue loop).<br>- If child returns RUNNING: return RUNNING. |
| **Precondition** | 在子节点外加一层脚本条件（基于内置脚本引擎）：只有条件表达式为真时才 tick 子节点，否则直接返回 `FAILURE` / `SKIPPED` 等配置好的结果。 | （源码中 PreconditionNode 没有单独的 @brief 注释） | - Evaluate "if" script condition.<br>- If condition is true OR child is already running: tick child and return child status.<br>- If condition is false AND child not running: return "else" status (default FAILURE).<br>- When child completes, reset _children_running flag. |
| **UpdatedDecorator** | 仅在子节点状态变化（例如从 RUNNING→SUCCESS）时触发特定逻辑，其余 tick 可能直接透传或短路，可用来做"边沿检测""状态变化通知"等。 | The EntryUpdatedDecorator checks the Timestamp in an entry to determine if the value was updated since the last time (true, the first time). | - Check Timestamp in blackboard entry.<br>- If value was updated since last time (or first time): execute child and return child status.<br>- Otherwise: return [if_not_updated] value (default SKIPPED). |
| **UpdatedDecorator** | 仅在子节点状态变化（例如从 RUNNING→SUCCESS）时触发特定逻辑，其余 tick 可能直接透传或短路，可用来做"边沿检测""状态变化通知"等。 | The EntryUpdatedDecorator checks the Timestamp in an entry to determine if the value was updated since the last time (true, the first time). If it is, the child will be executed, otherwise [if_not_updated] value is returned. |The EntryUpdatedDecorator checks the Timestamp in an entry to determine if the value was updated since the last time (true, the first time). | - If child is still executing (still_executing_child_ == true): continue ticking child and return child status.<br>- If child is not executing: check the sequence_id of the blackboard entry.<br>  - If entry does not exist: return [if_not_updated] value.<br>  - If previous_id == current_id (value not updated): return [if_not_updated] value.<br>  - If previous_id != current_id (value updated): update sequence_id_, tick child, set still_executing_child_ flag, and return child status.<br>- When child completes (status != RUNNING): reset still_executing_child_ flag. |

> **核心思想：装饰器不改“做什么”，只通过外层规则改“何时做 / 做几次 / 结果怎么解释”。**

---
## **（3）Condition（条件节点）**

- 不产生物理动作，只读取黑板或传感器状态做逻辑判断；
- 一般在单个 tick 内就完成判断并返回：
  - 满足条件 → `SUCCESS`
  - 不满足 → `FAILURE`
- 通常不应该返回 `RUNNING`（除非你真的实现了一个“耗时判断”的 Condition）。

语法角色：

> **相当于 if 条件表达式，是行为树里的“判断语句”。**

典型用途：电量是否低、电机是否故障、路径是否有效、障碍物距离是否安全、任务参数是否准备好等。

---

## **（4）Action（动作节点）**

- 负责具体行动（导航、升降货叉、插叉、放货、对位等）；
- 典型实现基于 `StatefulActionNode`：
  - 第一次 tick 调用 `onStart()`，一般返回 `RUNNING` 或 `SUCCESS`；
  - 后续 tick 调用 `onRunning()`，在内部驱动长期动作；
  - `onHalted()` 处理被中断时的清理（停止控制指令、释放资源等）；
- 运行中通常返回：
  - `RUNNING` → 动作正在进行中；
  - `SUCCESS` → 动作完成；
  - `FAILURE` → 动作失败（例如规划失败、控制异常）。

语法角色：

> **Action 是行为树中最基本的“可执行语句”，真正推动机器人去“做事”。**

---

## **（5）SubTree（子树节点）**

- 对应枚举类型：`NodeType::SUBTREE`
- 主要实现类：`SubTreeNode`（继承自 `DecoratorNode`）
  - 在当前树中**实例化并执行另一个已注册的行为树**；
  - 负责子树输入/输出端口与当前黑板之间的映射与同步。

一句话语义：

> **SubTree 节点本质上是“调用另一个行为树”的装饰器，用于子图复用和参数/黑板的映射，是大型 BT 工程中的“模块化接口”。**

# **3.3 行为树运行语法—Tick 机制**

行为树不是一次执行完，而是不断被"Tick"：

1. **根节点每周期被 Tick（例如 10Hz）**
2. Tick 沿树向下传递
3. 每个节点按规则执行并返回状态
4. 返回状态自下往上回传，决定是否继续执行下个节点

Tick 机制的语法性特点：

* **Sequence** 遇到 RUNNING 会暂存状态，下次从该子节点继续
* **Fallback** 遇到 SUCCESS 会终止其他分支
* **Reactive 节点** 会在每个 tick 从头检查条件
* **Parallel** 每 tick 处理多个节点，根据阈值决定返回状态
* **Repeat/Retry** 在循环中持续执行，直到满足条件或失败

BT 的 Tick 机制决定了它可以：

* 随时中断动作（例如安全制动）
* 随时替换子行为（例如避障/重规划）
* 在 Running 中持续执行（如导航过程）

---

# **3.4 Nav2 的行为树语法扩展**

Nav2 采用 BehaviorTree.CPP 并扩展了一些语法概念。

## **（1）基于数据黑板（Blackboard）的参数传递语法**

在 Nav2 中：

* 全局目标点、当前路径、机器人状态
* 都以“键值对”的方式存在黑板中
* 节点间通过黑板共享信息（类似上下文变量）

语法特点：

> **Action 不需要知道来源，只从黑板读取。**
> **Sequence 中的节点通过黑板自动共享状态。**

---

## **（2）Nav2 的强类型节点体系**

Nav2 的 BT 节点分为：

* **Condition 节点（检查状态）**
* **Action 节点（执行导航或局部动作）**
* **Control 节点（由 BT 框架提供）**

它们遵循统一的 BT 语法规则，但 Nav2 把许多机器人行为封装成标准节点，例如：

* “生成路径”
* “执行路径跟随”
* “清除 costmap”
* “导航恢复行为”

这些本质上都属于叶子节点，但具有机器人特化语义。

---

## **（3）Nav2 的“恢复语法”模式**

Nav2 会统一使用 Fallback + RecoveryNode 构造“恢复树”：

* 正常动作失败 -> 进入恢复动作
* 恢复动作成功 -> 回到正常流程
* 多次恢复失败 -> 返回 Failure

这是一种 Nav2 固定的行为树语法模式。

---

# **3.5 行为树语法总结（关键语法点）**

| 语法点               | 说明                             |
| ----------------- | ------------------------------ |
| **树结构即语法结构**      | 通过父子节点描述流程                     |
| **Tick 驱动运行语法**   | 行为树通过周期性 Tick 执行节点             |
| **三类节点构成语法基础**    | 控制节点、装饰节点、叶子节点                 |
| **节点返回值是语法流程的核心** | Success/Failure/Running 决定流程走向 |
| **使用黑板传递参数**      | 是 Nav2 的标准语法机制                 |
| **通过组合构建复杂流程**    | Sequence + Selector 是最基本语法结构   |
| **通过装饰器修改行为逻辑**   | 超时、重试、条件判断都靠 Decorator         |
| **容错语法（Nav2 特色）** | RecoveryNode + Fallback 构成恢复机制 |

---

# **4.行为树例子**

![](image.png)