# AI 协作省 token / 防重复工作：调研与落地

> 调研时间：2026-10-24。目标：让接手的 AI 不重复排查、不重复读大文件、不把上下文拖烂，并把“已经做过什么”变成可检索的事实。
> 本文是**依据与清单**；真正的硬规则在根目录 `AGENTS.md`，按需流程在 `.agents/skills/fwr-workflow/SKILL.md`，会话历史在 `docs/AGENT_LOG.md`。
>
> **安装状态（2026-10-24）**：`ros2-engineering-skills` 已装到 `~/.agents/skills/`；`ros-mcp` 3.1.2 客户端 + 只读包装已装并启用，DSH 已热加载 11 个只读工具；`rosbridge 2.0.8` 已在 9090 运行，端到端只读验证通过。详见 [`ros-mcp-setup.md`](ros-mcp-setup.md)。

## 0. 一句话结论

省 token 的核心不是“少说话”，而是**别把同一事实反复装进上下文**：

1. 稳定前缀 + 追加式历史（提高 KV 缓存命中）；
2. 把状态写到上下文之外（`AGENTS.md` / `AGENT_LOG.md` / `progress.md`）；
3. 先检索再精读（grep → 行范围 read）；
4. 长任务用压缩/子代理隔离，而不是无限拖长一个会话；
5. 按改动类型跑最小验证，证据写清楚，避免下个会话重跑。

## 1. 为什么 AI 会重复劳动、烧 token

- **上下文腐烂（context rot）**：上下文越长，模型对其中信息的准确召回越差；token 不是免费的，注意力预算有限。Anthropic 把这称为 “attention budget”，并建议找“最小的高信号 token 集合”。（[Effective context engineering](https://www.anthropic.com/engineering/effective-context-engineering-for-ai-agents)）
- **输入/输出比例极度倾斜**：Manus 的实测是 agent 平均输入:输出约 **100:1**；每轮工具调用都会把历史再喂一遍，重复读文件/日志的代价被放大。（[Manus: Context Engineering for AI Agents](https://manus.im/blog/Context-Engineering-for-AI-Agents-Lessons-from-Building-Manus)）
- **没有外部记忆**：新会话不知道上次试过什么，就会重新 `ls`、重新读 README、重新跑同一组测试、重新踩同一个坑。
- **长会话不压缩**：接近窗口上限后，工具结果、旧日志、失败尝试都还在，模型更容易迷路或复读。

## 2. 七条可执行原则

### 2.1 稳定前缀，追加式历史（KV cache）

Manus 把 **KV-cache 命中率**称为生产级 agent 最重要的单一指标；相同前缀的缓存输入可能便宜一个数量级。做法：

- 系统提示词/`AGENTS.md` 不要在会话中途反复改；
- 历史只追加，不回头改旧消息；
- 时间戳、随机 ID 不要放在前缀最前面；
- 需要缓存断点时，至少把系统提示词结尾包进去。

对本项目：**开工前或收尾时改 `AGENTS.md`，不要边做边改**。`docs/AGENT_LOG.md` 只追加。

### 2.2 把状态写到上下文之外（结构化笔记）

Anthropic 明确推荐 “structured note-taking / agentic memory”：把关键状态写到 `NOTES.md` 之类的文件，之后按需取回。Claude Code 用 `CLAUDE.md`，本项目对应三层：

| 层 | 文件 | 内容 | 何时读 |
| --- | --- | --- | --- |
| 长期规则 | `AGENTS.md` | 安全红线、验证矩阵、省 token 规则 | 每次自动加载 |
| 会话历史 | `docs/AGENT_LOG.md` | 上次改了什么、证据、运行状态、遗留 | 开工读最后 2–3 条 |
| 计划进度 | `.superpowers/sdd/<plan>/progress.md` | 每个 task 的 commit/test/ruling | 做该计划时读 |

原则：**一个事实只写一处**。不要把 `AGENTS_LOG` 的内容复制进 `AGENTS.md`，也不要把计划进度复制进日志。

### 2.3 渐进披露（Skills）

Anthropic 的 Agent Skills 用三层披露：启动时只加载 `name + description`，匹配任务才读 `SKILL.md` 正文，需要时再读 `references/`。这让“知识很多”不等于“每轮都占 token”。（[Equipping agents with Agent Skills](https://www.anthropic.com/engineering/equipping-agents-for-the-real-world-with-agent-skills)）

对本项目：`AGENTS.md` 只放短规则；详细步骤放进 `.agents/skills/fwr-workflow/SKILL.md`；更长的调研/参考放 `docs/agent/`。不要把 500 行手册塞进 `AGENTS.md`。

### 2.4 先检索，再精读（just-in-time）

Anthropic 的 “just in time” 策略：上下文里只保留文件路径、查询、链接等轻量标识，真正需要时再用工具取。做法：

- `grep -rn` / `glob` 先定位，再 `read` 指定行范围；
- 大文件用 `head/tail/sed -n`，不要整篇读；
- 日志先落盘再 `grep`，不要把几 MB 输出贴进对话；
- ROS 话题用 `--once`、`hz` 采样，不要 `echo` 整个数组。

### 2.5 压缩与修剪（compaction）

DSH 自带压缩：接近上下文上限时把最旧历史摘要、保留近期；`/compact` 可手动触发；工具结果超过默认 8,192 字符会在压缩时被修剪为头 4,096 + 尾 1,024 字符（原始内容仍在会话日志中）。（本机 `@deepseek-ai/dsh-compaction-basic` / `dsh-compaction-tool-result-pruner` 文档）

对本项目：

- 完成一个 task / 一次实机验证后，用 `/compact` 或开新会话；
- 不要把整棵 Nav2 启动日志、整篇 README 留在上下文里；
- 压缩前先把关键结论写进 `progress.md`，别指望摘要一定保留细节。

### 2.6 子代理隔离（subagent）

Anthropic 的多代理模式：子代理可以花几万 token 探索，只回传 1–2k token 的结论，主代理保持干净。适合：

- “把 `fwr_bringup` 里所有和 reverse permit 相关的代码找出来”；
- “对比两份 plan/progress，列出未完成任务”；
- “在网上查某插件是否兼容 DSH 0.2.1-rc.2”。

提示词要写死输出格式：**≤20 行、带文件路径和行号、不要贴大段代码**。

### 2.7 证据化验证（本项目特有）

避免“上次好像跑过了”：

- 每个 task 记录 **命令 + 结果 + 日志路径**；
- 纯函数测试优先，仿真其次，实机最后；
- 连续两次同类失败就停，向用户报告事实和选项；
- “离线测试通过”和“实机已验证”分开写。

## 3. 本项目已经落地的机制

| 文件 | 作用 | 加载方式 |
| --- | --- | --- |
| `AGENTS.md` | 长期规则、当前快照、验证矩阵、安全红线 | DSH 每次会话自动加载 |
| `.agents/skills/fwr-workflow/SKILL.md` | 开工/定位/验证/调试/收尾的详细 SOP | 任务匹配时按需加载 |
| `docs/AGENT_LOG.md` | 跨会话交接：改动、证据、运行状态、遗留 | 开工读最后 2–3 条 |
| `.superpowers/sdd/*/progress.md` | 计划级 task 账本 | 做对应计划时读 |
| `docs/agent/token-efficient-workflow.md` | 本文：依据、插件清单、安装建议 | 需要时读 |

## 4. 插件 / Skill 清单（截至 2026-10-24）

> 说明：当前 DSH 是 **0.2.1-rc.2 Desktop**。Desktop 的插件安装应走 GUI 的「插件 / 安全市场」，不要照搬只适用于 CLI profile 的 `dsh plugin` 命令。第三方插件本质是在本机运行别人的代码；安装前看源码、锁版本、确认 `engines.dsh` 兼容，市场收录不等于安全审计。

### 4.1 优先推荐（与当前痛点直接相关）

| 名称 | 类型 | 为什么适合 | 注意 |
| --- | --- | --- | --- |
| [ROS MCP Server](https://github.com/robotmcp/ros-mcp-server) | 外部 MCP | 通过 rosbridge 让 agent 直接读 topic/service/action/TF、查传感器；支持 ROS 2 Humble；做诊断/巡检很省“人肉转述” | 当前版本**没有内置只读/权限模型**（README 把 permissions 列为后续工作）；先在隔离 `ROS_DOMAIN_ID`/仿真里用，实机不要暴露 `/cmd_vel`，rosbridge 限网段并加鉴权 |
| [ros2-engineering-skills](https://github.com/dbwls99706/ros2-engineering-skills) | Agent Skill | 20 个 reference，覆盖 Humble、Nav2、SLAM、micro-ROS/ESP32、测试、调试；渐进披露，不占常驻上下文 | Apache-2.0；13k+ 行，装到用户级 `~/.agents/skills` 或项目 `.agents/skills`，不要复制进仓库 |
| [robotics-agent-skills](https://github.com/arpitg1304/robotics-agent-skills) | Agent Skills | 10 个较轻的机器人技能（ROS2、测试、bringup、感知、安全），可按需选装 | Apache-2.0；比上一个浅，适合只要 2–3 个技能时 |
| `dsh-superpowers` | DSH 插件 | 把 obra/superpowers 的 14 个方法论技能接进 DSH；本项目已经在用 `.superpowers/sdd` 流程，语义一致 | 会新增技能目录和 bootstrap；先确认不会和现有 plan/progress 流程重复 |
| `dsh-repeat-guard` | DSH 插件 | 检测 DeepSeek V4 Flash 在长上下文里的“碎片复读/空话循环”，截断并提醒直接调用工具；直接针对“烧 token 不干活” | 会中断生成，先在非关键会话试用；不是“防重复任务”的替代品 |

### 4.2 值得一试（按需）

| 名称 | 类型 | 作用 | 注意 |
| --- | --- | --- | --- |
| `dsh-taskfold` | DSH 插件 | 把一段完成的对话折叠成一条摘要，原文可按需取回；适合长任务 | 作者声明支持 DSH 0.2.1-rc.2；和 `progress.md` 有重叠，二选一或明确分工 |
| `dsh-mcp-lens` | DSH 插件 | 把大量 MCP 工具 Schema 收缩成两个入口，按需揭示；实测工具 JSON 674KB→27KB（作者数据） | **装 ROS MCP 后再装**；否则没有收益 |
| `dsh-budget` / `dsh-usage-panel` | DSH 插件 | 看 token/费用、设预算和阈值告警 | 二选一，别同时装多个统计插件 |
| `dsh-capability-panel` | DSH 插件 | 查看 MCP/skill/tool 的加载、截断、驱逐状态 | 排查“为什么 skill 没生效”时有用 |
| `@zzerx/dsh-plugin-memory` | DSH 插件 | 跨会话自动提炼偏好/项目决策，全局+项目双作用域 | 自动记忆会额外调用模型；容易和 `AGENTS.md` 重复。若装，只让它记“偏好”，项目事实仍以 `AGENTS.md` 为准 |
| `dsh-ops` | DSH 插件 | 原生 Rust 文件/PTC 工具，声称提高 token 效率 | 替换核心工具，风险较高；先小仓库试用 |

### 4.3 暂时不建议

- **多个自动记忆插件同时装**（`dsh-plugin-memory` + `dsh-self-improved` + `dsh-unified-agent-memory`）：会互相注入、重复、甚至冲突；本项目已经有可审计的 `AGENTS.md` + `AGENT_LOG.md`。
- **多个压缩插件同时装**（`dsh-taskfold` + `dsh-dcp` + `dsh-force-compact`）：DSH 自带 compaction，重复接管容易出边界问题。
- **一开始就上多代理/工作流编排**：先把单会话的交接和验证跑顺，再考虑并行。
- **来源不明的 skill 包**：skill 可以指示模型执行命令；安装前读 `SKILL.md` 和脚本。

## 5. 建议的安装顺序

1. **先不装插件**，用 1–2 周验证 `AGENTS.md` + `AGENT_LOG.md` + `fwr-workflow` 是否真的减少了重复排查。
2. 需要让 agent 直接看 ROS 状态时：装 **ROS MCP Server**（只读），再考虑 `dsh-mcp-lens`。
3. 需要补机器人工程知识时：把 **ros2-engineering-skills** 克隆到用户级 `~/.agents/skills/`（不要进仓库），只保留 `SKILL.md` + 需要的 `references/`。
4. 长会话经常爆上下文时：再评估 `dsh-taskfold` 或 DSH 自带 `/compact`。
5. 发现模型复读/空转时：装 `dsh-repeat-guard`。
6. 想看花了多少钱：装一个 `dsh-budget` 或 `dsh-usage-panel`。

### 安装命令（用户级 skill，需在仓库外执行）

```bash
mkdir -p ~/.agents/skills
# 若目录已存在，先改名或删除，避免 clone 失败
git clone --depth 1 https://github.com/dbwls99706/ros2-engineering-skills.git \
  ~/.agents/skills/ros2-engineering-skills
```

DSH 会自动发现 `~/.agents/skills/<name>/SKILL.md`；项目级则放到 `.agents/skills/`。装完新开会话，用 `skill` 工具应能看到目录项。

ROS MCP（只读诊断用）：

```bash
# 机器人侧
sudo apt install ros-humble-rosbridge-server
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# 开发机侧（MCP 客户端按 DSH 的 MCP 配置接入）
uvx ros-mcp --transport=stdio
```

## 6. 怎么衡量“有没有变省”

不要只看感觉，建议每次会话记录三个数：

| 指标 | 怎么取 | 目标 |
| --- | --- | --- |
| 输入 token / 会话 | `dsh-usage-panel` 或 `dsh-budget` | 同类任务逐月下降 |
| 重复排查次数 | 手工记：本次是否重读了上次已结论的日志/代码 | 趋近 0 |
| 首次正确命令时间 | 从开工到第一条有效验证命令 | 下降 |
| 无效全量测试次数 | 日志里同一天重复跑同一套 | ≤1 次/天 |

如果 token 降了但返工率上升，说明压缩过头；如果返工率没降，说明日志写得不够“可执行”（缺命令/结果/路径）。

## 7. 参考来源

- Anthropic, [Effective context engineering for AI agents](https://www.anthropic.com/engineering/effective-context-engineering-for-ai-agents)（context rot、compaction、structured notes、subagents、JIT retrieval）
- Anthropic, [Equipping agents for the real world with Agent Skills](https://www.anthropic.com/engineering/equipping-agents-for-the-real-world-with-agent-skills)（渐进披露、SKILL.md、安全）
- Manus, [Context Engineering for AI Agents: Lessons from Building Manus](https://manus.im/blog/Context-Engineering-for-AI-Agents-Lessons-from-Building-Manus)（KV cache、100:1、append-only、文件系统记忆）
- [AGENTS.md 开放格式](https://agents.md/)（60k+ 项目使用的 agent 说明文件）
- OpenAI, [Codex Prompting Guide](https://developers.openai.com/cookbook/examples/gpt-5/codex_prompting_guide)（Codex 读取 `AGENTS.md` 作为项目指令）
- OpenAI, [Prompt caching](https://developers.openai.com/api/docs/guides/prompt-caching)（稳定前缀与按需加载工具；API 缓存命中不应当作本地 Codex 会话的保证）
- [robotmcp/ros-mcp-server](https://github.com/robotmcp/ros-mcp-server)（ROS 2 Humble、rosbridge、MCP）
- [dbwls99706/ros2-engineering-skills](https://github.com/dbwls99706/ros2-engineering-skills)（Humble/Nav2/micro-ROS，Apache-2.0）
- [arpitg1304/robotics-agent-skills](https://github.com/arpitg1304/robotics-agent-skills)（10 个机器人技能，Apache-2.0）
- DSH 本机文档：`@deepseek-ai/dsh-agent-instructions`、`dsh-compaction-basic`、`dsh-compaction-tool-result-pruner`、`dsh-skill-filesystem`
- DSH 社区目录：[Bruce's Plugin Picks](https://bruc3van.com/markets/dsh/index.json)、[awesome-dsh-plugin](https://github.com/bruc3van/awesome-dsh-plugin)
