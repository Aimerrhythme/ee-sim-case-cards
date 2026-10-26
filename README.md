# ee-sim-case-cards · 电气仿真案例卡

面向高职 / 专升本电气类课程的**仿真案例卡**库。每张卡回答「这个仿真**怎么讲**」：目标是什么、怎么接、参数取多少、曲线应该长什么样、学生常在哪报错。

与 [`ee-lab-toolkit`](https://github.com/Aimerrhythme/ee-lab-toolkit) 互补：toolkit 提供**可运行工具**，本仓提供**讲法与案例**。

## 目录

```
cases/<slug>/
  README.md        # 案例卡正文（按模板）
  matlab/          # 可选：MATLAB 脚本
  simulink/        # 可选：.slx 或搭建步骤说明
  python/          # 可选：Python 复现脚本
  img/             # 可选：示意图 / 预期曲线
docs/case-convention.md
themes.md          # 主题队列
INDEX.md           # 总索引
```

## 案例卡模板（摘要）

1. **目标**：一句话说明本案例要让学生看懂什么
2. **接线图示意**：ASCII / Mermaid / 图片
3. **参数表**：符号、含义、取值、单位
4. **预期曲线**：关键特征（超调、稳态值、时间常数…）
5. **常见报错**：现象 → 原因 → 处理
6. **三平台指引**：Simulink / MATLAB / Python 分别怎么做

完整字段见 [`docs/case-convention.md`](docs/case-convention.md)。

## 说明

- **中文为主**，必要处附英文术语。
- 本仓由定时自动化**持续追加**内容：每次从 `themes.md` 取一条 `todo` 主题 → 生成条目 → 更新 `INDEX.md` 与 `CHANGELOG.md` → 将主题标为 `done`。
- 定位为**教学材料**，**不代写**毕业设计 / 课程设计 / 论文。
- 许可：MIT（见 `LICENSE`，含教学声明）。
