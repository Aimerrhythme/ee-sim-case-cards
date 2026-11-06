# 主题队列

状态：`todo` → `doing` → `done`（完成时在 INDEX 留链接）。

格式：`- [status] slug | 中文标题 | 关键词`

## 种子主题（来自电气×AI 毕设方向池，仅作教辅选题参考）

- [done] pid-nn-motor-speed | 基于 PID 优化与神经网络的电机调速控制系统 | PID, 神经网络, 电机调速
- [done] pv-lstm-short-term | 面向配网消纳的光伏系统仿真及 LSTM 短期功率预测 | 光伏, LSTM, 配网消纳
- [done] wind-rf-abnormal | 基于随机森林的风电机组出力工况识别与异常预警 | 风电, 随机森林, 异常预警
- [done] park-load-clustering | 基于聚类的工业园区典型用电负荷模式识别 | 聚类, 负荷模式, 工业园区
- [done] dc-microgrid-voltage-ml | 基于仿真数据与机器学习的直流微网电压波动抑制 | 直流微网, 电压波动, 机器学习
- [done] renewables-load-partition | 基于聚类分析的新能源配网负荷特性分区仿真 | 聚类, 新能源配网, 负荷特性
- [done] lstm-voltage-sag | 基于 LSTM 时序模型的配电网电压暂降故障预测 | LSTM, 电压暂降, 故障预测
- [done] rf-pv-inverter-fault | 基于随机森林的光伏逆变器故障诊断仿真 | 随机森林, 光伏逆变器, 故障诊断

## 自定义主题

（在下方追加；Routine 优先取本文件中第一条 `todo`。补货条目可附课堂角度与参考链接。）

- [done] cnn-bilstm-residential-load | 居民侧日前负荷 BiLSTM / CNN-BiLSTM 对比 | BiLSTM, 居民负荷, 短期预测
  - 课堂角度：用公开居民侧聚合负荷做「昨日→次日」滚动预测，对比朴素日翻日、LSTM、BiLSTM、CNN-BiLSTM 的 RMSE，适合专科讲清双向时序与卷积特征提取各自带来什么。
  - 参考：https://arxiv.org/abs/2302.05033 · https://github.com/Varat7v2/STLF-BiLSTM-CNNBiLSTM

- [done] cnn-rf-battery-soh | 锂电健康状态 CNN+随机森林估算 | SOH, CNN, 随机森林, 储能
  - 课堂角度：从部分充放电曲线入手，先用 CNN 抽特征再用随机森林回归容量衰减，接储能/电动汽车导论里的「健康状态」概念，不要求搭完整电池电化学模型。
  - 参考：https://arxiv.org/abs/2010.10452

- [todo] seq2point-nilm | 非侵入式负荷分解 seq2point CNN 入门 | NILM, CNN, 负荷分解
  - 课堂角度：只给总表功率，用滑动窗口 CNN 估计单台电器功率（seq2point），可选用 REDD/UK-DALE 公开数据做电器开关识别演示，贴近智能用电与需求侧管理。
  - 参考：https://arxiv.org/abs/1612.09106 · https://github.com/redefik/ConvNILM

- [todo] cnn-pf-surrogate-5bus | 五母线潮流 CNN 代理模型课堂实验 | CNN, 潮流, 代理模型
  - 课堂角度：把蒙特卡洛潮流结果当标签，训练一维 CNN 近似电压/支路潮流，对照 pandapower 精确潮流，让学生体会「物理仿真 vs 数据代理」的速度与误差权衡。
  - 参考：https://github.com/enliten/IEEE-course_AI-for-Power-Energy-Systems

- [todo] cnn-voltage-quality | 电压质量扰动时序 CNN 分类 | 电能质量, CNN, 暂降 swell
  - 课堂角度：把暂降、暂升、中断、谐波等波形做成多类分类任务，强调特征窗长与噪声鲁棒性，可与继电保护/电能质量实验课联动。
  - 参考：https://arxiv.org/abs/1904.00228

- [todo] microgrid-inverter-switch-ml | 微网逆变器开关故障与异常 ML 检测 | 微网, 逆变器, 故障定位, KNN
  - 课堂角度：在含光伏与储能的逆变器主导微网里区分内部开关故障与虚假数据注入类异常，用 KNN 等分类器做开关定位，扩展已有「光伏逆变器故障」选题到系统级场景。
  - 参考：https://arxiv.org/abs/2411.08761
