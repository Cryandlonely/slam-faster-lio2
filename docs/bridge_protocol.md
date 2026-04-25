# Bridge 通讯协议文档

**版本**: v1.3  
**更新日期**: 2026-04-24

---

## 1. 连接规格

| 参数 | 值 |
|------|----|
| 传输协议 | TCP (长连接 / 短连接均可) |
| 控制端口 | **9090** |
| PCD 传输端口 | **9091** |
| 消息编码 | UTF-8 JSON |
| 帧格式 | 每条消息以 `\n`（0x0A）结尾 |
| 方向 | 双向。上位机 → Bridge 为指令；Bridge → 上位机为应答 + 主动推送 |

> **注意**：控制端口和 PCD 端口是独立 TCP 服务，需分别建连。

---

## 2. 状态主动推送（控制端口 9090）

Bridge 以固定频率（默认 **5 Hz**）向所有已连接客户端广播当前状态包，**无需发送任何指令**即可持续接收。

> Bridge 同时也会在导航任务结束时推送一次 `nav_result` 事件包，详见 §2.2。

### 2.1 位置状态包

位置字段**根据当前定位模式自动切换**，所有模式均附带电量和底盘速度：

#### 室内模式（`type == "pose"`, `mode == "slam"`）

```json
{"type":"pose","x":1.2340,"y":5.6780,"yaw":0.5200,"battery":24.6,"vx":0.350,"vy":0.000}
```

#### 室外模式（`type == "pose"`, `mode == "gps"`）

```json
{"type":"pose","lat":36.66132500,"lon":117.01681200,"yaw":0.5200,"battery":24.6,"vx":0.500,"vy":0.000}
```

### 字段说明

| 字段 | 类型 | 模式 | 说明 |
|------|------|------|------|
| `type` | string | 全部 | 固定 `"pose"` |
| `x` | float | **slam** | 当前位置 X（m，map 坐标系） |
| `y` | float | **slam** | 当前位置 Y（m，map 坐标系） |
| `lat` | float | **gps** | 当前纬度（度，WGS84，精度 8 位小数） |
| `lon` | float | **gps** | 当前经度（度，WGS84，精度 8 位小数） |
| `yaw` | float | 全部 | 车辆朝向（rad，[-π, π]） |
| `battery` | float | 全部 | 底盘电池电压（V） |
| `vx` | float | 全部 | 底盘前向速度（m/s） |
| `vy` | float | 全部 | 底盘侧向速度（m/s） |

### 2.2 导航结果事件包

当导航任务结束时（状态机跳变到 `REACHED` 或 `ERROR`），Bridge 立即向所有客户端推送一次：

```json
{"type":"nav_result","result":"REACHED"}
```
```json
{"type":"nav_result","result":"ERROR"}
```

| `result` 值 | 说明 |
|-------------|------|
| `REACHED` | 已成功到达目标 |
| `ERROR` | 导航异常终止 |



---

## 3. 控制指令（控制端口 9090）

所有指令均为 **JSON + `\n`** 帧，Bridge 收到后同步返回一条应答。

---

### 3.1 单点导航 `nav_goal`

#### 室内模式（map 坐标系）

```json
{"cmd":"nav_goal","coord_mode":"local","x":2.0,"y":1.5,"yaw":0.0}
```

| 字段 | 必填 | 类型 | 说明 |
|------|------|------|------|
| `cmd` | ✅ | string | `"nav_goal"` |
| `coord_mode` | ✅ | string | `"local"` |
| `x` | ✅ | float | 目标 X（m） |
| `y` | ✅ | float | 目标 Y（m） |
| `yaw` | ❌ | float | 目标航向（rad），默认 0.0 |

应答：
```json
{"ack":"nav_goal"}
```

#### 室外模式（WGS84 经纬度）

```json
{"cmd":"nav_goal","coord_mode":"gps","lat":36.661230,"lon":117.016880,"yaw":0.0}
```

| 字段 | 必填 | 类型 | 说明 |
|------|------|------|------|
| `cmd` | ✅ | string | `"nav_goal"` |
| `coord_mode` | ✅ | string | `"gps"` |
| `lat` | ✅ | float | 纬度（度，WGS84） |
| `lon` | ✅ | float | 经度（度，WGS84） |
| `yaw` | ❌ | float | 到达后航向（rad） |
| `heading_deg` | ❌ | float | 到达后航向（度，与 yaw 二选一） |
| `target_vel` | ❌ | float | 目标速度（m/s） |

应答：
```json
{"ack":"nav_goal"}
```

---

### 3.2 多点导航 `nav_waypoints`

#### 室内多点

```json
{
  "cmd": "nav_waypoints",
  "coord_mode": "local",
  "waypoints": [
    {"x": 1.0, "y": 0.5},
    {"x": 2.0, "y": 1.0, "yaw": 1.57},
    {"x": 3.0, "y": 2.0}
  ],
  "target_vel": 0.5
}
```

#### 室外多点（GPS）

```json
{
  "cmd": "nav_waypoints",
  "coord_mode": "gps",
  "waypoints": [
    {"lat": 36.661230, "lon": 117.016880},
    {"lat": 36.661400, "lon": 117.017100, "yaw": 0.0}
  ],
  "target_vel": 0.8
}
```

| 字段 | 必填 | 类型 | 说明 |
|------|------|------|------|
| `cmd` | ✅ | string | `"nav_waypoints"` |
| `coord_mode` | ✅ | string | `"local"` 或 `"gps"` |
| `waypoints` | ✅ | array | 航点列表，至少 1 个 |
| `target_vel` | ❌ | float | 整段导航速度（m/s），默认 0.5 |

单个 local 航点字段：`x`（必填）、`y`（必填）、`yaw`（选填）  
单个 gps 航点字段：`lat`（必填）、`lon`（必填）、`yaw`（选填）

应答：
```json
{"ack":"nav_waypoints"}
```

---

### 3.3 停止导航 `nav_cancel`

```json
{"cmd":"nav_cancel"}
```

应答：
```json
{"ack":"nav_cancel"}
```

---

### 3.4 暂停 / 继续导航

```json
{"cmd":"nav_pause"}
{"cmd":"nav_resume"}
```

应答：
```json
{"ack":"nav_pause"}
{"ack":"nav_resume"}
```

---

### 3.5 查询当前状态 `query_status`

```json
{"cmd":"query_status"}
```

应答：同 §2.1 位置状态包格式，字段相同，按需查询时使用。

---

### 3.6 生命周期管理指令

这组指令控制 SLAM/RTK 节点的启停和定位模式切换，所有操作均**立即返回应答**，后台异步执行。

| 指令 | 效果 |
|------|------|
| `start_mapping` | 启动 FAST-LIO 建图节点；定位源切换到 `/Odometry` |
| `stop_mapping` | **先调用 `/map_save` service 保存地图**（最多等 15 秒），再停止建图进程 |
| `start_indoor_loc` | 停止建图（若在运行），启动室内定位（global_loc + transform_fusion）；定位源切换到 `/localization` |
| `stop_indoor_loc` | 停止室内定位节点 |
| `start_outdoor` | 停止室内节点，启动 RTK 节点；定位源切换到 `/outdoor/odom`；规划器切为直线模式 |
| `stop_outdoor` | 停止 RTK 节点；定位源切换回室内（`/localization` 或 `/Odometry`） |

> **注意**：系统启动（`start_nav.sh`）**不会**自动启动定位/建图节点，需手动发送 `start_mapping` 或 `start_indoor_loc` 指令。

**标准建图流程：**

```
start_mapping  →  [遥控探索建图]  →  stop_mapping  →  [地图自动保存到 PCD/scans.pcd]
```

**标准室内定位导航流程：**

```
start_indoor_loc  →  [等待定位稳定，pose 推送正常]  →  nav_goal / nav_waypoints
```

**室外 RTK 导航流程：**

```
start_outdoor  →  [等待 RTK 收星]  →  nav_goal(gps) / nav_waypoints(gps)  →  stop_outdoor
```

各指令格式：

```json
{"cmd":"start_mapping"}
{"cmd":"stop_mapping"}
{"cmd":"start_indoor_loc"}
{"cmd":"stop_indoor_loc"}
{"cmd":"start_outdoor"}
{"cmd":"stop_outdoor"}
```

应答示例（以 `start_mapping` 为例）：

```json
{"ack":"start_mapping"}
```

---

### 3.7 错误应答

当指令格式有误或命令未知时，Bridge 返回 error 包：

```json
{"error":"missing 'cmd' field"}
{"error":"nav_goal local mode requires x,y"}
{"error":"unknown cmd","supported":["nav_goal","nav_waypoints","nav_cancel","nav_pause","nav_resume","query_status","start_mapping","stop_mapping","start_indoor_loc","stop_indoor_loc","start_outdoor","stop_outdoor"]}
{"error":"json_parse_error","detail":"[101] parse error ..."}
```

---

## 4. PCD 文件传输（PCD 端口 9091）

建图后可通过 PCD 端口下载 `trans.pcd` 文件（实时转换后的点云地图）。传输采用**分块 Base64** 方式，与控制指令完全分离。

---

### 4.1 查询文件信息 `pcd_info`

**请求：**
```json
{"cmd":"pcd_info"}
```

**应答：**
```json
{
  "type": "pcd_info",
  "path": "/abs/path/to/trans.pcd",
  "exists": true,
  "size": 2097152,
  "chunk_bytes": 65536,
  "mtime_ns": 1745000000000000000
}
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `exists` | bool | 文件是否存在 |
| `size` | int | 文件总字节数 |
| `chunk_bytes` | int | 服务端每片最大字节数 |
| `mtime_ns` | int | 文件最后修改时间（ns） |

---

### 4.2 下载分片 `pcd_chunk`

**请求：**
```json
{"cmd":"pcd_chunk","offset":0,"length":65536}
```

| 字段 | 必填 | 类型 | 说明 |
|------|------|------|------|
| `offset` | ✅ | int | 读取起始字节偏移（0 起） |
| `length` | ❌ | int | 请求读取字节数，不填则用服务端默认值 |

**应答：**
```json
{
  "type": "pcd_chunk",
  "offset": 0,
  "length": 65536,
  "total_size": 2097152,
  "eof": false,
  "data_b64": "UENEIHYwLjcK..."
}
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `offset` | int | 本片起始偏移 |
| `length` | int | 本片实际字节数 |
| `total_size` | int | 文件总大小 |
| `eof` | bool | 是否已到文件末尾 |
| `data_b64` | string | 文件数据（Base64 编码） |

**下载循环伪代码：**
```
info = pcd_info()
offset = 0
while offset < info.size:
    chunk = pcd_chunk(offset, 65536)
    write(base64_decode(chunk.data_b64))
    offset += chunk.length
    if chunk.eof: break
```

---

## 5. 典型会话示例

### 示例 A：室内单次导航

```
[系统已通过 start_nav.sh 启动，定位节点需手动启动]

CLIENT → {"cmd":"start_indoor_loc"}
BRIDGE ← {"ack":"start_indoor_loc"}

[持续接收 pose 推送，等待定位就绪（pose 数据开始正常更新）]

CLIENT → {"cmd":"nav_goal","coord_mode":"local","x":3.0,"y":2.0,"yaw":0.0}
BRIDGE ← {"ack":"nav_goal"}

[持续接收 pose 推送：{"type":"pose","x":...,"y":..."yaw":...,"battery":24.6,"vx":0.35,"vy":0.0}]

[导航结束时收到事件包]
BRIDGE ← {"type":"nav_result","result":"REACHED"}
```

### 示例 B：室外多点 GPS 导航

```
CLIENT → {"cmd":"start_outdoor"}
BRIDGE ← {"ack":"start_outdoor"}

[等待 RTK 定位, status.nav.mode == "gps" 且 localization_valid == true]

CLIENT → {
  "cmd": "nav_waypoints",
  "coord_mode": "gps",
  "waypoints": [
    {"lat": 36.661230, "lon": 117.016880},
    {"lat": 36.661500, "lon": 117.017200}
  ],
  "target_vel": 0.8
}
BRIDGE ← {"ack":"nav_waypoints"}

[持续接收 pose 推送：{"type":"pose","lat":..."lon":...,"yaw":...,"battery":24.6,"vx":0.5,"vy":0.0}]

[导航结束时收到事件包]
BRIDGE ← {"type":"nav_result","result":"REACHED"}

CLIENT → {"cmd":"stop_outdoor"}
BRIDGE ← {"ack":"stop_outdoor"}
```

### 示例 C：下载点云地图

```
[PCD 端口 9091]
CLIENT → {"cmd":"pcd_info"}
BRIDGE ← {"type":"pcd_info","exists":true,"size":4194304,"chunk_bytes":65536,...}

CLIENT → {"cmd":"pcd_chunk","offset":0,"length":65536}
BRIDGE ← {"type":"pcd_chunk","offset":0,"length":65536,"eof":false,"data_b64":"..."}

CLIENT → {"cmd":"pcd_chunk","offset":65536,"length":65536}
...（循环直到 eof == true）
```

---

## 6. 注意事项

1. **帧结束符**：每条 JSON 必须以 `\n` 结尾，否则 Bridge 不会处理。
2. **并发**：一个客户端发多条指令时，应等待上一条应答后再发下一条。
3. **生命周期指令异步**：`start_*` / `stop_*` 应答仅表示指令已接收，子进程启停在后台进行，可观察 pose 包是否正常上报来判断定位是否就绪。
4. **GPS 坐标系**：室外导航使用平面投影（`111320.0 m/deg`），基准点为 RTK 节点启动时的首帧 GPS 坐标，所有 GPS 航点均相对此基准计算偏移。
5. **PCD 连接独立**：PCD 端口（9091）是独立服务，下载过程中不影响控制端口（9090）的指令收发。
