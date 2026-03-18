import streamlit as st

st.title("我的第一个 Streamlit 应用")
st.write("部署成功！")
import streamlit as st
import pandas as pd
import plotly.graph_objects as go
import plotly.express as px
from plotly.subplots import make_subplots
import time
import datetime
import random
import threading
import queue
from typing import Optional, Dict, List
from collections import deque
import numpy as np

# 设置页面配置
st.set_page_config(
    page_title="无人机心跳监控系统",
    page_icon="🚁",
    layout="wide"
)

# 自定义CSS样式
st.markdown("""
<style>
    .stAlert {
        background-color: #f0f2f6;
        padding: 1rem;
        border-radius: 0.5rem;
    }
    .css-1v3fvcr {
        background-color: #f0f2f6;
    }
    .drone-status-normal {
        color: #00ff00;
        font-weight: bold;
    }
    .drone-status-warning {
        color: #ffaa00;
        font-weight: bold;
    }
    .drone-status-critical {
        color: #ff0000;
        font-weight: bold;
        animation: blink 1s infinite;
    }
    @keyframes blink {
        0% { opacity: 1; }
        50% { opacity: 0.5; }
        100% { opacity: 1; }
    }
    .metric-card {
        background-color: #1e1e1e;
        padding: 1rem;
        border-radius: 0.5rem;
        color: white;
    }
    .st-emotion-cache-1y4p8pa {
        padding: 2rem 1rem;
    }
</style>
""", unsafe_allow_html=True)

class DroneHeartbeatSimulator:
    """无人机心跳包模拟器"""
    
    def __init__(self, drone_id: str = "DRONE-001"):
        self.drone_id = drone_id
        self.sequence_number = 0
        self.is_running = False
        self.heartbeat_queue = queue.Queue()
        self.simulation_thread: Optional[threading.Thread] = None
        self.simulate_connection_issue = False
        
    def generate_heartbeat(self) -> dict:
        current_time = datetime.datetime.now()
        
        heartbeat = {
            'drone_id': self.drone_id,
            'sequence': self.sequence_number,
            'timestamp': current_time.isoformat(),
            'timestamp_epoch': int(current_time.timestamp() * 1000),
            'status': random.choice(['normal', 'normal', 'normal', 'warning', 'normal']),
            'battery': round(random.uniform(65, 100), 1),
            'altitude': round(random.uniform(50, 150), 1),
            'speed': round(random.uniform(0, 20), 1),
            'signal_quality': random.randint(70, 100),
            'gps_satellites': random.randint(6, 12),
            'temperature': round(random.uniform(20, 45), 1),
        }
        
        return heartbeat
    
    def send_heartbeat(self) -> None:
        try:
            if self.simulate_connection_issue and random.random() < 0.3:
                self.sequence_number += 1
                return
            
            heartbeat = self.generate_heartbeat()
            self.heartbeat_queue.put(heartbeat)
            self.sequence_number += 1
            
        except Exception as e:
            print(f"发送心跳包时出错: {e}")
    
    def simulate_heartbeat_loop(self):
        while self.is_running:
            self.send_heartbeat()
            time.sleep(1)
            
    def start(self):
        if not self.is_running:
            self.is_running = True
            self.sequence_number = 0
            self.simulation_thread = threading.Thread(target=self.simulate_heartbeat_loop)
            self.simulation_thread.daemon = True
            self.simulation_thread.start()
    
    def stop(self):
        if self.is_running:
            self.is_running = False
            if self.simulation_thread:
                self.simulation_thread.join(timeout=2)
    
    def get_latest_heartbeat(self) -> Optional[dict]:
        try:
            return self.heartbeat_queue.get_nowait()
        except queue.Empty:
            return None
    
    def toggle_connection_issue(self):
        self.simulate_connection_issue = not self.simulate_connection_issue


class GroundStation:
    """地面站 - 带连接超时报警功能"""
    
    ALARM_NONE = 0
    ALARM_WARNING = 1
    ALARM_CRITICAL = 2
    
    def __init__(self, station_id: str = "GROUND-STATION-001", timeout_seconds: int = 3):
        self.station_id = station_id
        self.timeout_threshold = timeout_seconds
        self.is_running = False
        self.monitor_thread: Optional[threading.Thread] = None
        self.alarm_thread: Optional[threading.Thread] = None
        
        self.received_heartbeats: Dict[str, deque] = {}
        self.last_heartbeat_time: Dict[str, float] = {}
        self.alarm_status: Dict[str, int] = {}
        self.alarm_history: List[dict] = []
        self.sequence_history: List[dict] = []  # 用于存储序号历史
        self.lock = threading.Lock()
        
    def register_drone(self, drone_id: str):
        with self.lock:
            if drone_id not in self.received_heartbeats:
                self.received_heartbeats[drone_id] = deque(maxlen=1000)
                self.last_heartbeat_time[drone_id] = time.time()
                self.alarm_status[drone_id] = self.ALARM_NONE
    
    def process_heartbeat(self, heartbeat: dict):
        drone_id = heartbeat['drone_id']
        current_time = time.time()
        
        with self.lock:
            if drone_id not in self.received_heartbeats:
                self.register_drone(drone_id)
            
            self.received_heartbeats[drone_id].append({
                'heartbeat': heartbeat,
                'receive_time': current_time
            })
            
            # 记录序号历史（用于绘图）
            self.sequence_history.append({
                'time': datetime.datetime.now(),
                'sequence': heartbeat['sequence'],
                'drone_id': drone_id,
                'receive_interval': current_time - self.last_heartbeat_time[drone_id] if self.last_heartbeat_time[drone_id] else 0
            })
            
            self.last_heartbeat_time[drone_id] = current_time
            
            if self.alarm_status[drone_id] != self.ALARM_NONE:
                old_status = self.alarm_status[drone_id]
                self.alarm_status[drone_id] = self.ALARM_NONE
                
                alarm_record = {
                    'drone_id': drone_id,
                    'type': 'recovery',
                    'timestamp': datetime.datetime.now().isoformat(),
                    'message': f'无人机 {drone_id} 连接已恢复'
                }
                self.alarm_history.append(alarm_record)
    
    def check_connection_timeout(self):
        while self.is_running:
            current_time = time.time()
            
            with self.lock:
                for drone_id in list(self.last_heartbeat_time.keys()):
                    last_time = self.last_heartbeat_time[drone_id]
                    time_diff = current_time - last_time
                    current_status = self.alarm_status[drone_id]
                    
                    if time_diff > self.timeout_threshold:
                        if time_diff > self.timeout_threshold * 3:
                            new_status = self.ALARM_CRITICAL
                        elif time_diff > self.timeout_threshold * 2:
                            new_status = self.ALARM_WARNING
                        else:
                            new_status = self.ALARM_WARNING
                        
                        if new_status != current_status:
                            self.alarm_status[drone_id] = new_status
                            self.trigger_alarm(drone_id, time_diff, new_status)
            
            time.sleep(1)
    
    def trigger_alarm(self, drone_id: str, time_diff: float, status: int):
        status_text = "警告" if status == self.ALARM_WARNING else "严重"
        
        alarm_record = {
            'drone_id': drone_id,
            'type': 'timeout',
            'severity': status_text,
            'timeout_seconds': round(time_diff, 1),
            'timestamp': datetime.datetime.now().isoformat(),
            'message': f'无人机 {drone_id} 已 {time_diff:.1f} 秒无响应！'
        }
        
        self.alarm_history.append(alarm_record)
    
    def monitor_heartbeats(self, drone_simulator: DroneHeartbeatSimulator):
        self.register_drone(drone_simulator.drone_id)
        
        while self.is_running:
            heartbeat = drone_simulator.get_latest_heartbeat()
            if heartbeat:
                self.process_heartbeat(heartbeat)
            time.sleep(0.1)
    
    def start(self, drone_simulator: DroneHeartbeatSimulator):
        if not self.is_running:
            self.is_running = True
            
            self.monitor_thread = threading.Thread(
                target=self.monitor_heartbeats,
                args=(drone_simulator,)
            )
            self.monitor_thread.daemon = True
            self.monitor_thread.start()
            
            self.alarm_thread = threading.Thread(
                target=self.check_connection_timeout
            )
            self.alarm_thread.daemon = True
            self.alarm_thread.start()
    
    def stop(self):
        self.is_running = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=2)
        if self.alarm_thread:
            self.alarm_thread.join(timeout=2)
    
    def get_drone_status(self, drone_id: str) -> str:
        with self.lock:
            if drone_id not in self.alarm_status:
                return "未注册"
            
            status = self.alarm_status[drone_id]
            if status == self.ALARM_NONE:
                return "正常"
            elif status == self.ALARM_WARNING:
                return "警告"
            else:
                return "严重失联"


# 初始化session state
if 'drone' not in st.session_state:
    st.session_state.drone = DroneHeartbeatSimulator(drone_id="DJI-MAVIC-3")
    st.session_state.drone.start()

if 'ground_station' not in st.session_state:
    st.session_state.ground_station = GroundStation(
        station_id="GCS-BEIJING-001",
        timeout_seconds=3
    )
    st.session_state.ground_station.start(st.session_state.drone)

if 'auto_refresh' not in st.session_state:
    st.session_state.auto_refresh = True

if 'last_update' not in st.session_state:
    st.session_state.last_update = time.time()


# 主界面
st.title("🚁 无人机心跳监控系统")
st.markdown("---")

# 侧边栏控制面板
with st.sidebar:
    st.header("🕹️ 控制面板")
    
    # 无人机设置
    st.subheader("无人机设置")
    new_drone_id = st.text_input("无人机ID", value=st.session_state.drone.drone_id)
    if new_drone_id != st.session_state.drone.drone_id:
        st.session_state.drone.drone_id = new_drone_id
    
    # 地面站设置
    st.subheader("地面站设置")
    timeout = st.slider("超时阈值（秒）", min_value=1, max_value=10, value=3)
    if timeout != st.session_state.ground_station.timeout_threshold:
        st.session_state.ground_station.timeout_threshold = timeout
    
    # 模拟控制
    st.subheader("模拟控制")
    col1, col2 = st.columns(2)
    with col1:
        if st.button("启动/停止"):
            if st.session_state.drone.is_running:
                st.session_state.drone.stop()
                st.session_state.ground_station.stop()
            else:
                st.session_state.drone.start()
                st.session_state.ground_station.start(st.session_state.drone)
    
    with col2:
        if st.button("切换丢包模拟"):
            st.session_state.drone.toggle_connection_issue()
    
    # 显示当前状态
    st.subheader("当前状态")
    status_color = {
        "正常": "🟢",
        "警告": "🟡",
        "严重失联": "🔴",
        "未注册": "⚪"
    }
    current_status = st.session_state.ground_station.get_drone_status(st.session_state.drone.drone_id)
    st.markdown(f"无人机状态: {status_color.get(current_status, '⚪')} **{current_status}**")
    st.markdown(f"丢包模拟: {'🔴 开启' if st.session_state.drone.simulate_connection_issue else '🟢 关闭'}")
    
    # 刷新设置
    st.subheader("刷新设置")
    st.session_state.auto_refresh = st.checkbox("自动刷新", value=True)
    refresh_rate = st.slider("刷新频率（秒）", min_value=1, max_value=5, value=2)
    
    if st.button("手动刷新"):
        st.session_state.last_update = 0
        st.rerun()


# 主内容区域 - 使用标签页组织
tab1, tab2, tab3 = st.tabs(["📊 实时监控", "📈 数据分析", "🚨 报警记录"])

with tab1:
    # 第一行：关键指标
    col1, col2, col3, col4 = st.columns(4)
    
    with col1:
        with st.container():
            st.metric(
                label="心跳包序号",
                value=st.session_state.drone.sequence_number,
                delta=st.session_state.drone.sequence_number - (st.session_state.get('last_seq', 0))
            )
            st.session_state.last_seq = st.session_state.drone.sequence_number
    
    with col2:
        last_heartbeat_time = st.session_state.ground_station.last_heartbeat_time.get(
            st.session_state.drone.drone_id, time.time()
        )
        time_since_last = time.time() - last_heartbeat_time
        st.metric(
            label="最后心跳时间",
            value=f"{time_since_last:.1f}秒前",
            delta=f"阈值: {timeout}秒",
            delta_color="inverse" if time_since_last > timeout else "normal"
        )
    
    with col3:
        received_count = len(st.session_state.ground_station.received_heartbeats.get(
            st.session_state.drone.drone_id, []
        ))
        st.metric(
            label="已接收心跳",
            value=received_count,
            delta=f"丢失: {st.session_state.drone.sequence_number - received_count}"
        )
    
    with col4:
        alarm_count = len([a for a in st.session_state.ground_station.alarm_history 
                          if a['type'] == 'timeout'])
        st.metric(
            label="报警次数",
            value=alarm_count,
            delta="最近24小时"
        )
    
    st.markdown("---")
    
    # 第二行：心跳序号折线图
    st.subheader("📈 心跳序号实时监控")
    
    # 获取历史数据
    history = st.session_state.ground_station.sequence_history[-50:]  # 最近50个数据点
    
    if history:
        df = pd.DataFrame(history)
        
        # 创建折线图
        fig = make_subplots(
            rows=2, cols=1,
            subplot_titles=('心跳序号随时间变化', '心跳接收间隔'),
            vertical_spacing=0.15
        )
        
        # 心跳序号图
        fig.add_trace(
            go.Scatter(
                x=df['time'],
                y=df['sequence'],
                mode='lines+markers',
                name='心跳序号',
                line=dict(color='#00ff00', width=2),
                marker=dict(size=6, color='#00ff00')
            ),
            row=1, col=1
        )
        
        # 添加理论线（每秒递增1）
        if len(df) > 1:
            start_seq = df['sequence'].iloc[0]
            theoretical_seq = [start_seq + i for i in range(len(df))]
            fig.add_trace(
                go.Scatter(
                    x=df['time'],
                    y=theoretical_seq,
                    mode='lines',
                    name='理论值',
                    line=dict(color='#ffaa00', width=2, dash='dash')
                ),
                row=1, col=1
            )
        
        # 心跳间隔图
        fig.add_trace(
            go.Scatter(
                x=df['time'],
                y=df['receive_interval'],
                mode='lines+markers',
                name='接收间隔',
                line=dict(color='#ffaa00', width=2),
                marker=dict(size=6, color='#ffaa00')
            ),
            row=2, col=1
        )
        
        # 添加超时阈值线
        fig.add_hline(
            y=timeout,
            line_dash="dash",
            line_color="red",
            annotation_text=f"超时阈值 ({timeout}秒)",
            row=2, col=1
        )
        
        # 更新布局
        fig.update_layout(
            height=600,
            showlegend=True,
            template='plotly_dark',
            hovermode='x unified'
        )
        
        fig.update_xaxes(title_text="时间", row=2, col=1)
        fig.update_yaxes(title_text="序号", row=1, col=1)
        fig.update_yaxes(title_text="间隔（秒）", row=2, col=1)
        
        st.plotly_chart(fig, use_container_width=True)
    else:
        st.info("等待接收心跳数据...")
    
    st.markdown("---")
    
    # 第三行：实时数据流
    st.subheader("📡 实时心跳数据流")
    
    # 获取最新的心跳包
    latest_heartbeats = list(st.session_state.ground_station.received_heartbeats.get(
        st.session_state.drone.drone_id, []
    ))[-10:]  # 最近10条
    
    if latest_heartbeats:
        # 创建数据表格
        data = []
        for item in reversed(latest_heartbeats):
            hb = item['heartbeat']
            data.append({
                '时间': datetime.datetime.fromtimestamp(item['receive_time']).strftime('%H:%M:%S'),
                '序号': hb['sequence'],
                '电量': f"{hb['battery']}%",
                '高度': f"{hb['altitude']}m",
                '速度': f"{hb['speed']}m/s",
                '信号': f"{hb['signal_quality']}%",
                '温度': f"{hb['temperature']}°C"
            })
        
        df_latest = pd.DataFrame(data)
        st.dataframe(
            df_latest,
            use_container_width=True,
            hide_index=True,
            column_config={
                '时间': st.column_config.TextColumn('时间', width='small'),
                '序号': st.column_config.NumberColumn('序号', width='small'),
                '电量': st.column_config.TextColumn('电量', width='small'),
                '高度': st.column_config.TextColumn('高度', width='small'),
                '速度': st.column_config.TextColumn('速度', width='small'),
                '信号': st.column_config.TextColumn('信号', width='small'),
                '温度': st.column_config.TextColumn('温度', width='small')
            }
        )
    else:
        st.info("暂无心跳数据")

with tab2:
    st.header("📈 数据分析")
    
    # 获取所有历史数据
    all_history = st.session_state.ground_station.sequence_history
    
    if all_history:
        df_all = pd.DataFrame(all_history)
        
        # 统计信息
        col1, col2, col3 = st.columns(3)
        
        with col1:
            avg_interval = df_all['receive_interval'].mean()
            st.metric("平均接收间隔", f"{avg_interval:.2f}秒")
        
        with col2:
            max_interval = df_all['receive_interval'].max()
            st.metric("最大接收间隔", f"{max_interval:.2f}秒")
        
        with col3:
            timeout_count = len(df_all[df_all['receive_interval'] > timeout])
            st.metric("超时次数", timeout_count)
        
        # 间隔分布直方图
        st.subheader("心跳间隔分布")
        fig_hist = px.histogram(
            df_all,
            x='receive_interval',
            nbins=30,
            title="心跳接收间隔分布",
            labels={'receive_interval': '接收间隔（秒）', 'count': '次数'},
            template='plotly_dark'
        )
        fig_hist.add_vline(x=timeout, line_dash="dash", line_color="red")
        st.plotly_chart(fig_hist, use_container_width=True)
        
        # 序号增量分析
        st.subheader("序号增量分析")
        df_all['seq_diff'] = df_all['sequence'].diff()
        fig_seq = px.line(
            df_all,
            x='time',
            y='seq_diff',
            title="心跳序号增量（正常应为1）",
            labels={'time': '时间', 'seq_diff': '序号增量'},
            template='plotly_dark'
        )
        fig_seq.add_hline(y=1, line_dash="dash", line_color="green")
        st.plotly_chart(fig_seq, use_container_width=True)
    else:
        st.info("暂无足够数据进行分析")

with tab3:
    st.header("🚨 报警记录")
    
    # 报警过滤
    alarm_filter = st.selectbox(
        "过滤报警类型",
        ["全部", "超时报警", "恢复记录"]
    )
    
    # 获取报警历史
    alarm_history = st.session_state.ground_station.alarm_history
    
    if alarm_filter == "超时报警":
        alarm_history = [a for a in alarm_history if a['type'] == 'timeout']
    elif alarm_filter == "恢复记录":
        alarm_history = [a for a in alarm_history if a['type'] == 'recovery']
    
    if alarm_history:
        # 创建报警表格
        alarm_data = []
        for alarm in reversed(alarm_history[-50:]):  # 最近50条
            if alarm['type'] == 'timeout':
                alarm_data.append({
                    '时间': alarm['timestamp'][11:19],
                    '类型': f"🚨 {alarm['severity']}报警",
                    '消息': alarm['message'],
                    '超时时长': f"{alarm['timeout_seconds']}秒"
                })
            else:
                alarm_data.append({
                    '时间': alarm['timestamp'][11:19],
                    '类型': "✅ 恢复",
                    '消息': alarm['message'],
                    '超时时长': "-"
                })
        
        df_alarms = pd.DataFrame(alarm_data)
        st.dataframe(
            df_alarms,
            use_container_width=True,
            hide_index=True,
            column_config={
                '时间': st.column_config.TextColumn('时间', width='small'),
                '类型': st.column_config.TextColumn('类型', width='medium'),
                '消息': st.column_config.TextColumn('消息', width='large'),
                '超时时长': st.column_config.TextColumn('超时时长', width='small')
            }
        )
        
        # 报警统计
        st.subheader("报警统计")
        timeout_count = len([a for a in alarm_history if a['type'] == 'timeout'])
        recovery_count = len([a for a in alarm_history if a['type'] == 'recovery'])
        
        col1, col2 = st.columns(2)
        with col1:
            st.metric("超时报警次数", timeout_count)
        with col2:
            st.metric("恢复次数", recovery_count)
    else:
        st.info("暂无报警记录")


# 自动刷新逻辑
if st.session_state.auto_refresh:
    time_since_update = time.time() - st.session_state.last_update
    if time_since_update >= refresh_rate:
        st.session_state.last_update = time.time()
        st.rerun()


# 在页面底部显示当前时间
st.markdown("---")
col1, col2, col3 = st.columns(3)
with col1:
    st.caption(f"当前时间: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
with col2:
    st.caption(f"系统运行中...")
with col3:
    if st.session_state.drone.is_running:
        st.caption("🟢 无人机在线")
    else:
        st.caption("🔴 无人机离线")
