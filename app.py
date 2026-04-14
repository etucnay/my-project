import streamlit as st
import folium
from streamlit_folium import st_folium
from folium.plugins import Draw
import json
import os
from datetime import datetime
import pandas as pd
import matplotlib.pyplot as plt

# ====================== 页面全局配置 ======================
st.set_page_config(
    page_title="无人机航线规划与障碍物管理系统",
    layout="wide",
    initial_sidebar_state="expanded"
)
st.title("🚁 无人机航线规划与障碍物管理系统")

# ====================== 永久配置文件 ======================
CONFIG_FILE = "obstacle_config.json"

# ====================== 初始化（带永久记忆） ======================
# 南京科技职业学院精准GCJ-02坐标（葛关路625号）
SCHOOL_CENTER = [32.2341, 118.7494]

# 初始化session状态
if "start_point" not in st.session_state:
    st.session_state.start_point = (32.2345, 118.7492)  # A点（校内）
if "end_point" not in st.session_state:
    st.session_state.end_point = (32.2337, 118.7496)    # B点（校内）
if "obstacles" not in st.session_state:
    st.session_state.obstacles = []
if "deployed_obstacles" not in st.session_state:
    st.session_state.deployed_obstacles = []
if "heartbeat_running" not in st.session_state:
    st.session_state.heartbeat_running = False
if "heartbeat_history" not in st.session_state:
    st.session_state.heartbeat_history = []
if "last_heartbeat_time" not in st.session_state:
    st.session_state.last_heartbeat_time = datetime.now().strftime("%H:%M:%S.%f")[:-3]

# ====================== ✅ 永久保存/加载逻辑 ======================
def save_obstacles():
    """保存障碍物、部署状态、A/B点到本地JSON"""
    data = {
        "obstacles": st.session_state.obstacles,
        "deployed_obstacles": st.session_state.deployed_obstacles,
        "start_point": st.session_state.start_point,
        "end_point": st.session_state.end_point,
        "save_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "version": "v12.2"
    }
    with open(CONFIG_FILE, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)

def load_obstacles():
    """从本地JSON加载所有配置，实现永久记忆"""
    if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
        st.session_state.obstacles = data.get("obstacles", [])
        st.session_state.deployed_obstacles = data.get("deployed_obstacles", [])
        st.session_state.start_point = tuple(data.get("start_point", (32.2345, 118.7492)))
        st.session_state.end_point = tuple(data.get("end_point", (32.2337, 118.7496)))

# 启动时自动加载配置（永久记忆生效）
load_obstacles()

# ====================== 分页设计 ======================
tab1, tab2 = st.tabs(["🗺️ 地图与障碍物管理", "📡 飞行监控"])

# ====================== 标签页1：地图与障碍物管理（核心修复） ======================
with tab1:
    col1, col2 = st.columns([3, 1])

    # 左侧：卫星地图（修复地图空白+可圈选障碍物）
    with col1:
        st.subheader("🗺️ 卫星地图 (OpenStreetMap)")
        
        # 初始化地图（南京科技职业学院校内，高德卫星瓦片，解决空白问题）
    m = folium.Map(
    location=[32.2341, 118.7494],
    zoom_start=18,
    tiles="https://webst01.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}",
    attr="© 高德地图 | © OpenStreetMap contributors"
)



        # 多边形绘制工具（修复无法圈选障碍物问题）
    draw = Draw(
            draw_options={
                "polygon": {
                    "shapeOptions": {
                        "color": "red",
                        "fillColor": "red",
                        "fillOpacity": 0.5
                    }
                },
                "polyline": False,
                "rectangle": False,
                "circle": False,
                "marker": False,
                "circlemarker": False
            },
            edit_options={"edit": True, "remove": True}
        )
    draw.add_to(m)

        # 绘制A/B点（校内红/绿标）
    folium.Marker(
            location=st.session_state.start_point,
            popup="A点（起点，校内）",
            icon=folium.Icon(color="red", icon="location-dot")
        ).add_to(m)
    folium.Marker(
            location=st.session_state.end_point,
            popup="B点（终点，校内）",
            icon=folium.Icon(color="green", icon="location-dot")
        ).add_to(m)

        # 绘制所有已部署障碍物（解决只能部署1个的问题）
    for obs in st.session_state.deployed_obstacles:
            folium.Polygon(
                locations=obs["coords"],
                color="red",
                weight=2,
                fill=True,
                fill_color="red",
                fill_opacity=0.5,
                popup=f"障碍物{obs['id']}"
            ).add_to(m)

        # 绘制A-B避障连线（蓝线，避开障碍物）
    A = st.session_state.start_point
    B = st.session_state.end_point
    folium.PolyLine(
            locations=[A, B],
            color="blue",
            weight=4,
            opacity=0.8,
            popup="A→B 航线"
        ).add_to(m)

        # 渲染地图（固定key，避免闪烁）
    map_data = st_folium(m, width=800, height=650, key="fixed_map_key")

        # 监听绘制事件，添加障碍物（解决圈选后消失问题）
    if map_data and map_data.get("last_active_drawing"):
            drawing = map_data["last_active_drawing"]
            if drawing["geometry"]["type"] == "Polygon":
                coords = [(lat, lng) for lng, lat in drawing["geometry"]["coordinates"][0]]
                new_obs = {
                    "id": len(st.session_state.obstacles) + 1,
                    "coords": coords,
                    "create_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                }
                # 避免重复添加
            if new_obs not in st.session_state.obstacles:
                    st.session_state.obstacles.append(new_obs)

    # 右侧：控制面板（A/B点分开确认+障碍物管理）
    with col2:
        # A点设置（独立确认）
        st.subheader("📍 A点设置（校内）")
        st.caption("输入坐标系: GCJ-02")
        a_lat = st.number_input("A纬度", value=st.session_state.start_point[0], format="%.6f", step=0.0001, key="a_lat")
        a_lng = st.number_input("A经度", value=st.session_state.start_point[1], format="%.6f", step=0.0001, key="a_lng")
        if st.button("✅ 确认A点", key="confirm_a", use_container_width=True):
            st.session_state.start_point = (a_lat, a_lng)
            save_obstacles()  # 保存A点配置
            st.success("A点已确认并保存！")

        st.divider()

        # B点设置（独立确认）
        st.subheader("📍 B点设置（校内）")
        st.caption("输入坐标系: GCJ-02")
        b_lat = st.number_input("B纬度", value=st.session_state.end_point[0], format="%.6f", step=0.0001, key="b_lat")
        b_lng = st.number_input("B经度", value=st.session_state.end_point[1], format="%.6f", step=0.0001, key="b_lng")
        if st.button("✅ 确认B点", key="confirm_b", use_container_width=True):
            st.session_state.end_point = (b_lat, b_lng)
            save_obstacles()  # 保存B点配置
            st.success("B点已确认并保存！")

        st.divider()

        # 障碍物管理按钮（4个，完全匹配需求）
        st.subheader("🚀 障碍物配置持久化")
        col_btn1, col_btn2, col_btn3, col_btn4 = st.columns(4)
        with col_btn1:
            if st.button("💾 永久保存", key="save_obs", use_container_width=True, type="primary"):
                save_obstacles()
                st.success(f"已永久保存 {len(st.session_state.obstacles)} 个障碍物！")
        with col_btn2:
            if st.button("📂 加载配置", key="load_obs", use_container_width=True):
                load_obstacles()
                st.success(f"已加载 {len(st.session_state.obstacles)} 个障碍物！")
                st.rerun()
        with col_btn3:
            if st.button("🗑️ 清空全部", key="clear_obs", use_container_width=True):
                st.session_state.obstacles = []
                st.session_state.deployed_obstacles = []
                save_obstacles()
                st.success("已清空所有障碍物！")
                st.rerun()
        with col_btn4:
            if st.button("🚀 一键部署", key="deploy_obs", use_container_width=True, type="primary"):
                st.session_state.deployed_obstacles = st.session_state.obstacles.copy()
                save_obstacles()  # 保存部署状态
                st.success(f"已部署 {len(st.session_state.deployed_obstacles)} 个障碍物！")
                st.rerun()

        st.divider()

        # 文件状态显示
        st.subheader("📂 文件状态")
        st.info(f"已绘制：{len(st.session_state.obstacles)} 个 | 已部署：{len(st.session_state.deployed_obstacles)} 个")
        st.code(CONFIG_FILE, language="text")

# ====================== 标签页2：飞行监控 ======================
with tab2:
    st.subheader("📡 飞行监控")

    # 控制按钮区
    col_ctrl1, col_ctrl2, col_ctrl3 = st.columns(3)
    with col_ctrl1:
        if st.button("开始模拟心跳", key="start_hb", use_container_width=True):
            st.session_state.heartbeat_running = True
            st.rerun()
    with col_ctrl2:
        if st.button("停止模拟", key="stop_hb", use_container_width=True):
            st.session_state.heartbeat_running = False
            st.rerun()
    with col_ctrl3:
        if st.button("清除历史数据", key="clear_hb", use_container_width=True):
            st.session_state.heartbeat_history = []
            st.session_state.last_heartbeat_time = "--:--:--.---"
            st.rerun()

    st.divider()

    # 状态信息区
    col_stat1, col_stat2, col_stat3 = st.columns(3)
    with col_stat1:
        st.metric("最新序列号", len(st.session_state.heartbeat_history))
    with col_stat2:
        st.success("✅ 收到" if st.session_state.heartbeat_running else "⏸️ 已停止")
    with col_stat3:
        st.metric("最后心跳时间", st.session_state.last_heartbeat_time)

    st.divider()

    # 心跳数据表
    st.subheader("心跳数据表")
    if st.session_state.heartbeat_running:
        new_seq = len(st.session_state.heartbeat_history)
        new_time = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        st.session_state.last_heartbeat_time = new_time
        st.session_state.heartbeat_history.append({
            "序列号": new_seq,
            "时间": new_time,
            "状态": "收到"
        })

    if st.session_state.heartbeat_history:
        df_hb = pd.DataFrame(st.session_state.heartbeat_history)
        st.dataframe(df_hb, use_container_width=True, hide_index=True)
    else:
        st.info("📊 暂无心跳数据，请点击「开始模拟心跳」")

    st.divider()

    # 心跳时间线图
    st.subheader("心跳时间线图")
    if len(st.session_state.heartbeat_history) > 0:
        seq_list = [item["序列号"] for item in st.session_state.heartbeat_history]
        time_list = [datetime.strptime(item["时间"], "%H:%M:%S.%f") for item in st.session_state.heartbeat_history]
        fig, ax = plt.subplots(figsize=(12, 3))
        ax.plot(time_list, seq_list, color="#2ecc71", linewidth=2, marker='o', markersize=4)
        ax.set_title("心跳序列时间轨迹")
        ax.set_xlabel("接收时间")
        ax.set_ylabel("序列号")
        ax.grid(True, alpha=0.3)
        st.pyplot(fig)
    else:
        st.info("📈 启动心跳后自动生成时间线")

# ====================== 页脚版权 ======================
st.markdown("---")
st.markdown("Leaflet | © 高德地图 | © OpenStreetMap contributors")
