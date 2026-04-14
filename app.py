import streamlit as st
import folium
from streamlit_folium import st_folium
from folium.plugins import Draw
import json
import os
from datetime import datetime
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

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
SCHOOL_CENTER = [32.2341, 118.7494]

if "start_point" not in st.session_state:
    st.session_state.start_point = (32.2345, 118.7492)
if "end_point" not in st.session_state:
    st.session_state.end_point = (32.2337, 118.7496)
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
if "flight_altitude" not in st.session_state:
    st.session_state.flight_altitude = 10.0
if "safety_radius" not in st.session_state:
    st.session_state.safety_radius = 5.0
if "current_route_mode" not in st.session_state:
    st.session_state.current_route_mode = "best"

# ====================== 永久保存/加载逻辑（兼容旧数据） ======================
def save_obstacles():
    data = {
        "obstacles": st.session_state.obstacles,
        "deployed_obstacles": st.session_state.deployed_obstacles,
        "start_point": st.session_state.start_point,
        "end_point": st.session_state.end_point,
        "flight_altitude": st.session_state.flight_altitude,
        "safety_radius": st.session_state.safety_radius,
        "current_route_mode": st.session_state.current_route_mode,
        "save_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "version": "v18.0_KeyError修复版"
    }
    with open(CONFIG_FILE, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)

def load_obstacles():
    if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
        # 加载障碍物，兼容旧数据（补全缺失字段）
        loaded_obs = data.get("obstacles", [])
        for obs in loaded_obs:
            # 补全缺失的create_time字段，避免KeyError
            if "create_time" not in obs:
                obs["create_time"] = "未知"
        st.session_state.obstacles = loaded_obs
        
        # 加载已部署障碍物，同样兼容旧数据
        loaded_deployed = data.get("deployed_obstacles", [])
        for obs in loaded_deployed:
            if "create_time" not in obs:
                obs["create_time"] = "未知"
        st.session_state.deployed_obstacles = loaded_deployed
        
        st.session_state.start_point = tuple(data.get("start_point", (32.2345, 118.7492)))
        st.session_state.end_point = tuple(data.get("end_point", (32.2337, 118.7496)))
        st.session_state.flight_altitude = data.get("flight_altitude", 10.0)
        st.session_state.safety_radius = data.get("safety_radius", 5.0)
        st.session_state.current_route_mode = data.get("current_route_mode", "best")

load_obstacles()

# ====================== 纯原生航线规划算法 ======================
def calculate_safe_path(start, end, obstacles, safety_radius, mode="best"):
    start_lat, start_lng = start
    end_lat, end_lng = end
    
    # 高度判断：直飞/绕行
    max_obstacle_height = max([obs["height"] for obs in obstacles]) if obstacles else 0
    if st.session_state.flight_altitude > max_obstacle_height:
        return [start, end]
    
    # 计算方向向量
    lat_mid = (start_lat + end_lat) / 2
    lng_per_m = 1 / (111320 * np.cos(np.radians(lat_mid)))
    lat_per_m = 1 / 111320
    
    dx = end_lng - start_lng
    dy = end_lat - start_lat
    dist = np.sqrt(dx**2 + dy**2)
    if dist < 1e-6:
        return [start, end]
    
    ux = dx / dist
    uy = dy / dist
    nx = -uy
    ny = ux
    
    offset_m = safety_radius * 1.5
    offset_lat = offset_m * lat_per_m
    offset_lng = offset_m * lng_per_m
    
    if mode == "left":
        via1 = (start_lat + ny * offset_lat, start_lng + nx * offset_lng)
        via2 = (end_lat + ny * offset_lat, end_lng + nx * offset_lng)
        return [start, via1, via2, end]
    elif mode == "right":
        via1 = (start_lat - ny * offset_lat, start_lng - nx * offset_lng)
        via2 = (end_lat - ny * offset_lat, end_lng - nx * offset_lng)
        return [start, via1, via2, end]
    else:
        via1 = (start_lat + ny * offset_lat * 0.5, start_lng + nx * offset_lng * 0.5)
        via2 = (end_lat + ny * offset_lat * 0.5, end_lng + nx * offset_lng * 0.5)
        return [start, via1, via2, end]

# ====================== 分页设计 ======================
tab1, tab2 = st.tabs(["🗺️ 地图与航线规划", "📡 飞行监控"])

# ====================== 标签页1：地图与障碍物管理 ======================
with tab1:
    col1, col2 = st.columns([3, 1])
    with col1:
        st.subheader("🗺️ 卫星地图 (高德)")
        m = folium.Map(
            location=[32.2341, 118.7494],
            zoom_start=18,
            tiles="https://webst01.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}",
            attr="© 高德地图 | © OpenStreetMap contributors"
        )

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

        # 绘制A/B点
        folium.Marker(
            location=st.session_state.start_point,
            popup=f"A点（起点）\n坐标: {st.session_state.start_point[0]:.6f}, {st.session_state.start_point[1]:.6f}",
            icon=folium.Icon(color="red", icon="location-dot")
        ).add_to(m)

        folium.Marker(
            location=st.session_state.end_point,
            popup=f"B点（终点）\n坐标: {st.session_state.end_point[0]:.6f}, {st.session_state.end_point[1]:.6f}",
            icon=folium.Icon(color="green", icon="location-dot")
        ).add_to(m)

        # 绘制已部署障碍物（安全读取字段，避免KeyError）
        if "create_time" not in obs:
                obs["create_time"] = "未知"
        st.session_state.obstacles = loaded_obs
        
        # 加载已部署障碍物，同样兼容旧数据
        loaded_deployed = data.get("deployed_obstacles", [])
        for obs in loaded_deployed:
            if "create_time" not in obs:
                obs["create_time"] = "未知"
        st.session_state.deployed_obstacles = loaded_deployed
        
        st.session_state.start_point = tuple(data.get("start_point", (32.2345, 118.7492)))
        st.session_state.end_point = tuple(data.get("end_point", (32.2337, 118.7496)))
        st.session_state.flight_altitude = data.get("flight_altitude", 10.0)
        st.session_state.safety_radius = data.get("safety_radius", 5.0)
        st.session_state.current_route_mode = data.get("current_route_mode", "best")

load_obstacles()

# ====================== 纯原生航线规划算法 ======================
def calculate_safe_path(start, end, obstacles, safety_radius, mode="best"):
    start_lat, start_lng = start
    end_lat, end_lng = end
    
    # 高度判断：直飞/绕行
    max_obstacle_height = max([obs["height"] for obs in obstacles]) if obstacles else 0
    if st.session_state.flight_altitude > max_obstacle_height:
        return [start, end]
    
    # 计算方向向量
    lat_mid = (start_lat + end_lat) / 2
    lng_per_m = 1 / (111320 * np.cos(np.radians(lat_mid)))
    lat_per_m = 1 / 111320
    
    dx = end_lng - start_lng
    dy = end_lat - start_lat
    dist = np.sqrt(dx**2 + dy**2)
    if dist < 1e-6:
        return [start, end]
    
    ux = dx / dist
    uy = dy / dist
    nx = -uy
    ny = ux
    
    offset_m = safety_radius * 1.5
    offset_lat = offset_m * lat_per_m
    offset_lng = offset_m * lng_per_m
    
    if mode == "left":
        via1 = (start_lat + ny * offset_lat, start_lng + nx * offset_lng)
        via2 = (end_lat + ny * offset_lat, end_lng + nx * offset_lng)
        return [start, via1, via2, end]
    elif mode == "right":
        via1 = (start_lat - ny * offset_lat, start_lng - nx * offset_lng)
        via2 = (end_lat - ny * offset_lat, end_lng - nx * offset_lng)
        return [start, via1, via2, end]
    else:
        via1 = (start_lat + ny * offset_lat * 0.5, start_lng + nx * offset_lng * 0.5)
        via2 = (end_lat + ny * offset_lat * 0.5, end_lng + nx * offset_lng * 0.5)
        return [start, via1, via2, end]

# ====================== 分页设计 ======================
tab1, tab2 = st.tabs(["🗺️ 地图与航线规划", "📡 飞行监控"])

# ====================== 标签页1：地图与障碍物管理 ======================
with tab1:
    col1, col2 = st.columns([3, 1])
    with col1:
        st.subheader("🗺️ 卫星地图 (高德)")
        m = folium.Map(
            location=[32.2341, 118.7494],
            zoom_start=18,
            tiles="https://webst01.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}",
            attr="© 高德地图 | © OpenStreetMap contributors"
        )

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

        # 绘制A/B点
        folium.Marker(
            location=st.session_state.start_point,
            popup=f"A点（起点）\n坐标: {st.session_state.start_point[0]:.6f}, {st.session_state.start_point[1]:.6f}",
            icon=folium.Icon(color="red", icon="location-dot")
        ).add_to(m)

        folium.Marker(
            location=st.session_state.end_point,
            popup=f"B点（终点）\n坐标: {st.session_state.end_point[0]:.6f}, {st.session_state.end_point[1]:.6f}",
            icon=folium.Icon(color="green", icon="location-dot")
        ).add_to(m)

        # 绘制已部署障碍物（安全读取字段，避免KeyError）
         e.now().strftime("%H:%M:%S.%f")[:-3]
        st.session_state.last_heartbeat_time = new_time
        st.session_state.heartbeat_history.append({
            "序列号": new_seq,
            "接收时间": new_time,
            "状态": "正常"
        })

    st.subheader("📋 心跳数据记录表")
    if st.session_state.heartbeat_history:
        df_hb = pd.DataFrame(st.session_state.heartbeat_history)
        st.dataframe(df_hb, use_container_width=True, hide_index=True)
    else:
        st.info("📊 暂无心跳数据，请点击「开始模拟心跳」")

    st.divider()

    st.subheader("📈 心跳序列时间轨迹图")
    if len(st.session_state.heartbeat_history) > 0:
        seq_list = [item["序列号"] for item in st.session_state.heartbeat_history]
        time_list = [datetime.strptime(item["接收时间"], "%H:%M:%S.%f") for item in st.session_state.heartbeat_history]
        
        plt.rcParams["font.sans-serif"] = ["SimHei"]
        plt.rcParams["axes.unicode_minus"] = False
        
        fig, ax = plt.subplots(figsize=(12, 3))
        ax.plot(time_list, seq_list, color="#2ecc71", linewidth=2, marker='o', markersize=4)
        ax.set_title("无人机心跳序列时间轨迹", fontsize=12, fontweight="bold")
        ax.set_xlabel("接收时间", fontsize=10)
        ax.set_ylabel("心跳序列号", fontsize=10)
        ax.grid(True, alpha=0.3)
        fig.autofmt_xdate()
        st.pyplot(fig)
    else:
        st.info("📈 启动心跳模拟后自动生成时间线图")

# ====================== 页脚版权 ======================
st.markdown("---")
st.markdown("Leaflet | © 高德地图 | © OpenStreetMap contributors | 无人机航线规划系统 v18.0")
