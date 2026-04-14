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
    max_obstacle_height = max([obs.get("height", 0) for obs in obstacles]) if obstacles else 0
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
        for obs in st.session_state.deployed_obstacles:
            # 获取坐标（支持多种数据格式）
            coords = None
            if "coords" in obs:
                coords = obs["coords"]
            elif "geometry" in obs and "coordinates" in obs["geometry"]:
                coords = obs["geometry"]["coordinates"][0]
            
            if not coords:
                continue
            
            # 转换为folium需要的格式 [(lat, lng), ...]
            folium_coords = []
            for coord in coords:
                if len(coord) >= 2:
                    # 处理 [lng, lat] 格式
                    folium_coords.append([coord[1], coord[0]])
            
            # 安全获取属性
            height = obs.get("height", 0)
            name = obs.get("name", "未命名障碍物")
            create_time = obs.get("create_time", "未知")
            
            folium.Polygon(
                locations=folium_coords,
                color="red",
                weight=2,
                fill=True,
                fill_color="red",
                fill_opacity=0.3,
                popup=f"{name}\n高度: {height}m\n创建时间: {create_time}"
            ).add_to(m)
        
        # 绘制航线
        route_points = calculate_safe_path(
            st.session_state.start_point,
            st.session_state.end_point,
            st.session_state.deployed_obstacles,
            st.session_state.safety_radius,
            st.session_state.current_route_mode
        )
        
        # 添加航线
        folium.PolyLine(
            locations=route_points,
            color="blue",
            weight=4,
            opacity=0.8,
            popup=f"航线模式: {st.session_state.current_route_mode}"
        ).add_to(m)
        
        # 添加航点标记
        for i, point in enumerate(route_points):
            if i == 0:
                color = "red"
                icon_type = "play"
            elif i == len(route_points) - 1:
                color = "green"
                icon_type = "stop"
            else:
                color = "orange"
                icon_type = "info-sign"
            
            folium.Marker(
                location=point,
                popup=f"航点 {i}: ({point[0]:.6f}, {point[1]:.6f})",
                icon=folium.Icon(color=color, icon=icon_type, prefix='fa')
            ).add_to(m)
        
        # 显示地图
        output = st_folium(m, width=800, height=600, returned_objects=["all_drawings"])
        
        # 处理绘制的多边形（新增障碍物）
        if output and output.get("all_drawings"):
            for drawing in output["all_drawings"]:
                if drawing.get("geometry", {}).get("type") == "Polygon":
                    coords = drawing["geometry"]["coordinates"][0]
                    # 转换坐标格式
                    converted_coords = [[c[1], c[0]] for c in coords]
                    
                    # 显示添加障碍物的表单
                    with st.expander("✏️ 为新障碍物填写信息", expanded=True):
                        obs_name = st.text_input("障碍物名称", value=f"障碍物_{len(st.session_state.deployed_obstacles)+1}")
                        obs_height = st.number_input("障碍物高度 (米)", min_value=0.0, max_value=100.0, value=10.0)
                        
                        if st.button("✅ 确认添加障碍物"):
                            new_obstacle = {
                                "coords": converted_coords,
                                "name": obs_name,
                                "height": obs_height,
                                "create_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                            }
                            st.session_state.deployed_obstacles.append(new_obstacle)
                            st.session_state.obstacles.append(new_obstacle)
                            save_obstacles()
                            st.rerun()
    
    with col2:
        st.subheader("⚙️ 航线参数配置")
        
        # 起点设置
        with st.expander("📍 起点 (A点)", expanded=False):
            start_lat = st.number_input("纬度", value=st.session_state.start_point[0], format="%.6f", key="start_lat")
            start_lng = st.number_input("经度", value=st.session_state.start_point[1], format="%.6f", key="start_lng")
            if st.button("更新起点"):
                st.session_state.start_point = (start_lat, start_lng)
                save_obstacles()
                st.rerun()
        
        # 终点设置
        with st.expander("🏁 终点 (B点)", expanded=False):
            end_lat = st.number_input("纬度", value=st.session_state.end_point[0], format="%.6f", key="end_lat")
            end_lng = st.number_input("经度", value=st.session_state.end_point[1], format="%.6f", key="end_lng")
            if st.button("更新终点"):
                st.session_state.end_point = (end_lat, end_lng)
                save_obstacles()
                st.rerun()
        
        # 飞行参数
        st.session_state.flight_altitude = st.number_input(
            "✈️ 飞行高度 (米)", 
            min_value=0.0, 
            max_value=200.0, 
            value=st.session_state.flight_altitude,
            step=1.0,
            help="无人机飞行高度，高于所有障碍物时可直飞"
        )
        
        st.session_state.safety_radius = st.number_input(
            "🛡️ 安全半径 (米)", 
            min_value=1.0, 
            max_value=20.0, 
            value=st.session_state.safety_radius,
            step=0.5,
            help="障碍物周围的安全距离"
        )
        
        # 航线模式选择
        route_mode = st.radio(
            "🗺️ 航线模式",
            options=["best", "left", "right"],
            format_func=lambda x: {"best": "智能最优", "left": "左侧绕行", "right": "右侧绕行"}[x],
            horizontal=True
        )
        if route_mode != st.session_state.current_route_mode:
            st.session_state.current_route_mode = route_mode
            save_obstacles()
            st.rerun()
        
        st.divider()
        
        # 障碍物管理
        st.subheader("🏢 障碍物列表")
        if st.session_state.deployed_obstacles:
            for i, obs in enumerate(st.session_state.deployed_obstacles):
                col_a, col_b = st.columns([4, 1])
                with col_a:
                    st.write(f"**{obs.get('name', '未命名')}**")
                    st.write(f"📏 高度: {obs.get('height', 0)}m")
                    st.write(f"🕐 创建: {obs.get('create_time', '未知')}")
                with col_b:
                    if st.button("🗑️", key=f"del_{i}"):
                        st.session_state.deployed_obstacles.pop(i)
                        st.session_state.obstacles = st.session_state.deployed_obstacles.copy()
                        save_obstacles()
                        st.rerun()
                st.divider()
        else:
            st.info("暂无障碍物，请在地图上绘制多边形添加")
        
        if st.button("🗑️ 清空所有障碍物", use_container_width=True):
            st.session_state.deployed_obstacles = []
            st.session_state.obstacles = []
            save_obstacles()
            st.rerun()

# ====================== 标签页2：飞行监控 ======================
with tab2:
    col_mon1, col_mon2 = st.columns([1, 2])
    
    with col_mon1:
        st.subheader("🎮 飞行控制面板")
        
        # 心跳模拟控制
        if not st.session_state.heartbeat_running:
            if st.button("▶️ 开始模拟心跳", use_container_width=True, type="primary"):
                st.session_state.heartbeat_running = True
                st.rerun()
        else:
            if st.button("⏸️ 停止模拟心跳", use_container_width=True):
                st.session_state.heartbeat_running = False
                st.rerun()
        
        # 手动添加心跳
        if st.button("📡 手动发送心跳", use_container_width=True):
            new_seq = len(st.session_state.heartbeat_history) + 1
            new_time = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            st.session_state.last_heartbeat_time = new_time
            st.session_state.heartbeat_history.append({
                "序列号": new_seq,
                "接收时间": new_time,
                "状态": "正常"
            })
            st.rerun()
        
        # 清空历史记录
        if st.button("🗑️ 清空心跳记录", use_container_width=True):
            st.session_state.heartbeat_history = []
            st.session_state.heartbeat_running = False
            st.rerun()
        
        st.divider()
        
        # 飞行状态显示
        st.subheader("📊 当前飞行状态")
        st.metric("飞行高度", f"{st.session_state.flight_altitude} m")
        st.metric("安全半径", f"{st.session_state.safety_radius} m")
        st.metric("航线模式", {"best": "智能最优", "left": "左侧绕行", "right": "右侧绕行"}[st.session_state.current_route_mode])
        
        if st.session_state.deployed_obstacles:
            max_height = max([obs.get("height", 0) for obs in st.session_state.deployed_obstacles])
            st.metric("最大障碍物高度", f"{max_height} m")
            if st.session_state.flight_altitude > max_height:
                st.success("✅ 可直飞")
            else:
                st.warning("⚠️ 需要绕行")
    
    with col_mon2:
        st.subheader("📋 心跳数据记录表")
        if st.session_state.heartbeat_history:
            df_hb = pd.DataFrame(st.session_state.heartbeat_history)
            st.dataframe(df_hb, use_container_width=True, hide_index=True)
        else:
            st.info("📊 暂无心跳数据，请点击「开始模拟心跳」或「手动发送心跳」")
        
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

# ====================== 自动心跳模拟 ======================
if st.session_state.heartbeat_running:
    import time
    last_time = datetime.now()
    
    # 检查是否需要发送新心跳（每2秒一次）
    if (datetime.now() - last_time).total_seconds() >= 2:
        new_seq = len(st.session_state.heartbeat_history) + 1
        new_time = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        st.session_state.last_heartbeat_time = new_time
        st.session_state.heartbeat_history.append({
            "序列号": new_seq,
            "接收时间": new_time,
            "状态": "正常"
        })
        last_time = datetime.now()
        time.sleep(0.1)  # 避免过于频繁的更新
        st.rerun()

# ====================== 页脚版权 ======================
st.markdown("---")
st.markdown("Leaflet | © 高德地图 | © OpenStreetMap contributors | 无人机航线规划系统 v18.0")
