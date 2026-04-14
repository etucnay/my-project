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
from shapely.geometry import Polygon, Point
from shapely.affinity import translate

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
    st.session_state.obstacles = [] # 格式: [{"id":1, "coords": [...], "height": 10}, ...]
if "deployed_obstacles" not in st.session_state:
    st.session_state.deployed_obstacles = []
if "heartbeat_running" not in st.session_state:
    st.session_state.heartbeat_running = False
if "heartbeat_history" not in st.session_state:
    st.session_state.heartbeat_history = []
if "last_heartbeat_time" not in st.session_state:
    st.session_state.last_heartbeat_time = datetime.now().strftime("%H:%M:%S.%f")[:-3]
if "flight_altitude" not in st.session_state:
    st.session_state.flight_altitude = 10.0 # 默认飞行高度10米
if "safety_radius" not in st.session_state:
    st.session_state.safety_radius = 5.0  # 默认安全半径5米

# ====================== ✅ 永久保存/加载逻辑 ======================
def save_obstacles():
    """保存障碍物、部署状态、A/B点、飞行配置到本地JSON"""
    data = {
        "obstacles": st.session_state.obstacles,
        "deployed_obstacles": st.session_state.deployed_obstacles,
        "start_point": st.session_state.start_point,
        "end_point": st.session_state.end_point,
        "flight_altitude": st.session_state.flight_altitude,
        "safety_radius": st.session_state.safety_radius,
        "save_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "version": "v15.0_航线规划版"
    }
    with open(CONFIG_FILE, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)

def load_obstacles():
    """从本地JSON加载所有配置"""
    if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
        st.session_state.obstacles = data.get("obstacles", [])
        st.session_state.deployed_obstacles = data.get("deployed_obstacles", [])
        st.session_state.start_point = tuple(data.get("start_point", (32.2345, 118.7492)))
        st.session_state.end_point = tuple(data.get("end_point", (32.2337, 118.7496)))
        st.session_state.flight_altitude = data.get("flight_altitude", 10.0)
        st.session_state.safety_radius = data.get("safety_radius", 5.0)

# 启动时自动加载配置
load_obstacles()

# ====================== 航线规划核心算法 ======================
def calculate_safe_path(start, end, obstacles, safety_radius, mode="best"):
    """
    计算安全航线
    :param start: (lat, lng)
    :param end: (lat, lng)
    :param obstacles: list of {"coords": [(lat,lng),...], "height": int}
    :param safety_radius: 安全缓冲
    :param mode: left, right, best
    :return: list of points
    """
    start_point = Point(start[1], start[0])
    end_point = Point(end[1], end[0])
    
    # 1. 检查是否需要绕行
    max_obstacle_height = max([obs["height"] for obs in obstacles]) if obstacles else 0
    if st.session_state.flight_altitude > max_obstacle_height:
        # 高度足够，直接飞跃
        return [start, end]
    
    # 2. 需要绕行，构建障碍物多边形（带缓冲）
    obstacle_polygons = []
    for obs in obstacles:
        poly_coords = [(lng, lat) for lat, lng in obs["coords"]]
        poly = Polygon(poly_coords)
        buffered_poly = poly.buffer(safety_radius / 111320.0) # 度转米
        obstacle_polygons.append(buffered_poly)
    
    # 简单绕行算法示例：
    # 计算方向向量，根据模式选择左右偏移
    dx = end[1] - start[1]
    dy = end[0] - start[0]
    distance = np.sqrt(dx**2 + dy**2)
    
    # 单位法向量 (向左为正，向右为负)
    nx = -dy / distance
    ny = dx / distance
    
    path = [start]
    
    # 简化处理：如果只有一个障碍物或无阻碍，简单绕行
    # 实际项目建议使用A*算法结合RRT，这里为了演示提供基础逻辑
    if len(obstacle_polygons) == 0:
        path.append(end)
        return path
        
    # 强制绕行逻辑：生成绕过所有障碍的路径
    # 这里使用简单的“环绕”策略，实际应用可替换为更复杂算法
    # 计算绕行点
    offset = safety_radius * 1.2 / 111320.0 # 偏移量
    
    if mode == "left":
        # 向左绕行
        via_point1 = (start[0] + nx*offset, start[1] + ny*offset)
        via_point2 = (end[0] + nx*offset, end[1] + ny*offset)
        path = [start, via_point1, via_point2, end]
    elif mode == "right":
        # 向右绕行
        via_point1 = (start[0] - nx*offset, start[1] - ny*offset)
        via_point2 = (end[0] - nx*offset, end[1] - ny*offset)
        path = [start, via_point1, via_point2, end]
    else: # "best" 最佳航线
        # 简单策略：取平均偏移或计算最短路径，这里简化为结合左右最优
        # 实际逻辑可优化为：计算障碍两侧路径长度，选较短的一侧
        via_point1 = (start[0] + nx*offset*0.5, start[1] + ny*offset*0.5)
        via_point2 = (end[0] + nx*offset*0.5, end[1] + ny*offset*0.5)
        path = [start, via_point1, via_point2, end]
        
    return path

# ====================== 分页设计 ======================
tab1, tab2 = st.tabs(["🗺️ 地图与航线规划", "📡 飞行监控"])

# ====================== 标签页1：地图与障碍物管理（核心修复） ======================
with tab1:
    col1, col2 = st.columns([3, 1])

    # 左侧：卫星地图（修复地图空白+可圈选障碍物）
    with col1:
        st.subheader("🗺️ 卫星地图 (高德)")
        
        # 初始化地图
        m = folium.Map(
            location=[32.2341, 118.7494],
            zoom_start=18,
            tiles="https://webst01.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}",
            attr="© 高德地图"
        )

        # 多边形绘制工具
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
            popup="A点（起点）",
            icon=folium.Icon(color="red", icon="location-dot")
        ).add_to(m)

        folium.Marker(
            location=st.session_state.end_point,
            popup="B点（终点）",
            icon=folium.Icon(color="green", icon="location-dot")
        ).add_to(m)

        # 绘制已部署障碍物
        for obs in st.session_state.deployed_obstacles:
            folium.Polygon(
                locations=obs["coords"],
                color="red",
                weight=2,
                fill=True,
                fill_color="red",
                fill_opacity=0.5,
                popup=f"障碍物{obs['id']} (高度: {obs['height']}m)"
            ).add_to(m)

        # 航线规划逻辑
        A = st.session_state.start_point
        B = st.session_state.end_point
        
        # 根据当前模式绘制航线
        if "current_route_mode" not in st.session_state:
            st.session_state.current_route_mode = "best"
            
        route_points = calculate_safe_path(
            A, B, 
            st.session_state.deployed_obstacles, 
            st.session_state.safety_radius,
            st.session_state.current_route_mode
        )
        
        # 绘制航线
        if len(route_points) > 2:
            # 绕行航线：由多点组成
            folium.PolyLine(
                locations=route_points,
                color="orange",
                weight=5,
                opacity=0.8,
                dash_array="5, 10",
                popup=f"{st.session_state.current_route_mode}绕行航线"
            ).add_to(m)
        else:
            # 直接航线
            folium.PolyLine(
                locations=route_points,
                color="blue",
                weight=4,
                opacity=0.8,
                popup="A→B 直飞航线"
            ).add_to(m)

        # 渲染地图
        map_data = st_folium(m, width=800, height=650, key="fixed_map_key")

        # 监听绘制事件，添加障碍物（带高度设置）
        if map_data and map_data.get("last_active_drawing"):
            drawing = map_data["last_active_drawing"]
            if drawing["geometry"]["type"] == "Polygon":
                coords = [(lat, lng) for lng, lat in drawing["geometry"]["coordinates"][0]]
                # 弹出输入框设置高度
                with col2:
                    st.subheader("🏗️ 障碍物设置")
                    height = st.number_input("设置障碍物高度 (米)", min_value=1, max_value=1000, value=10, key="new_obs_height")
                    if st.button("✅ 确认添加此障碍物", use_container_width=True):
                        new_obs = {
                            "id": len(st.session_state.obstacles) + 1,
                            "coords": coords,
                            "height": height,
                            "create_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                        }
                        st.session_state.obstacles.append(new_obs)
                        save_obstacles()
                        st.success(f"障碍物 {new_obs['id']} 添加成功！")
                        # 清除绘图状态以防止重复添加
                        st.session_state.pop("last_active_drawing", None)
                        st.rerun()

    # 右侧：控制面板（A/B点、障碍物、航线配置）
    with col2:
        st.subheader("📍 A点/B点设置")
        a_lat = st.number_input("A纬度", value=st.session_state.start_point[0], format="%.6f", step=0.0001)
        a_lng = st.number_input("A经度", value=st.session_state.start_point[1], format="%.6f", step=0.0001)
        if st.button("✅ 确认A点", use_container_width=True):
            st.session_state.start_point = (a_lat, a_lng)
            save_obstacles()
            st.success("A点已保存")

        st.divider()

        b_lat = st.number_input("B纬度", value=st.session_state.end_point[0], format="%.6f", step=0.0001)
        b_lng = st.number_input("B经度", value=st.session_state.end_point[1], format="%.6f", step=0.0001)
        if st.button("✅ 确认B点", use_container_width=True):
            st.session_state.end_point = (b_lat, b_lng)
            save_obstacles()
            st.success("B点已保存")

        st.divider()
        
        # 🔴 新增：航线规划核心参数
        st.subheader("🛫 飞行参数配置")
        st.session_state.flight_altitude = st.number_input(
            "无人机飞行高度 (m)", 
            min_value=1.0, max_value=100.0, 
            value=st.session_state.flight_altitude, 
            step=0.5
        )
        st.session_state.safety_radius = st.number_input(
            "安全绕行半径 (m)", 
            min_value=1.0, max_value=50.0, 
            value=st.session_state.safety_radius, 
            step=0.5
        )
        st.caption("当飞行高度 > 障碍物最高高度 时将直飞")

        st.divider()
        
        # 🔴 新增：航线模式选择
        st.subheader("🧭 航线规划模式")
        mode = st.radio(
            "选择绕行策略",
            ("left", "right", "best"),
            format_func=lambda x: {"left": "🔴 向左绕行", "right": "🟢 向右绕行", "best": "🔵 最佳航线"}[x],
            horizontal=True
        )
        st.session_state.current_route_mode = mode
        
        if st.button("🔄 重新规划航线", use_container_width=True, type="primary"):
            save_obstacles() # 保存参数
            st.rerun() # 刷新重绘航线

        st.divider()

        # 🔴 障碍物管理
        st.subheader("🚀 障碍物配置")
        col_btn1, col_btn2, col_btn3, col_btn4 = st.columns(4)
        with col_btn1:
            if st.button("💾 永久保存", key="save_obs", use_container_width=True, type="primary"):
                save_obstacles()
                st.success(f"已保存 {len(st.session_state.obstacles)} 个障碍物！")
        with col_btn2:
            if st.button("📂 加载配置", key="load_obs
