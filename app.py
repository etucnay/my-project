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
from shapely.geometry import Point, Polygon, LineString
import math

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
if "planned_route" not in st.session_state:
    st.session_state.planned_route = []

# ====================== 永久保存/加载逻辑 ======================
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
        "version": "v19.0_完整航线规划版"
    }
    with open(CONFIG_FILE, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)

def load_obstacles():
    if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
        loaded_obs = data.get("obstacles", [])
        for obs in loaded_obs:
            if "create_time" not in obs:
                obs["create_time"] = "未知"
        st.session_state.obstacles = loaded_obs
        
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

# ====================== 障碍物碰撞检测 ======================
def is_line_intersecting_polygon(point1, point2, polygon_coords):
    """检测线段是否与多边形相交"""
    line = LineString([point1, point2])
    polygon = Polygon(polygon_coords)
    return line.intersects(polygon)

def get_expanded_polygon(polygon_coords, expand_meters):
    """将多边形向外扩展指定米数（简化版，用于安全区域）"""
    # 简化处理：返回原多边形，实际应用中需要复杂的缓冲区计算
    return polygon_coords

# ====================== A*航线规划算法 ======================
def a_star_route_planning(start, end, obstacles, safety_radius, step_size=0.0001):
    """
    使用A*算法规划避开障碍物的航线
    简化版：如果直线穿过障碍物，则生成绕行点
    """
    start_lat, start_lng = start
    end_lat, end_lng = end
    
    # 检查直线是否安全
    line_safe = True
    for obs in obstacles:
        if "coords" in obs:
            polygon_coords = [[c[0], c[1]] for c in obs["coords"]]  # 确保格式正确
            if is_line_intersecting_polygon(start, end, polygon_coords):
                line_safe = False
                break
    
    if line_safe:
        return [start, end]
    
    # 需要绕行：生成绕行点
    # 计算方向向量
    dx = end_lng - start_lng
    dy = end_lat - start_lat
    dist = math.sqrt(dx**2 + dy**2)
    
    if dist < 1e-6:
        return [start, end]
    
    # 单位方向向量
    ux = dx / dist
    uy = dy / dist
    
    # 垂直向量
    nx = -uy
    ny = ux
    
    # 绕行距离（米转经纬度）
    lat_mid = (start_lat + end_lat) / 2
    lng_per_m = 1 / (111320 * math.cos(math.radians(lat_mid)))
    lat_per_m = 1 / 111320
    
    offset_m = safety_radius * 2  # 绕行距离为安全半径的2倍
    offset_lat = offset_m * lat_per_m
    offset_lng = offset_m * lng_per_m
    
    # 生成绕行点
    waypoints = []
    
    if st.session_state.current_route_mode == "left":
        # 左侧绕行
        mid1 = (start_lat + ny * offset_lat, start_lng + nx * offset_lng)
        mid2 = (end_lat + ny * offset_lat, end_lng + nx * offset_lng)
        waypoints = [start, mid1, mid2, end]
    elif st.session_state.current_route_mode == "right":
        # 右侧绕行
        mid1 = (start_lat - ny * offset_lat, start_lng - nx * offset_lng)
        mid2 = (end_lat - ny * offset_lat, end_lng - nx * offset_lng)
        waypoints = [start, mid1, mid2, end]
    else:
        # 智能最优：尝试左右两侧，选择较短路径
        left_mid1 = (start_lat + ny * offset_lat, start_lng + nx * offset_lng)
        left_mid2 = (end_lat + ny * offset_lat, end_lng + nx * offset_lng)
        right_mid1 = (start_lat - ny * offset_lat, start_lng - nx * offset_lng)
        right_mid2 = (end_lat - ny * offset_lat, end_lng - nx * offset_lng)
        
        # 计算左右路径长度
        left_dist = calculate_path_distance([start, left_mid1, left_mid2, end])
        right_dist = calculate_path_distance([start, right_mid1, right_mid2, end])
        
        if left_dist <= right_dist:
            waypoints = [start, left_mid1, left_mid2, end]
        else:
            waypoints = [start, right_mid1, right_mid2, end]
    
    return waypoints

def calculate_path_distance(waypoints):
    """计算路径总距离（米）"""
    total_dist = 0
    for i in range(len(waypoints) - 1):
        lat1, lng1 = waypoints[i]
        lat2, lng2 = waypoints[i + 1]
        # 使用Haversine公式计算距离
        R = 6371000  # 地球半径（米）
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lng2 - lng1)
        
        a = math.sin(delta_phi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        total_dist += R * c
    return total_dist

# ====================== 一键规划航线 ======================
def plan_route():
    """一键规划航线"""
    if not st.session_state.deployed_obstacles:
        st.session_state.planned_route = [st.session_state.start_point, st.session_state.end_point]
        return
    
    # 检查飞行高度是否足够
    max_obstacle_height = max([obs.get("height", 0) for obs in st.session_state.deployed_obstacles])
    
    if st.session_state.flight_altitude > max_obstacle_height:
        # 高度足够，直飞
        st.session_state.planned_route = [st.session_state.start_point, st.session_state.end_point]
        st.success(f"✅ 飞行高度 {st.session_state.flight_altitude}m 高于最高障碍物 {max_obstacle_height}m，可直飞")
    else:
        # 需要绕行
        st.session_state.planned_route = a_star_route_planning(
            st.session_state.start_point,
            st.session_state.end_point,
            st.session_state.deployed_obstacles,
            st.session_state.safety_radius
        )
        st.warning(f"⚠️ 飞行高度 {st.session_state.flight_altitude}m 低于障碍物高度 {max_obstacle_height}m，已生成绕行航线")
    
    save_obstacles()

# ====================== 分页设计 ======================
tab1, tab2 = st.tabs(["🗺️ 地图与航线规划", "📡 飞行监控"])

# ====================== 标签页1：地图与障碍物管理 ======================
with tab1:
    # 顶部按钮栏
    col_btn1, col_btn2, col_btn3, col_btn4 = st.columns([1, 1, 1, 1])
    with col_btn1:
        if st.button("🎯 一键规划航线", use_container_width=True, type="primary"):
            plan_route()
            st.rerun()
    with col_btn2:
        if st.button("🗺️ 刷新地图", use_container_width=True):
            st.rerun()
    with col_btn3:
        if st.button("📥 保存配置", use_container_width=True):
            save_obstacles()
            st.success("配置已保存")
    with col_btn4:
        if st.button("📤 重置所有", use_container_width=True):
            st.session_state.deployed_obstacles = []
            st.session_state.obstacles = []
            st.session_state.start_point = (32.2345, 118.7492)
            st.session_state.end_point = (32.2337, 118.7496)
            st.session_state.planned_route = []
            save_obstacles()
            st.rerun()
    
    st.divider()
    
    col1, col2 = st.columns([3, 1])
    with col1:
        st.subheader("🗺️ 卫星地图 (高德)")
        m = folium.Map(
            location=[32.2341, 118.7494],
            zoom_start=18,
            tiles="https://webst01.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}",
            attr="© 高德地图"
        )

        draw = Draw(
            draw_options={
                "polygon": {
                    "shapeOptions": {
                        "color": "#ff4444",
                        "fillColor": "#ff4444",
                        "fillOpacity": 0.3
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
            popup=f"🚁 起点 A\n坐标: {st.session_state.start_point[0]:.6f}, {st.session_state.start_point[1]:.6f}",
            icon=folium.Icon(color="red", icon="play", prefix="fa")
        ).add_to(m)

        folium.Marker(
            location=st.session_state.end_point,
            popup=f"🎯 终点 B\n坐标: {st.session_state.end_point[0]:.6f}, {st.session_state.end_point[1]:.6f}",
            icon=folium.Icon(color="green", icon="flag-checkered", prefix="fa")
        ).add_to(m)

        # 绘制已部署障碍物
        for idx, obs in enumerate(st.session_state.deployed_obstacles):
            coords = obs.get("coords", [])
            if not coords:
                continue
            
            # 转换为folium格式
            folium_coords = []
            for coord in coords:
                if len(coord) >= 2:
                    folium_coords.append([coord[1], coord[0]])
            
            height = obs.get("height", 0)
            name = obs.get("name", f"障碍物_{idx+1}")
            
            # 根据高度设置颜色
            if height > st.session_state.flight_altitude:
                color = "#ff0000"
                fill_opacity = 0.5
            else:
                color = "#ffaa00"
                fill_opacity = 0.2
            
            folium.Polygon(
                locations=folium_coords,
                color=color,
                weight=3,
                fill=True,
                fill_color=color,
                fill_opacity=fill_opacity,
                popup=f"🏢 {name}\n📏 高度: {height}m\n⚠️ 高于飞行高度" if height > st.session_state.flight_altitude else f"🏢 {name}\n📏 高度: {height}m\n✅ 低于飞行高度"
            ).add_to(m)
            
            # 添加中心点标记
            center_lat = sum([c[1] for c in folium_coords]) / len(folium_coords)
            center_lng = sum([c[0] for c in folium_coords]) / len(folium_coords)
            folium.CircleMarker(
                location=[center_lat, center_lng],
                radius=3,
                color=color,
                fill=True,
                popup=f"{name} 中心点"
            ).add_to(m)
        
        # 绘制规划好的航线
        if st.session_state.planned_route:
            # 绘制航线
            folium.PolyLine(
                locations=st.session_state.planned_route,
                color="#00ff00",
                weight=5,
                opacity=0.9,
                popup=f"📡 规划航线 | 模式: {st.session_state.current_route_mode} | 航点数: {len(st.session_state.planned_route)}"
            ).add_to(m)
            
            # 绘制航点
            for i, point in enumerate(st.session_state.planned_route):
                if i == 0:
                    color = "red"
                    icon_type = "play"
                elif i == len(st.session_state.planned_route) - 1:
                    color = "green"
                    icon_type = "flag-checkered"
                else:
                    color = "orange"
                    icon_type = "circle"
                
                folium.Marker(
                    location=point,
                    popup=f"航点 {i+1}\n坐标: {point[0]:.6f}, {point[1]:.6f}",
                    icon=folium.Icon(color=color, icon=icon_type, prefix="fa")
                ).add_to(m)
        
        # 显示地图
        output = st_folium(m, width=850, height=600, returned_objects=["all_drawings"])
        
        # 处理新绘制的障碍物
        if output and output.get("all_drawings"):
            for drawing in output["all_drawings"]:
                if drawing.get("geometry", {}).get("type") == "Polygon":
                    coords = drawing["geometry"]["coordinates"][0]
                    converted_coords = [[c[1], c[0]] for c in coords]
                    
                    # 检查是否已存在相同多边形
                    is_duplicate = False
                    for existing_obs in st.session_state.deployed_obstacles:
                        if existing_obs.get("coords") == converted_coords:
                            is_duplicate = True
                            break
                    
                    if not is_duplicate:
                        with st.expander("✏️ 新障碍物信息", expanded=True):
                            obs_name = st.text_input("名称", value=f"障碍物_{len(st.session_state.deployed_obstacles)+1}")
                            obs_height = st.number_input("高度(米)", min_value=0.0, max_value=100.0, value=10.0, step=1.0)
                            
                            col_a, col_b = st.columns(2)
                            with col_a:
                                if st.button("✅ 确认添加", use_container_width=True):
                                    new_obstacle = {
                                        "coords": converted_coords,
                                        "name": obs_name,
                                        "height": obs_height,
                                        "create_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                                    }
                                    st.session_state.deployed_obstacles.append(new_obstacle)
                                    st.session_state.obstacles.append(new_obstacle)
                                    save_obstacles()
                                    st.success(f"已添加障碍物: {obs_name}")
                                    st.rerun()
                            with col_b:
                                if st.button("❌ 取消", use_container_width=True):
                                    st.rerun()
    
    with col2:
        st.subheader("⚙️ 航线参数配置")
        
        # 起点设置
        with st.expander("📍 起点 (A点)", expanded=False):
            start_lat = st.number_input("纬度", value=st.session_state.start_point[0], format="%.6f", key="start_lat")
            start_lng = st.number_input("经度", value=st.session_state.start_point[1], format="%.6f", key="start_lng")
            if st.button("✈️ 更新起点", use_container_width=True):
                st.session_state.start_point = (start_lat, start_lng)
                save_obstacles()
                st.rerun()
        
        # 终点设置
        with st.expander("🏁 终点 (B点)", expanded=False):
            end_lat = st.number_input("纬度", value=st.session_state.end_point[0], format="%.6f", key="end_lat")
            end_lng = st.number_input("经度", value=st.session_state.end_point[1], format="%.6f", key="end_lng")
            if st.button("🎯 更新终点", use_container_width=True):
                st.session_state.end_point = (end_lat, end_lng)
                save_obstacles()
                st.rerun()
        
        st.divider()
        
        # 飞行参数
        new_altitude = st.number_input(
            "✈️ 飞行高度 (米)", 
            min_value=0.0, 
            max_value=200.0, 
            value=st.session_state.flight_altitude,
            step=1.0,
            help="无人机飞行高度，高于所有障碍物时可直飞"
        )
        if new_altitude != st.session_state.flight_altitude:
            st.session_state.flight_altitude = new_altitude
            save_obstacles()
        
        new_radius = st.number_input(
            "🛡️ 安全半径 (米)", 
            min_value=1.0, 
            max_value=50.0, 
            value=st.session_state.safety_radius,
            step=1.0,
            help="障碍物周围的安全距离"
        )
        if new_radius != st.session_state.safety_radius:
            st.session_state.safety_radius = new_radius
            save_obstacles()
        
        st.divider()
        
        # 航线模式
        route_mode = st.radio(
            "🗺️ 绕行策略",
            options=["best", "left", "right"],
            format_func=lambda x: {"best": "🤖 智能最优", "left": "⬅️ 左侧绕行", "right": "➡️ 右侧绕行"}[x],
            horizontal=True
        )
        if route_mode != st.session_state.current_route_mode:
            st.session_state.current_route_mode = route_mode
            save_obstacles()
            st.rerun()
        
        st.divider()
        
        # 障碍物统计
        st.subheader("📊 障碍物统计")
        st.metric("障碍物数量", len(st.session_state.deployed_obstacles))
        if st.session_state.deployed_obstacles:
            max_h = max([obs.get("height", 0) for obs in st.session_state.deployed_obstacles])
            st.metric("最高障碍物", f"{max_h} m")
            if st.session_state.flight_altitude > max_h:
                st.success("✅ 飞行高度安全")
            else:
                st.warning(f"⚠️ 需要提升至 {max_h + 5}m 或使用绕行")
        
        st.divider()
        
        # 障碍物列表
        st.subheader("🏢 障碍物列表")
        if st.session_state.deployed_obstacles:
            for i, obs in enumerate(st.session_state.deployed_obstacles):
                with st.container():
                    col_a, col_b = st.columns([4, 1])
                    with col_a:
                        st.write(f"**{obs.get('name', '未命名')}**")
                        st.write(f"📏 高度: {obs.get('height', 0)}m")
                        st.write(f"🕐 {obs.get('create_time', '未知')[:10]}")
                    with col_b:
                        if st.button("🗑️", key=f"del_{i}"):
                            st.session_state.deployed_obstacles.pop(i)
                            st.session_state.obstacles = st.session_state.deployed_obstacles.copy()
                            save_obstacles()
                            st.rerun()
                    st.divider()
        else:
            st.info("📭 暂无障碍物，请在地图上绘制多边形添加")
        
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
        
        if st.button("📡 手动发送心跳", use_container_width=True):
            new_seq = len(st.session_state.heartbeat_history) + 1
            new_time = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            st.session_state.heartbeat_history.append({
                "序列号": new_seq,
                "接收时间": new_time,
                "状态": "正常"
            })
            st.rerun()
        
        if st.button("🗑️ 清空心跳记录", use_container_width=True):
            st.session_state.heartbeat_history = []
            st.session_state.heartbeat_running = False
            st.rerun()
        
        st.divider()
        
        # 飞行状态
        st.subheader("📊 飞行状态")
        
        # 计算航线信息
        if st.session_state.planned_route:
            total_distance = calculate_path_distance(st.session_state.planned_route)
            st.metric("航线距离", f"{total_distance:.1f} m")
            st.metric("航点数量", len(st.session_state.planned_route))
        
        st.metric("飞行高度", f"{st.session_state.flight_altitude} m")
        st.metric("安全半径", f"{st.session_state.safety_radius} m")
        
        if st.session_state.deployed_obstacles:
            max_height = max([obs.get("height", 0) for obs in st.session_state.deployed_obstacles])
            if st.session_state.flight_altitude > max_height:
                st.success("✅ 可直飞")
            else:
                st.warning("⚠️ 需要绕行")
    
    with col_mon2:
        st.subheader("📋 心跳数据")
        if st.session_state.heartbeat_history:
            df_hb = pd.DataFrame(st.session_state.heartbeat_history)
            st.dataframe(df_hb, use_container_width=True, hide_index=True)
        else:
            st.info("暂无心跳数据")
        
        st.divider()
        
        st.subheader("📈 心跳轨迹图")
        if len(st.session_state.heartbeat_history) > 0:
            seq_list = [item["序列号"] for item in st.session_state.heartbeat_history]
            time_list = [datetime.strptime(item["接收时间"], "%H:%M:%S.%f") for item in st.session_state.heartbeat_history]
            
            plt.rcParams["font.sans-serif"] = ["SimHei"]
            plt.rcParams["axes.unicode_minus"] = False
            
            fig, ax = plt.subplots(figsize=(12, 3))
            ax.plot(time_list, seq_list, color="#2ecc71", linewidth=2, marker='o', markersize=4)
            ax.set_title("心跳序列时间轨迹", fontsize=12, fontweight="bold")
            ax.set_xlabel("时间", fontsize=10)
            ax.set_ylabel("序列号", fontsize=10)
            ax.grid(True, alpha=0.3)
            fig.autofmt_xdate()
            st.pyplot(fig)

# ====================== 自动心跳 ======================
if st.session_state.heartbeat_running:
    import time
    if len(st.session_state.heartbeat_history) > 0:
        last_time_str = st.session_state.heartbeat_history[-1]["接收时间"]
        last_time = datetime.strptime(last_time_str, "%H:%M:%S.%f")
        now = datetime.now()
        if (now - last_time).total_seconds() >= 2:
            new_seq = len(st.session_state.heartbeat_history) + 1
            new_time = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            st.session_state.heartbeat_history.append({
                "序列号": new_seq,
                "接收时间": new_time,
                "状态": "正常"
            })
            time.sleep(0.1)
            st.rerun()

# ====================== 页脚 ======================
st.markdown("---")
st.markdown("🚁 无人机航线规划系统 v19.0 | 支持障碍物检测与智能绕行")
