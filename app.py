import streamlit as st
import folium
from streamlit_folium import st_folium, folium_static
from folium.plugins import Draw
import json
import os
from datetime import datetime
import pandas as pd
import math
import random
import time

# ====================== 页面配置 ======================
st.set_page_config(
    page_title="无人机航线规划与飞行监控系统",
    layout="wide",
    initial_sidebar_state="expanded"
)

st.title("🚁 无人机航线规划与飞行监控系统")

# ====================== 配置文件 ======================
CONFIG_FILE = "obstacle_config.json"

# ====================== 初始化 Session State ======================
if "start_point" not in st.session_state:
    st.session_state.start_point = (32.2345, 118.7492)
if "end_point" not in st.session_state:
    st.session_state.end_point = (32.2337, 118.7496)
if "obstacles" not in st.session_state:
    st.session_state.obstacles = []
if "flight_altitude" not in st.session_state:
    st.session_state.flight_altitude = 15.0
if "safety_radius" not in st.session_state:
    st.session_state.safety_radius = 15.0
if "current_route" not in st.session_state:
    st.session_state.current_route = []
if "map_center" not in st.session_state:
    st.session_state.map_center = [32.2341, 118.7494]
if "pending_polygon" not in st.session_state:
    st.session_state.pending_polygon = None
if "set_mode" not in st.session_state:
    st.session_state.set_mode = None
if "route_mode" not in st.session_state:
    st.session_state.route_mode = "best"
if "last_click" not in st.session_state:
    st.session_state.last_click = None

# ====================== 飞行监控状态 ======================
if "mission_active" not in st.session_state:
    st.session_state.mission_active = False
if "mission_paused" not in st.session_state:
    st.session_state.mission_paused = False
if "current_waypoint_index" not in st.session_state:
    st.session_state.current_waypoint_index = 0
if "mission_start_time" not in st.session_state:
    st.session_state.mission_start_time = None
if "flight_speed" not in st.session_state:
    st.session_state.flight_speed = 8.5
if "battery_level" not in st.session_state:
    st.session_state.battery_level = 100
if "flight_log" not in st.session_state:
    st.session_state.flight_log = []
if "current_position" not in st.session_state:
    st.session_state.current_position = None
if "telemetry_data" not in st.session_state:
    st.session_state.telemetry_data = []
if "is_flying" not in st.session_state:
    st.session_state.is_flying = False
if "last_auto_time" not in st.session_state:
    st.session_state.last_auto_time = 0
if "takeoff_time" not in st.session_state:
    st.session_state.takeoff_time = None

# ====================== 通信链路状态 ======================
if "gcs_status" not in st.session_state:
    st.session_state.gcs_status = "online"
if "obc_status" not in st.session_state:
    st.session_state.obc_status = "online"
if "fcu_status" not in st.session_state:
    st.session_state.fcu_status = "online"
if "link_quality" not in st.session_state:
    st.session_state.link_quality = 98
if "packet_loss" not in st.session_state:
    st.session_state.packet_loss = 0
if "signal_strength" not in st.session_state:
    st.session_state.signal_strength = -65
if "data_rate" not in st.session_state:
    st.session_state.data_rate = 120

# ====================== 心跳状态 ======================
if "heartbeat_history" not in st.session_state:
    st.session_state.heartbeat_history = []
if "heartbeat_running" not in st.session_state:
    st.session_state.heartbeat_running = False

# ====================== 保存/加载 ======================
def save_data():
    data = {
        "obstacles": st.session_state.obstacles,
        "start_point": st.session_state.start_point,
        "end_point": st.session_state.end_point,
        "flight_altitude": st.session_state.flight_altitude,
        "safety_radius": st.session_state.safety_radius,
        "route_mode": st.session_state.route_mode,
        "save_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    }
    with open(CONFIG_FILE, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)

def load_data():
    if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
        st.session_state.obstacles = data.get("obstacles", [])
        st.session_state.start_point = tuple(data.get("start_point", (32.2345, 118.7492)))
        st.session_state.end_point = tuple(data.get("end_point", (32.2337, 118.7496)))
        st.session_state.flight_altitude = data.get("flight_altitude", 15.0)
        st.session_state.safety_radius = data.get("safety_radius", 15.0)
        st.session_state.route_mode = data.get("route_mode", "best")

load_data()

# ====================== 几何计算函数 ======================
def calculate_distance(point1, point2):
    lat1, lng1 = point1
    lat2, lng2 = point2
    R = 6371000
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lng2 - lng1)
    a = math.sin(delta_phi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c

def point_in_polygon(point, polygon):
    x, y = point
    inside = False
    n = len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        if ((y1 > y) != (y2 > y)) and (x < (x2 - x1) * (y - y1) / (y2 - y1) + x1):
            inside = not inside
    return inside

def segments_intersect(p1, p2, p3, p4):
    def cross(o, a, b):
        return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])
    
    d1 = cross(p3, p4, p1)
    d2 = cross(p3, p4, p2)
    d3 = cross(p1, p2, p3)
    d4 = cross(p1, p2, p4)
    
    if ((d1 > 0 and d2 < 0) or (d1 < 0 and d2 > 0)) and ((d3 > 0 and d4 < 0) or (d3 < 0 and d4 > 0)):
        return True
    return False

def line_intersects_polygon(line_start, line_end, polygon):
    for i in range(len(polygon)):
        p1 = polygon[i]
        p2 = polygon[(i + 1) % len(polygon)]
        if segments_intersect(line_start, line_end, p1, p2):
            return True
    if point_in_polygon(line_start, polygon) or point_in_polygon(line_end, polygon):
        return True
    return False

def get_polygon_bounds(polygon):
    lats = [p[0] for p in polygon]
    lngs = [p[1] for p in polygon]
    return min(lats), max(lats), min(lngs), max(lngs)

def get_polygon_center(polygon):
    lats = [p[0] for p in polygon]
    lngs = [p[1] for p in polygon]
    return sum(lats)/len(lats), sum(lngs)/len(lngs)

def is_path_safe(start, end, obstacles, flight_altitude):
    for obs in obstacles:
        obs_height = obs.get("height", 0)
        if obs_height >= flight_altitude:
            polygon = obs.get("polygon", [])
            if polygon and line_intersects_polygon(start, end, polygon):
                return False
    return True

def get_obstacle_expanded_bounds(polygon, safety_radius):
    min_lat, max_lat, min_lng, max_lng = get_polygon_bounds(polygon)
    center_lat, center_lng = get_polygon_center(polygon)
    
    offset_meters = safety_radius
    offset_lat = offset_meters / 111320
    offset_lng = offset_meters / (111320 * math.cos(math.radians(center_lat)))
    
    return {
        'min_lat': min_lat - offset_lat,
        'max_lat': max_lat + offset_lat,
        'min_lng': min_lng - offset_lng,
        'max_lng': max_lng + offset_lng,
        'center_lat': center_lat,
        'center_lng': center_lng
    }

def find_path_between_obstacles(start, end, obstacles, flight_altitude, safety_radius):
    high_obstacles = [obs for obs in obstacles if obs.get("height", 0) >= flight_altitude]
    
    if not high_obstacles:
        return [start, end]
    
    if is_path_safe(start, end, high_obstacles, flight_altitude):
        return [start, end]
    
    expanded_bounds = []
    for obs in high_obstacles:
        bounds = get_obstacle_expanded_bounds(obs["polygon"], safety_radius)
        expanded_bounds.append(bounds)
    
    candidate_points = [start, end]
    
    for bounds in expanded_bounds:
        candidate_points.append((bounds['min_lat'], bounds['center_lng']))
        candidate_points.append((bounds['max_lat'], bounds['center_lng']))
        candidate_points.append((bounds['center_lat'], bounds['min_lng']))
        candidate_points.append((bounds['center_lat'], bounds['max_lng']))
    
    for i in range(len(expanded_bounds)):
        for j in range(i + 1, len(expanded_bounds)):
            b1 = expanded_bounds[i]
            b2 = expanded_bounds[j]
            
            if b1['center_lat'] < b2['center_lat']:
                gap_lat = (b1['max_lat'] + b2['min_lat']) / 2
                gap_lng = (b1['center_lng'] + b2['center_lng']) / 2
                candidate_points.append((gap_lat, gap_lng))
            
            if b1['center_lng'] < b2['center_lng']:
                gap_lng = (b1['max_lng'] + b2['min_lng']) / 2
                gap_lat = (b1['center_lat'] + b2['center_lat']) / 2
                candidate_points.append((gap_lat, gap_lng))
    
    unique_points = []
    for p in candidate_points:
        if p not in unique_points:
            unique_points.append(p)
    
    graph = {}
    for i, p1 in enumerate(unique_points):
        graph[i] = []
        for j, p2 in enumerate(unique_points):
            if i != j and is_path_safe(p1, p2, high_obstacles, flight_altitude):
                graph[i].append((j, calculate_distance(p1, p2)))
    
    start_idx = unique_points.index(start)
    end_idx = unique_points.index(end)
    
    distances = [float('inf')] * len(unique_points)
    distances[start_idx] = 0
    previous = [-1] * len(unique_points)
    visited = [False] * len(unique_points)
    
    for _ in range(len(unique_points)):
        min_dist = float('inf')
        min_idx = -1
        for i in range(len(unique_points)):
            if not visited[i] and distances[i] < min_dist:
                min_dist = distances[i]
                min_idx = i
        if min_idx == -1:
            break
        visited[min_idx] = True
        for neighbor, weight in graph[min_idx]:
            if not visited[neighbor]:
                new_dist = distances[min_idx] + weight
                if new_dist < distances[neighbor]:
                    distances[neighbor] = new_dist
                    previous[neighbor] = min_idx
    
    if distances[end_idx] == float('inf'):
        return [start, end]
    
    path_indices = []
    current = end_idx
    while current != -1:
        path_indices.insert(0, current)
        current = previous[current]
    
    path = [unique_points[i] for i in path_indices]
    
    simplified = [path[0]]
    i = 0
    while i < len(path) - 1:
        for j in range(len(path) - 1, i, -1):
            if j > i + 1 and is_path_safe(path[i], path[j], high_obstacles, flight_altitude):
                simplified.append(path[j])
                i = j
                break
        else:
            simplified.append(path[i + 1])
            i += 1
    
    return simplified

def plan_route():
    start = st.session_state.start_point
    end = st.session_state.end_point
    obstacles = st.session_state.obstacles
    altitude = st.session_state.flight_altitude
    safety_radius = st.session_state.safety_radius
    mode = st.session_state.route_mode
    
    if mode == "best":
        route = find_path_between_obstacles(start, end, obstacles, altitude, safety_radius)
    else:
        route = [start, end]
    
    st.session_state.current_route = route
    st.session_state.current_waypoint_index = 0
    st.session_state.current_position = start

# ====================== 飞行监控函数 ======================
def format_time(seconds):
    minutes = int(seconds // 60)
    secs = int(seconds % 60)
    return f"{minutes:02d}:{secs:02d}"

def get_elapsed_time():
    if st.session_state.mission_start_time:
        return (datetime.now() - st.session_state.mission_start_time).total_seconds()
    return 0

def get_estimated_arrival_time():
    if not st.session_state.current_route or st.session_state.current_waypoint_index >= len(st.session_state.current_route):
        return 0
    
    remaining_dist = 0
    for i in range(st.session_state.current_waypoint_index, len(st.session_state.current_route) - 1):
        remaining_dist += calculate_distance(st.session_state.current_route[i], st.session_state.current_route[i+1])
    
    return remaining_dist / st.session_state.flight_speed if st.session_state.flight_speed > 0 else 0

def calculate_total_distance(route):
    total = 0
    for i in range(len(route) - 1):
        total += calculate_distance(route[i], route[i+1])
    return total

def calculate_remaining_distance(route, current_index):
    if current_index >= len(route) - 1:
        return 0
    remaining = 0
    for i in range(current_index, len(route) - 1):
        remaining += calculate_distance(route[i], route[i+1])
    return remaining

def add_flight_log(action, details, level="info"):
    st.session_state.flight_log.insert(0, {
        "time": datetime.now().strftime("%H:%M:%S"),
        "action": action,
        "details": details,
        "level": level
    })
    if len(st.session_state.flight_log) > 50:
        st.session_state.flight_log = st.session_state.flight_log[:50]

def start_mission():
    if not st.session_state.current_route:
        st.toast("❌ 请先规划航线", icon="❌")
        return
    
    st.session_state.mission_active = True
    st.session_state.mission_paused = False
    st.session_state.current_waypoint_index = 0
    st.session_state.mission_start_time = datetime.now()
    st.session_state.takeoff_time = datetime.now()
    st.session_state.current_position = st.session_state.current_route[0]
    st.session_state.battery_level = 100
    st.session_state.is_flying = True
    st.session_state.last_auto_time = time.time()
    
    add_flight_log("任务开始", f"航线共 {len(st.session_state.current_route)-1} 个航段", "success")

def pause_mission():
    st.session_state.mission_paused = True
    st.session_state.is_flying = False
    add_flight_log("任务暂停", "", "warning")

def resume_mission():
    st.session_state.mission_paused = False
    st.session_state.is_flying = True
    st.session_state.last_auto_time = time.time()
    add_flight_log("任务恢复", "", "success")

def stop_mission():
    st.session_state.mission_active = False
    st.session_state.mission_paused = False
    st.session_state.is_flying = False
    st.session_state.current_waypoint_index = 0
    st.session_state.mission_start_time = None
    st.session_state.takeoff_time = None
    st.session_state.current_position = st.session_state.current_route[0] if st.session_state.current_route else None
    add_flight_log("任务停止", "", "error")

def reset_mission():
    stop_mission()
    st.session_state.current_waypoint_index = 0
    st.session_state.current_position = st.session_state.current_route[0] if st.session_state.current_route else None
    st.session_state.battery_level = 100
    add_flight_log("任务重置", "", "info")

def advance_to_next_waypoint():
    """前进到下一个航点"""
    if not st.session_state.current_route:
        return False
    
    if st.session_state.current_waypoint_index >= len(st.session_state.current_route) - 1:
        st.session_state.mission_active = False
        st.session_state.is_flying = False
        add_flight_log("任务完成", f"总飞行时间: {format_time(get_elapsed_time())}", "success")
        return False
    
    # 前进到下一个航点
    st.session_state.current_waypoint_index += 1
    st.session_state.current_position = st.session_state.current_route[st.session_state.current_waypoint_index]
    
    # 模拟电量消耗
    st.session_state.battery_level = max(0, st.session_state.battery_level - random.uniform(0.5, 1.0))
    
    # 记录航点到达
    add_flight_log("航点到达", f"航点 {st.session_state.current_waypoint_index}/{len(st.session_state.current_route)-1}", "info")
    
    # 更新链路质量
    st.session_state.link_quality = max(70, min(99, st.session_state.link_quality + random.uniform(-2, 2)))
    st.session_state.packet_loss = max(0, min(5, st.session_state.packet_loss + random.uniform(-0.3, 0.3)))
    st.session_state.signal_strength = max(-85, min(-55, st.session_state.signal_strength + random.uniform(-1, 1)))
    st.session_state.data_rate = random.randint(80, 150)
    
    # 添加遥测数据
    st.session_state.telemetry_data.insert(0, {
        "time": datetime.now().strftime("%H:%M:%S"),
        "waypoint": st.session_state.current_waypoint_index,
        "lat": f"{st.session_state.current_position[0]:.6f}",
        "lng": f"{st.session_state.current_position[1]:.6f}",
        "altitude": st.session_state.flight_altitude,
        "speed": st.session_state.flight_speed,
        "battery": f"{st.session_state.battery_level:.1f}%"
    })
    if len(st.session_state.telemetry_data) > 20:
        st.session_state.telemetry_data = st.session_state.telemetry_data[:20]
    
    # 检查是否完成
    if st.session_state.current_waypoint_index >= len(st.session_state.current_route) - 1:
        st.session_state.mission_active = False
        st.session_state.is_flying = False
        add_flight_log("任务完成", f"总飞行时间: {format_time(get_elapsed_time())}", "success")
    
    return True

# ====================== 创建地图 ======================
@st.cache_resource
def create_base_map():
    """创建基础地图（缓存）"""
    m = folium.Map(
        location=st.session_state.map_center,
        zoom_start=18,
        tiles="https://webst01.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}",
        attr="高德地图"
    )
    return m

def update_map_with_data(m):
    """在地图上添加数据（不重建整个地图）"""
    # 清除旧的数据层（保留基础地图）
    # 由于folium没有直接的清除方法，我们返回新地图
    return m

# ====================== 创建地图（无闪烁版本） ======================
def create_map(show_flight=True):
    m = folium.Map(
        location=st.session_state.map_center,
        zoom_start=18,
        tiles="https://webst01.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}",
        attr="高德地图"
    )
    
    # 绘图工具（仅当未飞行时显示）
    if not st.session_state.mission_active:
        draw = Draw(
            draw_options={
                "polygon": {
                    "allowIntersection": False,
                    "shapeOptions": {"color": "#ff4444", "fillColor": "#ff4444", "fillOpacity": 0.3}
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
    
    # 起点
    folium.Marker(
        location=st.session_state.start_point,
        popup="🚁 起点",
        icon=folium.Icon(color="red", icon="play", prefix="fa")
    ).add_to(m)
    
    # 终点
    folium.Marker(
        location=st.session_state.end_point,
        popup="🎯 终点",
        icon=folium.Icon(color="green", icon="flag-checkered", prefix="fa")
    ).add_to(m)
    
    # 障碍物
    for i, obs in enumerate(st.session_state.obstacles):
        polygon = obs.get("polygon", [])
        height = obs.get("height", 10)
        name = obs.get("name", f"障碍物{i+1}")
        
        if polygon:
            if height >= st.session_state.flight_altitude:
                color = "#ff0000"
                fill_opacity = 0.4
            else:
                color = "#00aa00"
                fill_opacity = 0.2
            
            folium.Polygon(
                locations=polygon,
                color=color,
                weight=2,
                fill=True,
                fill_color=color,
                fill_opacity=fill_opacity,
                popup=f"{name}\n高度: {height}m"
            ).add_to(m)
    
    # 航线
    if st.session_state.current_route:
        # 完整航线（虚线）
        folium.PolyLine(
            locations=st.session_state.current_route,
            color="#00ff00",
            weight=3,
            opacity=0.6,
            dash_array='5, 5'
        ).add_to(m)
        
        # 已完成航段（实线）
        if show_flight and st.session_state.current_waypoint_index > 0:
            completed = st.session_state.current_route[:st.session_state.current_waypoint_index + 1]
            if len(completed) > 1:
                folium.PolyLine(
                    locations=completed,
                    color="#00ff00",
                    weight=5,
                    opacity=0.9
                ).add_to(m)
        
        # 航点标记
        for i, point in enumerate(st.session_state.current_route):
            if i == 0:
                color = "red"
            elif i == len(st.session_state.current_route) - 1:
                color = "green"
            else:
                color = "orange"
            
            if show_flight and i <= st.session_state.current_waypoint_index:
                color = "lightgreen"
            
            folium.CircleMarker(
                location=point,
                radius=6,
                color=color,
                fill=True,
                fill_opacity=0.8,
                popup=f"航点 {i+1}"
            ).add_to(m)
    
    # 无人机当前位置
    if show_flight and st.session_state.current_position:
        folium.Marker(
            location=st.session_state.current_position,
            popup="✈️ 无人机当前位置",
            icon=folium.Icon(color="blue", icon="plane", prefix="fa")
        ).add_to(m)
        
        # 添加飞行方向指示圆圈
        folium.Circle(
            location=st.session_state.current_position,
            radius=15,
            color="blue",
            weight=2,
            fill=True,
            fill_opacity=0.3
        ).add_to(m)
        
        # 飞行方向线
        if st.session_state.current_waypoint_index < len(st.session_state.current_route) - 1:
            next_point = st.session_state.current_route[st.session_state.current_waypoint_index + 1]
            folium.PolyLine(
                locations=[st.session_state.current_position, next_point],
                color="#00aaff",
                weight=3,
                opacity=0.7,
                dash_array='5, 5'
            ).add_to(m)
    
    return m

# ====================== 页面布局 ======================
tab1, tab2, tab3 = st.tabs(["🗺️ 地图与航线规划", "📡 飞行任务监控", "📊 飞行日志与遥测"])

# ====================== 标签页1 ======================
with tab1:
    col_btn1, col_btn2, col_btn3, col_btn4, col_btn5 = st.columns(5)
    with col_btn1:
        if st.button("🎯 规划航线", use_container_width=True, type="primary"):
            plan_route()
            save_data()
            st.success("航线规划完成！")
    with col_btn2:
        if st.button("💾 保存数据", use_container_width=True):
            save_data()
            st.success("已保存")
    with col_btn3:
        if st.button("🗑️ 清空障碍物", use_container_width=True):
            st.session_state.obstacles = []
            st.session_state.current_route = []
            st.session_state.current_waypoint_index = 0
            save_data()
            st.success("已清空")
    with col_btn4:
        if st.button("🗺️ 重置视图", use_container_width=True):
            st.session_state.map_center = [32.2341, 118.7494]
            st.session_state.start_point = (32.2345, 118.7492)
            st.session_state.end_point = (32.2337, 118.7496)
            st.session_state.obstacles = []
            st.session_state.current_route = []
            st.session_state.current_waypoint_index = 0
            save_data()
            st.success("已重置")
    with col_btn5:
        if st.button("❌ 取消模式", use_container_width=True):
            st.session_state.set_mode = None
            st.success("已退出坐标设置模式")
    
    st.divider()
    
    if st.session_state.set_mode == 'start':
        st.info("🔴 当前模式：设置起点 - 请点击地图上的位置")
    elif st.session_state.set_mode == 'end':
        st.info("🟢 当前模式：设置终点 - 请点击地图上的位置")
    
    col1, col2 = st.columns([3, 1])
    
    with col1:
        st.subheader("🗺️ 地图")
        st.caption("💡 提示：使用绘图工具绘制多边形障碍物 | 点击下方按钮设置起点/终点")
        
        m = create_map(show_flight=False)
        output = st_folium(m, width=850, height=550, returned_objects=["last_clicked", "all_drawings"], key="planning_map")
        
        if output and output.get("last_clicked"):
            clicked = output["last_clicked"]
            if clicked and "lat" in clicked:
                current_click = (clicked["lat"], clicked["lng"])
                if current_click != st.session_state.last_click:
                    st.session_state.last_click = current_click
                    lat, lng = current_click
                    
                    if st.session_state.set_mode == 'start':
                        st.session_state.start_point = (lat, lng)
                        st.session_state.current_route = []
                        st.session_state.set_mode = None
                        save_data()
                        st.toast("✅ 起点已设置", icon="✅")
                        st.rerun()
                    elif st.session_state.set_mode == 'end':
                        st.session_state.end_point = (lat, lng)
                        st.session_state.current_route = []
                        st.session_state.set_mode = None
                        save_data()
                        st.toast("✅ 终点已设置", icon="✅")
                        st.rerun()
        
        if output and output.get("all_drawings"):
            drawings = output["all_drawings"]
            if len(drawings) > 0:
                last_drawing = drawings[-1]
                if last_drawing.get("geometry", {}).get("type") == "Polygon":
                    coords = last_drawing["geometry"]["coordinates"][0]
                    polygon_points = [[c[1], c[0]] for c in coords]
                    if st.session_state.pending_polygon != polygon_points:
                        st.session_state.pending_polygon = polygon_points
        
        if st.session_state.pending_polygon:
            with st.container():
                st.markdown("### ➕ 添加新障碍物")
                st.info(f"多边形顶点数: {len(st.session_state.pending_polygon)}")
                
                col_form1, col_form2 = st.columns(2)
                with col_form1:
                    obs_name = st.text_input("障碍物名称", value=f"障碍物_{len(st.session_state.obstacles)+1}", key="obs_name")
                with col_form2:
                    obs_height = st.number_input("高度（米）", min_value=0, max_value=100, value=20, step=1, key="obs_height")
                
                col_btn_a, col_btn_b = st.columns(2)
                with col_btn_a:
                    if st.button("✅ 确认添加", use_container_width=True, key="confirm_add"):
                        new_obs = {
                            "name": obs_name,
                            "height": obs_height,
                            "polygon": st.session_state.pending_polygon,
                            "create_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                        }
                        st.session_state.obstacles.append(new_obs)
                        st.session_state.pending_polygon = None
                        st.session_state.current_route = []
                        save_data()
                        st.success(f"✅ 已添加障碍物: {obs_name}")
                        st.rerun()
                with col_btn_b:
                    if st.button("❌ 取消", use_container_width=True, key="cancel_add"):
                        st.session_state.pending_polygon = None
                        st.rerun()
    
    with col2:
        st.subheader("⚙️ 参数设置")
        
        st.markdown("### 🗺️ 绕行模式")
        
        selected_mode = st.radio(
            "选择绕行策略",
            options=["best", "left", "right"],
            format_func=lambda x: {"best": "🌟 智能穿行", "left": "⬅️ 强制向左", "right": "➡️ 强制向右"}[x],
            index=["best", "left", "right"].index(st.session_state.route_mode),
            key="mode_select"
        )
        if selected_mode != st.session_state.route_mode:
            st.session_state.route_mode = selected_mode
            st.session_state.current_route = []
            save_data()
        
        st.divider()
        
        st.markdown("### 🎯 坐标设置")
        
        col_set1, col_set2 = st.columns(2)
        with col_set1:
            if st.button("📍 设置起点", use_container_width=True, key="set_start"):
                st.session_state.set_mode = 'start'
                st.toast("🔴 请点击地图设置起点", icon="🔴")
        with col_set2:
            if st.button("🏁 设置终点", use_container_width=True, key="set_end"):
                st.session_state.set_mode = 'end'
                st.toast("🟢 请点击地图设置终点", icon="🟢")
        
        st.divider()
        
        with st.expander("📍 起点手动输入", expanded=False):
            col_s1, col_s2 = st.columns(2)
            with col_s1:
                new_start_lat = st.number_input("纬度", value=st.session_state.start_point[0], format="%.6f", key="start_lat")
            with col_s2:
                new_start_lng = st.number_input("经度", value=st.session_state.start_point[1], format="%.6f", key="start_lng")
            
            if st.button("✈️ 更新起点", use_container_width=True, key="update_start"):
                st.session_state.start_point = (new_start_lat, new_start_lng)
                st.session_state.current_route = []
                save_data()
                st.success("起点已更新")
        
        with st.expander("🏁 终点手动输入", expanded=False):
            col_e1, col_e2 = st.columns(2)
            with col_e1:
                new_end_lat = st.number_input("纬度", value=st.session_state.end_point[0], format="%.6f", key="end_lat")
            with col_e2:
                new_end_lng = st.number_input("经度", value=st.session_state.end_point[1], format="%.6f", key="end_lng")
            
            if st.button("🎯 更新终点", use_container_width=True, key="update_end"):
                st.session_state.end_point = (new_end_lat, new_end_lng)
                st.session_state.current_route = []
                save_data()
                st.success("终点已更新")
        
        st.divider()
        
        new_altitude = st.number_input(
            "✈️ 飞行高度（米）",
            min_value=0.0, max_value=100.0,
            value=st.session_state.flight_altitude, step=1.0,
            key="altitude"
        )
        if new_altitude != st.session_state.flight_altitude:
            st.session_state.flight_altitude = new_altitude
            st.session_state.current_route = []
            save_data()
        
        new_radius = st.number_input(
            "🛡️ 安全半径（米）",
            min_value=5.0, max_value=30.0,
            value=st.session_state.safety_radius, step=2.0,
            key="radius"
        )
        if new_radius != st.session_state.safety_radius:
            st.session_state.safety_radius = new_radius
            st.session_state.current_route = []
            save_data()
        
        st.divider()
        
        st.subheader(f"📦 障碍物列表 ({len(st.session_state.obstacles)})")
        
        if st.session_state.obstacles:
            for i, obs in enumerate(st.session_state.obstacles):
                col_a, col_b = st.columns([4, 1])
                with col_a:
                    height = obs.get('height', 0)
                    name = obs.get('name', '未知')
                    if height >= st.session_state.flight_altitude:
                        st.markdown(f"**🔴 {name}** | {height}m (需绕行)")
                    else:
                        st.markdown(f"**🟢 {name}** | {height}m (可飞越)")
                with col_b:
                    if st.button("🗑️", key=f"del_{i}"):
                        st.session_state.obstacles.pop(i)
                        st.session_state.current_route = []
                        st.session_state.current_waypoint_index = 0
                        save_data()
                        st.rerun()
        else:
            st.info("📭 暂无障碍物，请在地图上绘制多边形")
        
        if st.session_state.current_route:
            st.divider()
            st.subheader("📊 航线信息")
            total_dist = calculate_total_distance(st.session_state.current_route)
            st.metric("总距离", f"{total_dist:.1f} m")
            st.metric("航点数", len(st.session_state.current_route))

# ====================== 标签页2 ======================
with tab2:
    st.subheader("🎮 飞行任务控制")
    
    col_ctl1, col_ctl2, col_ctl3, col_ctl4, col_ctl5 = st.columns(5)
    with col_ctl1:
        if not st.session_state.mission_active:
            if st.button("▶️ 开始任务", use_container_width=True, type="primary"):
                start_mission()
                st.rerun()
        else:
            st.button("✅ 飞行中", use_container_width=True, disabled=True)
    
    with col_ctl2:
        if st.session_state.mission_active and not st.session_state.mission_paused:
            if st.button("⏸️ 暂停", use_container_width=True):
                pause_mission()
                st.rerun()
        elif st.session_state.mission_paused:
            if st.button("▶️ 恢复", use_container_width=True):
                resume_mission()
                st.rerun()
        else:
            st.button("⏸️ 暂停", use_container_width=True, disabled=True)
    
    with col_ctl3:
        if st.button("⏹️ 停止", use_container_width=True):
            stop_mission()
            st.rerun()
    
    with col_ctl4:
        if st.button("🔄 重置", use_container_width=True):
            reset_mission()
            st.rerun()
    
    with col_ctl5:
        if st.session_state.mission_paused:
            st.button("⏸️ 已暂停", use_container_width=True, disabled=True)
        elif not st.session_state.mission_active:
            st.button("⏹️ 未开始", use_container_width=True, disabled=True)
        else:
            st.button("✅ 执行中", use_container_width=True, disabled=True)
    
    st.divider()
    
    # 飞行实时状态表格（仿照图片样式）
    st.subheader("📊 飞行实时状态")
    
    # 使用列布局显示状态数据
    col_stat1, col_stat2, col_stat3, col_stat4, col_stat5, col_stat6 = st.columns(6)
    
    with col_stat1:
        if st.session_state.current_route:
            current_wp = f"{st.session_state.current_waypoint_index}/{len(st.session_state.current_route)-1}"
        else:
            current_wp = "0/0"
        st.metric("航点", current_wp)
    
    with col_stat2:
        st.metric("飞行速度", f"{st.session_state.flight_speed} m/s")
    
    with col_stat3:
        st.metric("已用时间", format_time(get_elapsed_time()))
    
    with col_stat4:
        if st.session_state.current_route:
            remaining_dist = calculate_remaining_distance(st.session_state.current_route, st.session_state.current_waypoint_index)
        else:
            remaining_dist = 0
        st.metric("剩余距离", f"{remaining_dist:.0f} m")
    
    with col_stat5:
        st.metric("预计到达", format_time(get_estimated_arrival_time()))
    
    with col_stat6:
        battery = st.session_state.battery_level
        st.metric("电量模拟", f"{battery:.0f}%")
    
    st.divider()
    
    # 实时飞行地图
    st.subheader("🗺️ 实时飞行地图")
    flight_map = create_map(show_flight=True)
    st_folium(flight_map, width=900, height=450, returned_objects=[], key="monitor_map")
    
    st.divider()
    
    # 通信链路拓扑与数据流（仿照图片样式 - 简洁版）
    st.subheader("📡 通信链路拓扑与数据流")
    
    # 使用三列显示链路状态（像图片一样）
    col_link1, col_link2, col_link3 = st.columns(3)
    
    with col_link1:
        st.markdown("#### GCS")
        st.markdown("🟢 在线")
        st.caption("地面站")
    
    with col_link2:
        st.markdown("#### OBC")
        st.markdown("🟢 在线")
        st.caption("机载计算机")
    
    with col_link3:
        st.markdown("#### FCU")
        st.markdown("🟢 在线")
        st.caption("飞行控制单元")
    
    # 链路状态说明
    st.markdown("---")
    st.markdown("**数据流状态**: GCS ↔ OBC ↔ FCU")
    
    # 链路质量指示
    col_qual1, col_qual2, col_qual3 = st.columns(3)
    with col_qual1:
        st.metric("链路质量", f"{st.session_state.link_quality:.0f}%")
    with col_qual2:
        st.metric("丢包率", f"{st.session_state.packet_loss:.1f}%")
    with col_qual3:
        st.metric("信号强度", f"{st.session_state.signal_strength:.0f} dBm")
    
    st.divider()
    
    # 飞行进度条
    st.subheader("📈 任务进度")
    
    if st.session_state.current_route:
        total_wp = len(st.session_state.current_route) - 1
        current_wp = st.session_state.current_waypoint_index
        progress = current_wp / total_wp if total_wp > 0 else 0
        
        st.progress(progress, text=f"航点进度: {current_wp}/{total_wp}")
        
        if st.session_state.mission_active and not st.session_state.mission_paused:
            remaining_time = get_estimated_arrival_time()
            st.caption(f"预计剩余时间: {format_time(remaining_time)}")

# ====================== 标签页3 ======================
with tab3:
    col_log1, col_log2 = st.columns([1, 1])
    
    with col_log1:
        st.subheader("📝 飞行日志")
        
        if st.button("🗑️ 清空日志", use_container_width=True):
            st.session_state.flight_log = []
            st.rerun()
        
        if st.session_state.flight_log:
            for log in st.session_state.flight_log[:20]:
                if log.get('level') == 'success':
                    st.success(f"**[{log['time']}]** {log['action']} - {log['details']}")
                elif log.get('level') == 'error':
                    st.error(f"**[{log['time']}]** {log['action']} - {log['details']}")
                elif log.get('level') == 'warning':
                    st.warning(f"**[{log['time']}]** {log['action']} - {log['details']}")
                else:
                    st.info(f"**[{log['time']}]** {log['action']} - {log['details']}")
        else:
            st.info("暂无飞行日志")
    
    with col_log2:
        st.subheader("📊 遥测数据")
        
        if st.session_state.current_position:
            current_pos = f"({st.session_state.current_position[0]:.6f}, {st.session_state.current_position[1]:.6f})"
        else:
            current_pos = "未起飞"
        
        telemetry_table = {
            "参数": ["当前位置", "当前航点", "总航点数", "飞行速度", "飞行高度", "安全半径", "绕行模式", "电池电量", "已用时间", "剩余距离", "预计到达时间"],
            "数值": [
                current_pos,
                f"{st.session_state.current_waypoint_index}",
                f"{len(st.session_state.current_route)-1}",
                f"{st.session_state.flight_speed} m/s",
                f"{st.session_state.flight_altitude} m",
                f"{st.session_state.safety_radius} m",
                {"best": "智能穿行", "left": "强制向左", "right": "强制向右"}[st.session_state.route_mode],
                f"{st.session_state.battery_level:.1f}%",
                format_time(get_elapsed_time()),
                f"{calculate_remaining_distance(st.session_state.current_route, st.session_state.current_waypoint_index):.0f} m",
                format_time(get_estimated_arrival_time())
            ]
        }
        
        df = pd.DataFrame(telemetry_table)
        st.dataframe(df, use_container_width=True, hide_index=True)
        
        st.divider()
        
        # 心跳检测
        st.subheader("💓 心跳检测")
        
        col_heart1, col_heart2 = st.columns(2)
        with col_heart1:
            if not st.session_state.heartbeat_running:
                if st.button("▶️ 开始心跳", use_container_width=True):
                    st.session_state.heartbeat_running = True
                    st.rerun()
            else:
                if st.button("⏸️ 停止心跳", use_container_width=True):
                    st.session_state.heartbeat_running = False
                    st.rerun()
        
        with col_heart2:
            if st.button("📡 发送心跳", use_container_width=True):
                new_seq = len(st.session_state.heartbeat_history) + 1
                st.session_state.heartbeat_history.append({
                    "seq": new_seq,
                    "time": datetime.now().strftime("%H:%M:%S"),
                    "status": "正常"
                })
                st.rerun()
        
        if st.session_state.heartbeat_history:
            df_heart = pd.DataFrame(st.session_state.heartbeat_history[-10:])
            st.dataframe(df_heart, use_container_width=True, hide_index=True)
        else:
            st.info("暂无心跳数据")

# ====================== 自动心跳 ======================
if st.session_state.heartbeat_running:
    now = datetime.now()
    
    if len(st.session_state.heartbeat_history) == 0:
        st.session_state.heartbeat_history.append({
            "seq": 1,
            "time": now.strftime("%H:%M:%S"),
            "status": "正常"
        })
        time.sleep(0.5)
        st.rerun()
    else:
        last_time = datetime.strptime(st.session_state.heartbeat_history[-1]["time"], "%H:%M:%S")
        if (now - last_time).total_seconds() >= 2:
            st.session_state.heartbeat_history.append({
                "seq": len(st.session_state.heartbeat_history) + 1,
                "time": now.strftime("%H:%M:%S"),
                "status": "正常"
            })
            time.sleep(0.5)
            st.rerun()

# ====================== 自动飞行循环 ======================
if st.session_state.mission_active and not st.session_state.mission_paused:
    current_time = time.time()
    if current_time - st.session_state.last_auto_time >= 1.5:
        st.session_state.last_auto_time = current_time
        advance_to_next_waypoint()
        st.rerun()

# ====================== 页脚 ======================
st.markdown("---")
st.markdown("🚁 无人机航线规划与飞行监控系统 | 智能穿行模式自动寻找安全通道")
