import streamlit as st
import folium
from streamlit_folium import st_folium
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
if "last_update" not in st.session_state:
    st.session_state.last_update = 0

# ====================== 通信链路状态 ======================
if "gcs_status" not in st.session_state:
    st.session_state.gcs_status = "在线"
if "obc_status" not in st.session_state:
    st.session_state.obc_status = "在线"
if "fcu_status" not in st.session_state:
    st.session_state.fcu_status = "在线"

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

# ====================== 几何函数 ======================
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

# ====================== 路径规划 ======================
def is_path_safe(start, end, obstacles, flight_altitude):
    for obs in obstacles:
        if obs.get("height", 0) >= flight_altitude:
            polygon = obs.get("polygon", [])
            if polygon and line_intersects_polygon(start, end, polygon):
                return False
    return True

def plan_route_best(start, end, obstacles, flight_altitude, safety_radius):
    """智能穿行 - 尝试从障碍物之间穿过"""
    high_obstacles = [obs for obs in obstacles if obs.get("height", 0) >= flight_altitude]
    
    if not high_obstacles:
        return [start, end]
    
    if is_path_safe(start, end, high_obstacles, flight_altitude):
        return [start, end]
    
    # 尝试左侧和右侧，选择较短的
    left_route = plan_route_left(start, end, obstacles, flight_altitude, safety_radius)
    right_route = plan_route_right(start, end, obstacles, flight_altitude, safety_radius)
    
    left_dist = sum(calculate_distance(left_route[i], left_route[i+1]) for i in range(len(left_route)-1))
    right_dist = sum(calculate_distance(right_route[i], right_route[i+1]) for i in range(len(right_route)-1))
    
    return left_route if left_dist <= right_dist else right_route

def plan_route_left(start, end, obstacles, flight_altitude, safety_radius):
    """强制向左绕行 - 完全避开障碍物"""
    high_obstacles = [obs for obs in obstacles if obs.get("height", 0) >= flight_altitude]
    
    if not high_obstacles:
        return [start, end]
    
    if is_path_safe(start, end, high_obstacles, flight_altitude):
        return [start, end]
    
    waypoints = [start]
    current = start
    remaining = high_obstacles.copy()
    
    for _ in range(20):
        if is_path_safe(current, end, remaining, flight_altitude):
            waypoints.append(end)
            break
        
        # 找到阻挡的障碍物
        blocking = None
        for obs in remaining:
            if line_intersects_polygon(current, end, obs["polygon"]):
                blocking = obs
                break
        
        if blocking:
            bounds = get_polygon_bounds(blocking["polygon"])
            min_lat, max_lat, min_lng, max_lng = bounds
            center_lat = (min_lat + max_lat) / 2
            
            # 安全偏移 - 使用足够大的值
            offset_meters = safety_radius * 3
            offset_lng = offset_meters / (111320 * math.cos(math.radians(center_lat)))
            
            # 左侧绕行点（比最左边更左）
            left_point = (center_lat, min_lng - offset_lng)
            waypoints.append(left_point)
            current = left_point
            remaining.remove(blocking)
        else:
            waypoints.append(end)
            break
    
    return waypoints

def plan_route_right(start, end, obstacles, flight_altitude, safety_radius):
    """强制向右绕行 - 完全避开障碍物"""
    high_obstacles = [obs for obs in obstacles if obs.get("height", 0) >= flight_altitude]
    
    if not high_obstacles:
        return [start, end]
    
    if is_path_safe(start, end, high_obstacles, flight_altitude):
        return [start, end]
    
    waypoints = [start]
    current = start
    remaining = high_obstacles.copy()
    
    for _ in range(20):
        if is_path_safe(current, end, remaining, flight_altitude):
            waypoints.append(end)
            break
        
        # 找到阻挡的障碍物
        blocking = None
        for obs in remaining:
            if line_intersects_polygon(current, end, obs["polygon"]):
                blocking = obs
                break
        
        if blocking:
            bounds = get_polygon_bounds(blocking["polygon"])
            min_lat, max_lat, min_lng, max_lng = bounds
            center_lat = (min_lat + max_lat) / 2
            
            # 安全偏移 - 使用足够大的值
            offset_meters = safety_radius * 3
            offset_lng = offset_meters / (111320 * math.cos(math.radians(center_lat)))
            
            # 右侧绕行点（比最右边更右）
            right_point = (center_lat, max_lng + offset_lng)
            waypoints.append(right_point)
            current = right_point
            remaining.remove(blocking)
        else:
            waypoints.append(end)
            break
    
    return waypoints

def plan_route():
    start = st.session_state.start_point
    end = st.session_state.end_point
    obstacles = st.session_state.obstacles
    altitude = st.session_state.flight_altitude
    safety = st.session_state.safety_radius
    mode = st.session_state.route_mode
    
    if mode == "best":
        route = plan_route_best(start, end, obstacles, altitude, safety)
    elif mode == "left":
        route = plan_route_left(start, end, obstacles, altitude, safety)
    else:
        route = plan_route_right(start, end, obstacles, altitude, safety)
    
    st.session_state.current_route = route
    st.session_state.current_waypoint_index = 0
    st.session_state.current_position = start

# ====================== 飞行函数 ======================
def format_time(seconds):
    return f"{int(seconds//60):02d}:{int(seconds%60):02d}"

def get_elapsed():
    if st.session_state.mission_start_time:
        return (datetime.now() - st.session_state.mission_start_time).total_seconds()
    return 0

def get_eta():
    if not st.session_state.current_route or st.session_state.current_waypoint_index >= len(st.session_state.current_route):
        return 0
    remaining = 0
    for i in range(st.session_state.current_waypoint_index, len(st.session_state.current_route)-1):
        remaining += calculate_distance(st.session_state.current_route[i], st.session_state.current_route[i+1])
    return remaining / st.session_state.flight_speed if st.session_state.flight_speed > 0 else 0

def add_log(action, details, level="info"):
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
    st.session_state.current_position = st.session_state.current_route[0]
    st.session_state.battery_level = 100
    add_log("任务开始", f"航线共 {len(st.session_state.current_route)-1} 个航段", "success")

def pause_mission():
    st.session_state.mission_paused = True
    add_log("任务暂停", "", "warning")

def resume_mission():
    st.session_state.mission_paused = False
    add_log("任务恢复", "", "success")

def stop_mission():
    st.session_state.mission_active = False
    st.session_state.mission_paused = False
    st.session_state.current_waypoint_index = 0
    st.session_state.mission_start_time = None
    st.session_state.current_position = st.session_state.current_route[0] if st.session_state.current_route else None
    add_log("任务停止", "", "error")

def reset_mission():
    stop_mission()
    st.session_state.current_waypoint_index = 0
    st.session_state.current_position = st.session_state.current_route[0] if st.session_state.current_route else None
    st.session_state.battery_level = 100
    add_log("任务重置", "", "info")

# ====================== 自动飞行 ======================
now_time = time.time()
if st.session_state.mission_active and not st.session_state.mission_paused:
    if now_time - st.session_state.last_update >= 1.2:
        st.session_state.last_update = now_time
        if st.session_state.current_waypoint_index < len(st.session_state.current_route) - 1:
            st.session_state.current_waypoint_index += 1
            st.session_state.current_position = st.session_state.current_route[st.session_state.current_waypoint_index]
            st.session_state.battery_level = max(0, st.session_state.battery_level - 0.8)
            add_log("航点到达", f"航点 {st.session_state.current_waypoint_index}/{len(st.session_state.current_route)-1}", "info")
            if st.session_state.current_waypoint_index >= len(st.session_state.current_route) - 1:
                st.session_state.mission_active = False
                add_log("任务完成", f"总时间: {format_time(get_elapsed())}", "success")
            st.rerun()

# ====================== 创建地图 ======================
def create_map(show_flight=True):
    m = folium.Map(
        location=st.session_state.map_center,
        zoom_start=18,
        tiles="https://webst01.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}",
        attr="高德地图"
    )
    
    if not st.session_state.mission_active and not show_flight:
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
    
    folium.Marker(
        location=st.session_state.start_point,
        popup="🚁 起点",
        icon=folium.Icon(color="red", icon="play", prefix="fa")
    ).add_to(m)
    
    folium.Marker(
        location=st.session_state.end_point,
        popup="🎯 终点",
        icon=folium.Icon(color="green", icon="flag-checkered", prefix="fa")
    ).add_to(m)
    
    for obs in st.session_state.obstacles:
        polygon = obs.get("polygon", [])
        height = obs.get("height", 10)
        name = obs.get("name", "障碍物")
        if polygon:
            color = "#ff0000" if height >= st.session_state.flight_altitude else "#00aa00"
            fill_opacity = 0.4 if height >= st.session_state.flight_altitude else 0.2
            folium.Polygon(
                locations=polygon,
                color=color,
                weight=2,
                fill=True,
                fill_color=color,
                fill_opacity=fill_opacity,
                popup=f"{name}\n高度: {height}m"
            ).add_to(m)
    
    if st.session_state.current_route:
        folium.PolyLine(
            locations=st.session_state.current_route,
            color="#00ff00",
            weight=4,
            opacity=0.9
        ).add_to(m)
        
        if show_flight and st.session_state.current_waypoint_index > 0:
            completed = st.session_state.current_route[:st.session_state.current_waypoint_index + 1]
            if len(completed) > 1:
                folium.PolyLine(
                    locations=completed,
                    color="#00ff00",
                    weight=5,
                    opacity=0.9
                ).add_to(m)
        
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
                radius=5,
                color=color,
                fill=True,
                fill_opacity=0.8,
                popup=f"航点 {i+1}"
            ).add_to(m)
    
    if show_flight and st.session_state.current_position:
        folium.Marker(
            location=st.session_state.current_position,
            popup="✈️ 无人机当前位置",
            icon=folium.Icon(color="blue", icon="plane", prefix="fa")
        ).add_to(m)
        folium.Circle(
            location=st.session_state.current_position,
            radius=15,
            color="blue",
            weight=2,
            fill=True,
            fill_opacity=0.3
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
            save_data()
            st.success("已清空")
    with col_btn4:
        if st.button("🗺️ 重置视图", use_container_width=True):
            st.session_state.map_center = [32.2341, 118.7494]
            st.session_state.start_point = (32.2345, 118.7492)
            st.session_state.end_point = (32.2337, 118.7496)
            st.session_state.obstacles = []
            st.session_state.current_route = []
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
        m = create_map(show_flight=False)
        output = st_folium(m, width=850, height=550, returned_objects=["last_clicked", "all_drawings"], key="planning_map")
        
        if output and output.get("last_clicked"):
            clicked = output["last_clicked"]
            if clicked and "lat" in clicked:
                lat, lng = clicked["lat"], clicked["lng"]
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
                last = drawings[-1]
                if last.get("geometry", {}).get("type") == "Polygon":
                    coords = last["geometry"]["coordinates"][0]
                    polygon_points = [[c[1], c[0]] for c in coords]
                    st.session_state.pending_polygon = polygon_points
        
        if st.session_state.pending_polygon:
            with st.container():
                st.markdown("### ➕ 添加新障碍物")
                obs_name = st.text_input("名称", value=f"障碍物_{len(st.session_state.obstacles)+1}")
                obs_height = st.number_input("高度（米）", min_value=0, max_value=100, value=20, step=1)
                if st.button("✅ 确认添加", use_container_width=True):
                    st.session_state.obstacles.append({
                        "name": obs_name,
                        "height": obs_height,
                        "polygon": st.session_state.pending_polygon
                    })
                    st.session_state.pending_polygon = None
                    st.session_state.current_route = []
                    save_data()
                    st.rerun()
    
    with col2:
        st.subheader("⚙️ 参数设置")
        
        mode = st.radio(
            "绕行模式",
            options=["best", "left", "right"],
            format_func=lambda x: {"best": "🌟 智能穿行", "left": "⬅️ 强制向左", "right": "➡️ 强制向右"}[x],
            index=["best", "left", "right"].index(st.session_state.route_mode)
        )
        if mode != st.session_state.route_mode:
            st.session_state.route_mode = mode
            st.session_state.current_route = []
            save_data()
        
        st.divider()
        
        col_set1, col_set2 = st.columns(2)
        with col_set1:
            if st.button("📍 设置起点", use_container_width=True):
                st.session_state.set_mode = 'start'
        with col_set2:
            if st.button("🏁 设置终点", use_container_width=True):
                st.session_state.set_mode = 'end'
        
        st.divider()
        
        new_alt = st.number_input("✈️ 飞行高度（米）", value=st.session_state.flight_altitude, step=1.0)
        if new_alt != st.session_state.flight_altitude:
            st.session_state.flight_altitude = new_alt
            st.session_state.current_route = []
            save_data()
        
        new_safe = st.number_input("🛡️ 安全半径（米）", value=st.session_state.safety_radius, step=2.0)
        if new_safe != st.session_state.safety_radius:
            st.session_state.safety_radius = new_safe
            st.session_state.current_route = []
            save_data()
        
        st.divider()
        st.subheader(f"📦 障碍物 ({len(st.session_state.obstacles)})")
        
        for i, obs in enumerate(st.session_state.obstacles):
            col_a, col_b = st.columns([4, 1])
            with col_a:
                h = obs.get('height', 0)
                name = obs.get('name', '未知')
                status = "🔴 需绕行" if h >= st.session_state.flight_altitude else "🟢 可飞越"
                st.markdown(f"**{name}** | {h}m {status}")
            with col_b:
                if st.button("🗑️", key=f"del_{i}"):
                    st.session_state.obstacles.pop(i)
                    st.session_state.current_route = []
                    save_data()
                    st.rerun()
        
        if st.session_state.current_route:
            st.divider()
            total = sum(calculate_distance(st.session_state.current_route[i], st.session_state.current_route[i+1]) 
                       for i in range(len(st.session_state.current_route)-1))
            st.metric("总距离", f"{total:.1f} m")
            st.metric("航点数", len(st.session_state.current_route))

# ====================== 标签页2 ======================
with tab2:
    st.subheader("🎮 飞行任务控制")
    
    c1, c2, c3, c4, c5 = st.columns(5)
    with c1:
        if not st.session_state.mission_active:
            if st.button("▶️ 开始任务", use_container_width=True, type="primary"):
                start_mission()
                st.rerun()
        else:
            st.button("✈️ 飞行中", use_container_width=True, disabled=True)
    with c2:
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
    with c3:
        if st.button("⏹️ 停止", use_container_width=True):
            stop_mission()
            st.rerun()
    with c4:
        if st.button("🔄 重置", use_container_width=True):
            reset_mission()
            st.rerun()
    with c5:
        if st.session_state.mission_paused:
            st.button("⏸️ 已暂停", use_container_width=True, disabled=True)
        elif not st.session_state.mission_active:
            st.button("⏹️ 未开始", use_container_width=True, disabled=True)
        else:
            st.button("✅ 执行中", use_container_width=True, disabled=True)
    
    st.divider()
    
    # 实时状态
    st.subheader("📊 飞行实时状态")
    col_s1, col_s2, col_s3, col_s4, col_s5, col_s6 = st.columns(6)
    
    with col_s1:
        wp = f"{st.session_state.current_waypoint_index}/{len(st.session_state.current_route)-1}" if st.session_state.current_route else "0/0"
        st.metric("航点", wp)
    with col_s2:
        st.metric("飞行速度", f"{st.session_state.flight_speed} m/s")
    with col_s3:
        st.metric("已用时间", format_time(get_elapsed()))
    with col_s4:
        remaining = calculate_distance(st.session_state.current_position, st.session_state.end_point) if st.session_state.current_position else 0
        st.metric("剩余距离", f"{remaining:.0f} m")
    with col_s5:
        st.metric("预计到达", format_time(get_eta()))
    with col_s6:
        battery = st.session_state.battery_level
        color = "🟢" if battery > 50 else "🟡" if battery > 20 else "🔴"
        st.metric("电量", f"{color} {battery:.0f}%")
    
    st.divider()
    
    # 实时地图
    st.subheader("🗺️ 实时飞行地图")
    flight_map = create_map(show_flight=True)
    st_folium(flight_map, width=900, height=450, returned_objects=[], key="monitor_map")
    
    st.divider()
    
    # 通信链路
    st.subheader("📡 通信链路拓扑与数据流")
    col_l1, col_l2, col_l3 = st.columns(3)
    with col_l1:
        st.markdown("**GCS**  \n🟢 在线")
    with col_l2:
        st.markdown("**OBC**  \n🟢 在线")
    with col_l3:
        st.markdown("**FCU**  \n🟢 在线")
    st.caption("数据链路: GCS ↔ OBC ↔ FCU")
    
    st.divider()
    
    # 进度
    st.subheader("📈 任务进度")
    if st.session_state.current_route:
        total = len(st.session_state.current_route) - 1
        current = st.session_state.current_waypoint_index
        progress = current / total if total > 0 else 0
        st.progress(progress, text=f"航点进度: {current}/{total}")

# ====================== 标签页3 ======================
with tab3:
    col1, col2 = st.columns([1, 1])
    
    with col1:
        st.subheader("📝 飞行日志")
        if st.button("🗑️ 清空日志", use_container_width=True):
            st.session_state.flight_log = []
            st.rerun()
        for log in st.session_state.flight_log[:20]:
            if log['level'] == 'success':
                st.success(f"**[{log['time']}]** {log['action']} - {log['details']}")
            elif log['level'] == 'error':
                st.error(f"**[{log['time']}]** {log['action']} - {log['details']}")
            elif log['level'] == 'warning':
                st.warning(f"**[{log['time']}]** {log['action']} - {log['details']}")
            else:
                st.info(f"**[{log['time']}]** {log['action']} - {log['details']}")
    
    with col2:
        st.subheader("📊 遥测数据")
        pos = f"({st.session_state.current_position[0]:.6f}, {st.session_state.current_position[1]:.6f})" if st.session_state.current_position else "未起飞"
        data = {
            "参数": ["当前位置", "当前航点", "总航点数", "飞行速度", "飞行高度", "安全半径", "绕行模式", "电池电量", "已用时间"],
            "数值": [
                pos,
                f"{st.session_state.current_waypoint_index}",
                f"{len(st.session_state.current_route)-1}",
                f"{st.session_state.flight_speed} m/s",
                f"{st.session_state.flight_altitude} m",
                f"{st.session_state.safety_radius} m",
                {"best": "智能穿行", "left": "强制向左", "right": "强制向右"}[st.session_state.route_mode],
                f"{st.session_state.battery_level:.1f}%",
                format_time(get_elapsed())
            ]
        }
        df = pd.DataFrame(data)
        st.dataframe(df, use_container_width=True, hide_index=True)

# ====================== 页脚 ======================
st.markdown("---")
st.markdown("🚁 无人机航线规划与飞行监控系统 | 开始任务后自动每1.2秒飞行一个航点")
