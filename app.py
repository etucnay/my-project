import streamlit as st
import folium
from streamlit_folium import st_folium
from folium.plugins import Draw
import json
import os
from datetime import datetime
import pandas as pd
import matplotlib.pyplot as plt
import math
import numpy as np

# ====================== 页面配置 ======================
st.set_page_config(
    page_title="无人机航线规划系统",
    layout="wide",
    initial_sidebar_state="expanded"
)
st.title("🚁 无人机航线规划与障碍物管理系统")

# ====================== 配置文件 ======================
CONFIG_FILE = "obstacle_config.json"

# ====================== 初始化 ======================
if "start_point" not in st.session_state:
    st.session_state.start_point = (32.2345, 118.7492)
if "end_point" not in st.session_state:
    st.session_state.end_point = (32.2337, 118.7496)
if "obstacles" not in st.session_state:
    st.session_state.obstacles = []
if "heartbeat_history" not in st.session_state:
    st.session_state.heartbeat_history = []
if "heartbeat_running" not in st.session_state:
    st.session_state.heartbeat_running = False
if "flight_altitude" not in st.session_state:
    st.session_state.flight_altitude = 15.0
if "safety_radius" not in st.session_state:
    st.session_state.safety_radius = 10.0
if "current_route" not in st.session_state:
    st.session_state.current_route = []
if "map_center" not in st.session_state:
    st.session_state.map_center = [32.2341, 118.7494]
if "pending_polygon" not in st.session_state:
    st.session_state.pending_polygon = None
if "last_drawings" not in st.session_state:
    st.session_state.last_drawings = None
if "set_mode" not in st.session_state:
    st.session_state.set_mode = None
if "route_mode" not in st.session_state:
    st.session_state.route_mode = "best"

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
        st.session_state.safety_radius = data.get("safety_radius", 10.0)
        st.session_state.route_mode = data.get("route_mode", "best")

load_data()

# ====================== 几何计算函数 ======================
def point_in_polygon(point, polygon):
    """射线法判断点是否在多边形内"""
    if not polygon or len(polygon) < 3:
        return False
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
    """判断线段是否相交"""
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
    """判断线段是否与多边形相交"""
    if not polygon or len(polygon) < 3:
        return False
    for i in range(len(polygon)):
        p1 = polygon[i]
        p2 = polygon[(i + 1) % len(polygon)]
        if segments_intersect(line_start, line_end, p1, p2):
            return True
    if point_in_polygon(line_start, polygon) or point_in_polygon(line_end, polygon):
        return True
    return False

def calculate_distance(point1, point2):
    """计算两点间距离（米）"""
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

def get_bounding_box(polygon):
    """获取多边形边界框"""
    if not polygon:
        return 0, 0, 0, 0
    lats = [p[0] for p in polygon]
    lngs = [p[1] for p in polygon]
    return min(lats), max(lats), min(lngs), max(lngs)

def get_expanded_point(start, end, polygon, safety_radius, direction='left'):
    """获取绕过障碍物的偏移点"""
    if not polygon or len(polygon) < 3:
        return ((start[0] + end[0]) / 2, (start[1] + end[1]) / 2)
    
    center_lat = sum([p[0] for p in polygon]) / len(polygon)
    center_lng = sum([p[1] for p in polygon]) / len(polygon)
    
    min_lat, max_lat, min_lng, max_lng = get_bounding_box(polygon)
    poly_width_lat = (max_lat - min_lat) * 111320
    poly_width_lng = (max_lng - min_lng) * 111320 * math.cos(math.radians(center_lat))
    poly_size = max(poly_width_lat, poly_width_lng)
    
    offset_dist = max((poly_size / 2) + safety_radius, safety_radius + 5)
    
    lat_mid = (start[0] + end[0]) / 2
    if abs(lat_mid) > 85:
        lat_mid = 32.0
    lng_per_m = 1 / (111320 * math.cos(math.radians(lat_mid))) if abs(math.cos(math.radians(lat_mid))) > 0.01 else 0.00001
    lat_per_m = 1 / 111320
    
    dx = end[1] - start[1]
    dy = end[0] - start[0]
    dist = math.sqrt(dx**2 + dy**2)
    
    if dist < 1e-6:
        return (center_lat + offset_dist * lat_per_m, center_lng)
    
    ux = dx / dist
    uy = dy / dist
    
    if direction == 'left':
        nx = -uy
        ny = ux
    else:
        nx = uy
        ny = -ux
    
    offset_lat = offset_dist * lat_per_m
    offset_lng = offset_dist * lng_per_m
    
    return (center_lat + ny * offset_lat, center_lng + nx * offset_lng)

def find_obstacle_on_path(start, end, obstacles, flight_altitude):
    """找到路径上的第一个障碍物"""
    for obs in obstacles:
        obs_height = obs.get("height", 0)
        if isinstance(obs_height, (int, float)) and obs_height >= flight_altitude:
            polygon = obs.get("polygon", [])
            if polygon and len(polygon) >= 3:
                try:
                    if line_intersects_polygon(start, end, polygon):
                        return obs
                except Exception:
                    continue
    return None

def plan_route_left(start, end, obstacles, flight_altitude, safety_radius):
    """左侧绕行规划"""
    waypoints = [start]
    current_start = start
    remaining_obstacles = obstacles.copy()
    used_obstacles = set()
    
    max_iterations = 20
    for _ in range(max_iterations):
        obs = find_obstacle_on_path(current_start, end, remaining_obstacles, flight_altitude)
        if obs is None:
            waypoints.append(end)
            break
        
        obs_id = id(obs)
        if obs_id in used_obstacles:
            waypoints.append(end)
            break
        used_obstacles.add(obs_id)
        
        polygon = obs.get("polygon", [])
        if not polygon:
            waypoints.append(end)
            break
            
        bypass_point = get_expanded_point(current_start, end, polygon, safety_radius, 'left')
        waypoints.append(bypass_point)
        current_start = bypass_point
        remaining_obstacles = [o for o in remaining_obstacles if o != obs]
    
    if waypoints[-1] != end:
        waypoints.append(end)
    
    return waypoints

def plan_route_right(start, end, obstacles, flight_altitude, safety_radius):
    """右侧绕行规划"""
    waypoints = [start]
    current_start = start
    remaining_obstacles = obstacles.copy()
    used_obstacles = set()
    
    max_iterations = 20
    for _ in range(max_iterations):
        obs = find_obstacle_on_path(current_start, end, remaining_obstacles, flight_altitude)
        if obs is None:
            waypoints.append(end)
            break
        
        obs_id = id(obs)
        if obs_id in used_obstacles:
            waypoints.append(end)
            break
        used_obstacles.add(obs_id)
        
        polygon = obs.get("polygon", [])
        if not polygon:
            waypoints.append(end)
            break
            
        bypass_point = get_expanded_point(current_start, end, polygon, safety_radius, 'right')
        waypoints.append(bypass_point)
        current_start = bypass_point
        remaining_obstacles = [o for o in remaining_obstacles if o != obs]
    
    if waypoints[-1] != end:
        waypoints.append(end)
    
    return waypoints

def plan_route_best(start, end, obstacles, flight_altitude, safety_radius):
    """最佳航线 - 尝试左右并选择较短路径"""
    left_route = plan_route_left(start, end, obstacles, flight_altitude, safety_radius)
    right_route = plan_route_right(start, end, obstacles, flight_altitude, safety_radius)
    
    left_dist = 0
    for i in range(len(left_route) - 1):
        left_dist += calculate_distance(left_route[i], left_route[i+1])
    
    right_dist = 0
    for i in range(len(right_route) - 1):
        right_dist += calculate_distance(right_route[i], right_route[i+1])
    
    return left_route if left_dist <= right_dist else right_route

def plan_route():
    """规划航线主函数"""
    try:
        start = st.session_state.start_point
        end = st.session_state.end_point
        obstacles = st.session_state.obstacles
        altitude = st.session_state.flight_altitude
        safety_radius = st.session_state.safety_radius
        mode = st.session_state.route_mode
        
        if not start or not end or len(start) != 2 or len(end) != 2:
            st.session_state.current_route = []
            return
        
        high_obstacles = []
        for obs in obstacles:
            obs_height = obs.get("height", 0)
            if isinstance(obs_height, (int, float)) and obs_height >= altitude:
                polygon = obs.get("polygon", [])
                if polygon and len(polygon) >= 3:
                    high_obstacles.append(obs)
        
        if not high_obstacles:
            st.session_state.current_route = [start, end]
            return
        
        if find_obstacle_on_path(start, end, high_obstacles, altitude) is None:
            st.session_state.current_route = [start, end]
            return
        
        if mode == "left":
            route = plan_route_left(start, end, high_obstacles, altitude, safety_radius)
        elif mode == "right":
            route = plan_route_right(start, end, high_obstacles, altitude, safety_radius)
        else:
            route = plan_route_best(start, end, high_obstacles, altitude, safety_radius)
        
        if route and len(route) >= 2:
            st.session_state.current_route = route
        else:
            st.session_state.current_route = [start, end]
            
    except Exception as e:
        st.session_state.current_route = [st.session_state.start_point, st.session_state.end_point]

# ====================== 创建地图 ======================
def create_map():
    m = folium.Map(
        location=st.session_state.map_center,
        zoom_start=18,
        tiles="https://webst01.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}",
        attr="高德地图"
    )
    
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
        popup=f"🚁 起点 A\n{st.session_state.start_point[0]:.6f}, {st.session_state.start_point[1]:.6f}",
        icon=folium.Icon(color="red", icon="play", prefix="fa"),
        draggable=False
    ).add_to(m)
    
    folium.Marker(
        location=st.session_state.end_point,
        popup=f"🎯 终点 B\n{st.session_state.end_point[0]:.6f}, {st.session_state.end_point[1]:.6f}",
        icon=folium.Icon(color="green", icon="flag-checkered", prefix="fa"),
        draggable=False
    ).add_to(m)
    
    for i, obs in enumerate(st.session_state.obstacles):
        polygon = obs.get("polygon", [])
        height = obs.get("height", 10)
        name = obs.get("name", f"障碍物{i+1}")
        
        if polygon and len(polygon) >= 3:
            if height >= st.session_state.flight_altitude:
                color = "#ff0000"
                fill_opacity = 0.4
                status = "⚠️ 需要绕行"
            else:
                color = "#00aa00"
                fill_opacity = 0.2
                status = "✅ 可飞越"
            
            folium.Polygon(
                locations=polygon,
                color=color,
                weight=2,
                fill=True,
                fill_color=color,
                fill_opacity=fill_opacity,
                popup=f"📦 {name}\n📏 高度: {height}m\n{status}"
            ).add_to(m)
    
    if st.session_state.current_route and len(st.session_state.current_route) >= 2:
        if st.session_state.route_mode == "left":
            route_color = "#ff8800"
        elif st.session_state.route_mode == "right":
            route_color = "#aa66ff"
        else:
            route_color = "#00ff00"
        
        folium.PolyLine(
            locations=st.session_state.current_route,
            color=route_color,
            weight=5,
            opacity=0.8,
            popup=f"规划航线 | 模式: {st.session_state.route_mode} | {len(st.session_state.current_route)}个航点"
        ).add_to(m)
        
        for i, point in enumerate(st.session_state.current_route):
            if i == 0:
                color = "red"
            elif i == len(st.session_state.current_route) - 1:
                color = "green"
            else:
                color = "orange"
            
            folium.CircleMarker(
                location=point,
                radius=6,
                color=color,
                fill=True,
                fill_opacity=0.8,
                popup=f"航点 {i+1}\n({point[0]:.6f}, {point[1]:.6f})"
            ).add_to(m)
    
    return m

# ====================== 界面 ======================
tab1, tab2 = st.tabs(["🗺️ 地图与航线规划", "📡 飞行监控"])

# ====================== 标签页1 ======================
with tab1:
    col_btn1, col_btn2, col_btn3, col_btn4, col_btn5 = st.columns(5)
    with col_btn1:
        if st.button("🎯 规划航线", use_container_width=True, type="primary"):
            plan_route()
            save_data()
            mode_names = {"best": "最佳航线", "left": "向左绕行", "right": "向右绕行"}
            st.success(f"航线规划完成！模式: {mode_names[st.session_state.route_mode]}")
            st.rerun()
    with col_btn2:
        if st.button("💾 保存数据", use_container_width=True):
            save_data()
            st.success("已保存")
    with col_btn3:
        if st.button("🗑️ 清空障碍物", use_container_width=True):
            st.session_state.obstacles = []
            st.session_state.current_route = []
            save_data()
            st.rerun()
    with col_btn4:
        if st.button("🗺️ 重置视图", use_container_width=True):
            st.session_state.map_center = [32.2341, 118.7494]
            st.session_state.start_point = (32.2345, 118.7492)
            st.session_state.end_point = (32.2337, 118.7496)
            st.session_state.obstacles = []
            st.session_state.current_route = []
            save_data()
            st.rerun()
    with col_btn5:
        if st.button("❌ 取消模式", use_container_width=True):
            st.session_state.set_mode = None
            st.success("已退出坐标设置模式")
            st.rerun()
    
    st.divider()
    
    if st.session_state.set_mode == 'start':
        st.info("🔴 当前模式：设置起点 - 请点击地图上的位置")
    elif st.session_state.set_mode == 'end':
        st.info("🟢 当前模式：设置终点 - 请点击地图上的位置")
    
    col1, col2 = st.columns([3, 1])
    
    with col1:
        st.subheader("🗺️ 地图")
        st.caption("💡 提示：点击下方按钮进入坐标设置模式，然后点击地图设置起点/终点")
        
        m = create_map()
        output = st_folium(m, width=850, height=550, returned_objects=["last_clicked", "all_drawings"])
        
        if output and output.get("last_clicked"):
            clicked = output["last_clicked"]
            if clicked and "lat" in clicked and "lng" in clicked:
                lat, lng = clicked["lat"], clicked["lng"]
                
                if st.session_state.set_mode == 'start':
                    st.session_state.start_point = (lat, lng)
                    st.session_state.current_route = []
                    st.session_state.set_mode = None
                    save_data()
                    st.success(f"✅ 起点已设置为: ({lat:.6f}, {lng:.6f})")
                    st.rerun()
                elif st.session_state.set_mode == 'end':
                    st.session_state.end_point = (lat, lng)
                    st.session_state.current_route = []
                    st.session_state.set_mode = None
                    save_data()
                    st.success(f"✅ 终点已设置为: ({lat:.6f}, {lng:.6f})")
                    st.rerun()
        
        if output and output.get("all_drawings"):
            current_drawings = output["all_drawings"]
            
            if current_drawings != st.session_state.last_drawings:
                st.session_state.last_drawings = current_drawings
                
                for drawing in current_drawings:
                    if drawing.get("geometry", {}).get("type") == "Polygon":
                        coords = drawing["geometry"]["coordinates"][0]
                        polygon_points = [[c[1], c[0]] for c in coords]
                        st.session_state.pending_polygon = polygon_points
                        st.rerun()
        
        if st.session_state.pending_polygon:
            with st.container():
                st.markdown("### ➕ 添加新障碍物")
                st.info(f"多边形顶点数: {len(st.session_state.pending_polygon)}")
                
                col_form1, col_form2 = st.columns(2)
                with col_form1:
                    obs_name = st.text_input("障碍物名称", value=f"障碍物_{len(st.session_state.obstacles)+1}")
                with col_form2:
                    obs_height = st.number_input("高度（米）", min_value=0, max_value=100, value=10, step=1)
                
                col_btn_a, col_btn_b, col_btn_c = st.columns(3)
                with col_btn_a:
                    if st.button("✅ 确认添加", use_container_width=True, type="primary"):
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
                        st.success(f"✅ 已添加障碍物: {obs_name} (高度: {obs_height}m)")
                        st.rerun()
                with col_btn_b:
                    if st.button("❌ 取消", use_container_width=True):
                        st.session_state.pending_polygon = None
                        st.rerun()
                with col_btn_c:
                    if st.button("🗑️ 清除所有绘制", use_container_width=True):
                        st.session_state.pending_polygon = None
                        st.session_state.last_drawings = None
                        st.rerun()
    
    with col2:
        st.subheader("⚙️ 参数设置")
        
        st.markdown("### 🗺️ 绕行模式")
        
        mode_names = {
            "best": "🌟 最佳航线（智能选择最短路径）",
            "left": "⬅️ 向左绕行", 
            "right": "➡️ 向右绕行"
        }
        
        selected_mode = st.radio(
            "选择绕行策略",
            options=["best", "left", "right"],
            format_func=lambda x: mode_names[x],
            horizontal=False,
            index=["best", "left", "right"].index(st.session_state.route_mode)
        )
        if selected_mode != st.session_state.route_mode:
            st.session_state.route_mode = selected_mode
            st.session_state.current_route = []
            save_data()
            st.rerun()
        
        st.divider()
        
        st.markdown("### 🎯 坐标设置")
        st.caption("点击下方按钮后，再点击地图上的位置")
        
        col_set1, col_set2 = st.columns(2)
        with col_set1:
            if st.button("📍 设置起点", use_container_width=True, type="primary"):
                st.session_state.set_mode = 'start'
                st.rerun()
        with col_set2:
            if st.button("🏁 设置终点", use_container_width=True, type="primary"):
                st.session_state.set_mode = 'end'
                st.rerun()
        
        st.divider()
        
        with st.expander("📍 起点手动输入", expanded=False):
            col_s1, col_s2 = st.columns(2)
            with col_s1:
                new_start_lat = st.number_input("纬度", value=float(st.session_state.start_point[0]), format="%.6f")
            with col_s2:
                new_start_lng = st.number_input("经度", value=float(st.session_state.start_point[1]), format="%.6f")
            
            if st.button("✈️ 更新起点", use_container_width=True):
                st.session_state.start_point = (new_start_lat, new_start_lng)
                st.session_state.current_route = []
                save_data()
                st.rerun()
            
            st.caption(f"当前: {st.session_state.start_point[0]:.6f}, {st.session_state.start_point[1]:.6f}")
        
        with st.expander("🏁 终点手动输入", expanded=False):
            col_e1, col_e2 = st.columns(2)
            with col_e1:
                new_end_lat = st.number_input("纬度", value=float(st.session_state.end_point[0]), format="%.6f")
            with col_e2:
                new_end_lng = st.number_input("经度", value=float(st.session_state.end_point[1]), format="%.6f")
            
            if st.button("🎯 更新终点", use_container_width=True):
                st.session_state.end_point = (new_end_lat, new_end_lng)
                st.session_state.current_route = []
                save_data()
                st.rerun()
            
            st.caption(f"当前: {st.session_state.end_point[0]:.6f}, {st.session_state.end_point[1]:.6f}")
        
        st.divider()
        
        new_altitude = st.number_input(
            "✈️ 飞行高度（米）",
            min_value=0.0,
            max_value=100.0,
            value=st.session_state.flight_altitude,
            step=1.0,
            help="高于红色障碍物时需要绕行"
        )
        if new_altitude != st.session_state.flight_altitude:
            st.session_state.flight_altitude = new_altitude
            st.session_state.current_route = []
            save_data()
        
        new_radius = st.number_input(
            "🛡️ 安全半径（米）",
            min_value=1.0,
            max_value=30.0,
            value=st.session_state.safety_radius,
            step=1.0,
            help="绕行时与障碍物的距离"
        )
        if new_radius != st.session_state.safety_radius:
            st.session_state.safety_radius = new_radius
            st.session_state.current_route = []
            save_data()
        
        st.divider()
        
        st.subheader(f"📦 障碍物 ({len(st.session_state.obstacles)})")
        
        high_obs = [o for o in st.session_state.obstacles if o.get("height", 0) >= st.session_state.flight_altitude]
        low_obs = [o for o in st.session_state.obstacles if o.get("height", 0) < st.session_state.flight_altitude]
        
        col_h, col_l = st.columns(2)
        with col_h:
            st.metric("⚠️ 需绕行", len(high_obs))
        with col_l:
            st.metric("✅ 可飞越", len(low_obs))
        
        st.divider()
        
        if st.session_state.obstacles:
            for i, obs in enumerate(st.session_state.obstacles):
                col_a, col_b = st.columns([4, 1])
                with col_a:
                    height = obs.get('height', 0)
                    name = obs.get('name', '未知')
                    if height >= st.session_state.flight_altitude:
                        st.markdown(f"**🔴 {name}**")
                        st.caption(f"📏 {height}m (需绕行)")
                    else:
                        st.markdown(f"**🟢 {name}**")
                        st.caption(f"📏 {height}m (可飞越)")
                with col_b:
                    if st.button("🗑️", key=f"del_{i}"):
                        st.session_state.obstacles.pop(i)
                        st.session_state.current_route = []
                        save_data()
                        st.rerun()
                st.divider()
        else:
            st.info("📭 暂无障碍物，请在地图上绘制多边形")
        
        if st.session_state.current_route and len(st.session_state.current_route) >= 2:
            st.divider()
            st.subheader("📊 航线信息")
            total_dist = 0
            for i in range(len(st.session_state.current_route) - 1):
                total_dist += calculate_distance(st.session_state.current_route[i], st.session_state.current_route[i+1])
            st.metric("总距离", f"{total_dist:.1f} m")
            st.metric("航点数", len(st.session_state.current_route))
            
            mode_display = {"best": "🌟 最佳航线", "left": "⬅️ 向左绕行", "right": "➡️ 向右绕行"}
            st.info(f"当前模式: {mode_display[st.session_state.route_mode]}")

# ====================== 标签页2：飞行监控 ======================
with tab2:
    col_m1, col_m2 = st.columns([1, 2])
    
    with col_m1:
        st.subheader("🎮 控制面板")
        
        if not st.session_state.heartbeat_running:
            if st.button("▶️ 开始心跳模拟", use_container_width=True, type="primary"):
                st.session_state.heartbeat_running = True
                st.rerun()
        else:
            if st.button("⏸️ 停止心跳模拟", use_container_width=True):
                st.session_state.heartbeat_running = False
                st.rerun()
        
        if st.button("📡 手动心跳", use_container_width=True):
            new_seq = len(st.session_state.heartbeat_history) + 1
            st.session_state.heartbeat_history.append({
                "seq": new_seq,
                "time": datetime.now().strftime("%H:%M:%S"),
                "status": "正常"
            })
            st.rerun()
        
        if st.button("🗑️ 清空记录", use_container_width=True):
            st.session_state.heartbeat_history = []
            st.rerun()
        
        st.divider()
        
        st.subheader("📊 飞行状态")
        st.metric("飞行高度", f"{st.session_state.flight_altitude} m")
        st.metric("安全半径", f"{st.session_state.safety_radius} m")
        
        mode_display = {"best": "🌟 最佳航线", "left": "⬅️ 向左绕行", "right": "➡️ 向右绕行"}
        st.metric("绕行模式", mode_display[st.session_state.route_mode])
        
        if st.session_state.obstacles:
            max_h = max([o.get("height", 0) for o in st.session_state.obstacles])
            if st.session_state.flight_altitude > max_h:
                st.success("✅ 可直飞")
            else:
                st.warning(f"⚠️ 需绕行（最高{max_h}m）")
    
    with col_m2:
        st.subheader("📋 心跳记录")
        if st.session_state.heartbeat_history:
            df = pd.DataFrame(st.session_state.heartbeat_history)
            st.dataframe(df, use_container_width=True, hide_index=True)
        else:
            st.info("暂无心跳数据")
        
        st.divider()
        
        st.subheader("📈 心跳趋势")
        if len(st.session_state.heartbeat_history) > 0:
            seq_list = [h["seq"] for h in st.session_state.heartbeat_history]
            fig, ax = plt.subplots(figsize=(10, 3))
            ax.plot(range(1, len(seq_list)+1), seq_list, marker='o', color='#2ecc71', linewidth=2)
            ax.set_xlabel("心跳次数")
            ax.set_ylabel("序列号")
            ax.set_title("无人机心跳序列")
            ax.grid(True, alpha=0.3)
            st.pyplot(fig)

# ====================== 自动心跳 ======================
if st.session_state.heartbeat_running:
    import time
    if len(st.session_state.heartbeat_history) > 0:
        last_time_str = st.session_state.heartbeat_history[-1]["time"]
        try:
            last_time = datetime.strptime(last_time_str, "%H:%M:%S")
            now = datetime.now()
            if (now - last_time).total_seconds() >= 2:
                new_seq = len(st.session_state.heartbeat_history) + 1
                st.session_state.heartbeat_history.append({
                    "seq": new_seq,
                    "time": now.strftime("%H:%M:%S"),
                    "status": "正常"
                })
                time.sleep(0.1)
                st.rerun()
        except:
            pass
    elif len(st.session_state.heartbeat_history) == 0:
        st.session_state.heartbeat_history.append({
            "seq": 1,
            "time": datetime.now().strftime("%H:%M:%S"),
            "status": "正常"
        })
        time.sleep(0.1)
        st.rerun()

# ====================== 页脚 ======================
st.markdown("---")
st.markdown("🚁 无人机航线规划系统 | 三种绕行模式：向左绕行 ⬅️ | 向右绕行 ➡️ | 最佳航线 🌟")
