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

# ====================== 页面配置 ======================
st.set_page_config(
    page_title="无人机航线规划系统",
    layout="wide",
    initial_sidebar_state="expanded"
)

st.title("🚁 无人机航线规划与障碍物管理系统")

# ====================== 初始化 Session State ======================
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
    st.session_state.flight_altitude = 20.0
if "safety_radius" not in st.session_state:
    st.session_state.safety_radius = 8.0
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

# ====================== 配置文件路径 ======================
CONFIG_FILE = "drone_config.json"

# ====================== 保存数据 ======================
def save_config():
    config = {
        "start_point": st.session_state.start_point,
        "end_point": st.session_state.end_point,
        "obstacles": st.session_state.obstacles,
        "flight_altitude": st.session_state.flight_altitude,
        "safety_radius": st.session_state.safety_radius,
        "route_mode": st.session_state.route_mode,
        "save_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    }
    with open(CONFIG_FILE, "w", encoding="utf-8") as f:
        json.dump(config, f, ensure_ascii=False, indent=2)

# ====================== 加载数据 ======================
def load_config():
    if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE, "r", encoding="utf-8") as f:
            config = json.load(f)
        st.session_state.start_point = tuple(config.get("start_point", (32.2345, 118.7492)))
        st.session_state.end_point = tuple(config.get("end_point", (32.2337, 118.7496)))
        st.session_state.obstacles = config.get("obstacles", [])
        st.session_state.flight_altitude = config.get("flight_altitude", 20.0)
        st.session_state.safety_radius = config.get("safety_radius", 8.0)
        st.session_state.route_mode = config.get("route_mode", "best")

# 加载配置
load_config()

# ====================== 几何计算函数 ======================
def point_in_polygon(point, polygon):
    """判断点是否在多边形内"""
    x, y = point
    inside = False
    n = len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        if ((y1 > y) != (y2 > y)) and (x < (x2 - x1) * (y - y1) / (y2 - y1) + x1):
            inside = not inside
    return inside

def line_segments_intersect(p1, p2, p3, p4):
    """判断两条线段是否相交"""
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
    for i in range(len(polygon)):
        p1 = polygon[i]
        p2 = polygon[(i + 1) % len(polygon)]
        if line_segments_intersect(line_start, line_end, p1, p2):
            return True
    if point_in_polygon(line_start, polygon) or point_in_polygon(line_end, polygon):
        return True
    return False

def distance_between_points(p1, p2):
    """计算两点之间的距离（米）"""
    lat1, lng1 = p1
    lat2, lng2 = p2
    R = 6371000
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lng2 - lng1)
    a = math.sin(delta_phi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c

def get_polygon_center(polygon):
    """获取多边形中心点"""
    lats = [p[0] for p in polygon]
    lngs = [p[1] for p in polygon]
    return sum(lats)/len(lats), sum(lngs)/len(lngs)

def get_polygon_size(polygon, center_lat):
    """获取多边形尺寸（米）"""
    lats = [p[0] for p in polygon]
    lngs = [p[1] for p in polygon]
    width_lat = (max(lats) - min(lats)) * 111320
    width_lng = (max(lngs) - min(lngs)) * 111320 * math.cos(math.radians(center_lat))
    return max(width_lat, width_lng)

def get_offset_point(start, end, polygon, safety_radius, direction):
    """获取绕行偏移点"""
    center_lat, center_lng = get_polygon_center(polygon)
    poly_size = get_polygon_size(polygon, center_lat)
    
    offset_dist = poly_size / 2 + safety_radius
    
    lat_mid = (start[0] + end[0]) / 2
    lng_per_m = 1 / (111320 * math.cos(math.radians(lat_mid)))
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

def find_blocking_obstacle(start, end, obstacles, flight_alt):
    """找到阻挡路径的障碍物"""
    for obs in obstacles:
        if obs.get("height", 0) >= flight_alt:
            polygon = obs.get("polygon", [])
            if polygon and line_intersects_polygon(start, end, polygon):
                return obs
    return None

# ====================== 航线规划算法 ======================
def plan_route_left(start, end, obstacles, flight_alt, safety_radius):
    """左侧绕行"""
    waypoints = [start]
    current = start
    remaining = obstacles.copy()
    
    for _ in range(10):
        obs = find_blocking_obstacle(current, end, remaining, flight_alt)
        if obs is None:
            waypoints.append(end)
            break
        bypass = get_offset_point(current, end, obs["polygon"], safety_radius, 'left')
        waypoints.append(bypass)
        current = bypass
        remaining = [o for o in remaining if o != obs]
    
    return waypoints

def plan_route_right(start, end, obstacles, flight_alt, safety_radius):
    """右侧绕行"""
    waypoints = [start]
    current = start
    remaining = obstacles.copy()
    
    for _ in range(10):
        obs = find_blocking_obstacle(current, end, remaining, flight_alt)
        if obs is None:
            waypoints.append(end)
            break
        bypass = get_offset_point(current, end, obs["polygon"], safety_radius, 'right')
        waypoints.append(bypass)
        current = bypass
        remaining = [o for o in remaining if o != obs]
    
    return waypoints

def plan_route_best(start, end, obstacles, flight_alt, safety_radius):
    """最佳航线"""
    left_route = plan_route_left(start, end, obstacles, flight_alt, safety_radius)
    right_route = plan_route_right(start, end, obstacles, flight_alt, safety_radius)
    
    left_dist = sum(distance_between_points(left_route[i], left_route[i+1]) for i in range(len(left_route)-1))
    right_dist = sum(distance_between_points(right_route[i], right_route[i+1]) for i in range(len(right_route)-1))
    
    return left_route if left_dist <= right_dist else right_route

def calculate_route():
    """计算航线"""
    start = st.session_state.start_point
    end = st.session_state.end_point
    obstacles = st.session_state.obstacles
    flight_alt = st.session_state.flight_altitude
    safety = st.session_state.safety_radius
    mode = st.session_state.route_mode
    
    high_obs = [o for o in obstacles if o.get("height", 0) >= flight_alt]
    
    if not high_obs:
        st.session_state.current_route = [start, end]
        return
    
    if find_blocking_obstacle(start, end, high_obs, flight_alt) is None:
        st.session_state.current_route = [start, end]
        return
    
    if mode == "left":
        st.session_state.current_route = plan_route_left(start, end, high_obs, flight_alt, safety)
    elif mode == "right":
        st.session_state.current_route = plan_route_right(start, end, high_obs, flight_alt, safety)
    else:
        st.session_state.current_route = plan_route_best(start, end, high_obs, flight_alt, safety)

# ====================== 创建地图 ======================
def create_map():
    """创建Folium地图"""
    m = folium.Map(
        location=st.session_state.map_center,
        zoom_start=19,
        tiles="https://webst01.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}",
        attr="高德地图"
    )
    
    # 绘图工具
    draw = Draw(
        draw_options={
            "polygon": {
                "allowIntersection": False,
                "shapeOptions": {"color": "#ff4444", "fillColor": "#ff4444", "fillOpacity": 0.3}
            }
        },
        edit_options={"edit": True, "remove": True}
    )
    draw.add_to(m)
    
    # 起点
    folium.Marker(
        location=st.session_state.start_point,
        popup=f"🚁 起点\n{st.session_state.start_point[0]:.6f}, {st.session_state.start_point[1]:.6f}",
        icon=folium.Icon(color="red", icon="play", prefix="fa")
    ).add_to(m)
    
    # 终点
    folium.Marker(
        location=st.session_state.end_point,
        popup=f"🎯 终点\n{st.session_state.end_point[0]:.6f}, {st.session_state.end_point[1]:.6f}",
        icon=folium.Icon(color="green", icon="flag-checkered", prefix="fa")
    ).add_to(m)
    
    # 障碍物
    for i, obs in enumerate(st.session_state.obstacles):
        polygon = obs.get("polygon", [])
        height = obs.get("height", 10)
        name = obs.get("name", f"障碍{i+1}")
        
        if polygon:
            if height >= st.session_state.flight_altitude:
                color = "#ff0000"
                status = "⚠️ 需绕行"
            else:
                color = "#00aa00"
                status = "✅ 可飞越"
            
            folium.Polygon(
                locations=polygon,
                color=color,
                weight=2,
                fill=True,
                fill_color=color,
                fill_opacity=0.3,
                popup=f"{name}\n高度: {height}m\n{status}"
            ).add_to(m)
    
    # 航线
    if st.session_state.current_route:
        colors = {"left": "#ff8800", "right": "#aa66ff", "best": "#00ff00"}
        folium.PolyLine(
            locations=st.session_state.current_route,
            color=colors.get(st.session_state.route_mode, "#00ff00"),
            weight=5,
            opacity=0.8
        ).add_to(m)
        
        for i, point in enumerate(st.session_state.current_route):
            if i == 0:
                c = "red"
            elif i == len(st.session_state.current_route) - 1:
                c = "green"
            else:
                c = "orange"
            folium.CircleMarker(
                location=point,
                radius=5,
                color=c,
                fill=True,
                popup=f"航点{i+1}"
            ).add_to(m)
    
    return m

# ====================== UI界面 ======================
tab1, tab2 = st.tabs(["🗺️ 航线规划", "📡 飞行监控"])

# ====================== 标签页1 ======================
with tab1:
    # 按钮行
    col1, col2, col3, col4, col5 = st.columns(5)
    with col1:
        if st.button("🎯 规划航线", use_container_width=True, type="primary"):
            calculate_route()
            save_config()
            st.success("航线规划完成！")
            st.rerun()
    with col2:
        if st.button("💾 保存配置", use_container_width=True):
            save_config()
            st.success("已保存")
    with col3:
        if st.button("🗑️ 清空障碍物", use_container_width=True):
            st.session_state.obstacles = []
            st.session_state.current_route = []
            save_config()
            st.rerun()
    with col4:
        if st.button("🗺️ 重置地图", use_container_width=True):
            st.session_state.map_center = [32.2341, 118.7494]
            st.rerun()
    with col5:
        if st.button("❌ 取消模式", use_container_width=True):
            st.session_state.set_mode = None
            st.rerun()
    
    st.divider()
    
    # 模式提示
    if st.session_state.set_mode == 'start':
        st.info("🔴 点击地图设置起点")
    elif st.session_state.set_mode == 'end':
        st.info("🟢 点击地图设置终点")
    
    # 左右布局
    left_col, right_col = st.columns([3, 1])
    
    with left_col:
        st.subheader("🗺️ 地图")
        st.caption("💡 点击左侧多边形工具绘制障碍物 | 点击下方按钮后点击地图设置起点/终点")
        
        m = create_map()
        output = st_folium(m, width=800, height=550, returned_objects=["last_clicked", "all_drawings"])
        
        # 处理点击设置起点/终点
        if output and output.get("last_clicked"):
            click = output["last_clicked"]
            if click and "lat" in click:
                lat, lng = click["lat"], click["lng"]
                if st.session_state.set_mode == 'start':
                    st.session_state.start_point = (lat, lng)
                    st.session_state.current_route = []
                    st.session_state.set_mode = None
                    save_config()
                    st.success(f"起点已设置: {lat:.6f}, {lng:.6f}")
                    st.rerun()
                elif st.session_state.set_mode == 'end':
                    st.session_state.end_point = (lat, lng)
                    st.session_state.current_route = []
                    st.session_state.set_mode = None
                    save_config()
                    st.success(f"终点已设置: {lat:.6f}, {lng:.6f}")
                    st.rerun()
        
        # 处理绘制的多边形
        if output and output.get("all_drawings"):
            drawings = output["all_drawings"]
            if drawings != st.session_state.last_drawings:
                st.session_state.last_drawings = drawings
                for draw in drawings:
                    if draw.get("geometry", {}).get("type") == "Polygon":
                        coords = draw["geometry"]["coordinates"][0]
                        polygon = [[c[1], c[0]] for c in coords]
                        st.session_state.pending_polygon = polygon
                        st.rerun()
        
        # 添加障碍物表单
        if st.session_state.pending_polygon:
            with st.container():
                st.markdown("### ➕ 添加障碍物")
                col_a, col_b = st.columns(2)
                with col_a:
                    name = st.text_input("名称", value=f"障碍_{len(st.session_state.obstacles)+1}")
                with col_b:
                    height = st.number_input("高度(m)", min_value=0, max_value=100, value=10)
                
                col_c, col_d = st.columns(2)
                with col_c:
                    if st.button("✅ 添加", use_container_width=True):
                        st.session_state.obstacles.append({
                            "name": name,
                            "height": height,
                            "polygon": st.session_state.pending_polygon,
                            "create_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                        })
                        st.session_state.pending_polygon = None
                        st.session_state.current_route = []
                        save_config()
                        st.rerun()
                with col_d:
                    if st.button("❌ 取消", use_container_width=True):
                        st.session_state.pending_polygon = None
                        st.rerun()
    
    with right_col:
        st.subheader("⚙️ 参数设置")
        
        # 绕行模式
        st.markdown("### 🗺️ 绕行模式")
        mode = st.radio(
            "选择模式",
            ["best", "left", "right"],
            format_func=lambda x: {"best": "🌟 最佳航线", "left": "⬅️ 向左绕行", "right": "➡️ 向右绕行"}[x],
            index=["best", "left", "right"].index(st.session_state.route_mode)
        )
        if mode != st.session_state.route_mode:
            st.session_state.route_mode = mode
            st.session_state.current_route = []
            save_config()
            st.rerun()
        
        st.divider()
        
        # 坐标设置
        st.markdown("### 🎯 坐标设置")
        col_s1, col_s2 = st.columns(2)
        with col_s1:
            if st.button("📍 设起点", use_container_width=True):
                st.session_state.set_mode = 'start'
                st.rerun()
        with col_s2:
            if st.button("🏁 设终点", use_container_width=True):
                st.session_state.set_mode = 'end'
                st.rerun()
        
        # 手动输入起点
        with st.expander("✏️ 手动输入起点"):
            lat = st.number_input("纬度", value=st.session_state.start_point[0], format="%.6f")
            lng = st.number_input("经度", value=st.session_state.start_point[1], format="%.6f")
            if st.button("更新起点"):
                st.session_state.start_point = (lat, lng)
                st.session_state.current_route = []
                save_config()
                st.rerun()
        
        # 手动输入终点
        with st.expander("✏️ 手动输入终点"):
            lat = st.number_input("纬度", value=st.session_state.end_point[0], format="%.6f")
            lng = st.number_input("经度", value=st.session_state.end_point[1], format="%.6f")
            if st.button("更新终点"):
                st.session_state.end_point = (lat, lng)
                st.session_state.current_route = []
                save_config()
                st.rerun()
        
        st.divider()
        
        # 飞行参数
        new_alt = st.number_input(
            "✈️ 飞行高度(m)",
            min_value=0.0, max_value=100.0,
            value=st.session_state.flight_altitude, step=1.0
        )
        if new_alt != st.session_state.flight_altitude:
            st.session_state.flight_altitude = new_alt
            st.session_state.current_route = []
            save_config()
        
        new_safety = st.number_input(
            "🛡️ 安全半径(m)",
            min_value=1.0, max_value=30.0,
            value=st.session_state.safety_radius, step=1.0
        )
        if new_safety != st.session_state.safety_radius:
            st.session_state.safety_radius = new_safety
            st.session_state.current_route = []
            save_config()
        
        st.divider()
        
        # 障碍物统计
        st.subheader(f"📦 障碍物 ({len(st.session_state.obstacles)})")
        high = [o for o in st.session_state.obstacles if o.get("height", 0) >= st.session_state.flight_altitude]
        low = [o for o in st.session_state.obstacles if o.get("height", 0) < st.session_state.flight_altitude]
        col_h, col_l = st.columns(2)
        with col_h:
            st.metric("⚠️ 需绕行", len(high))
        with col_l:
            st.metric("✅ 可飞越", len(low))
        
        st.divider()
        
        # 障碍物列表
        if st.session_state.obstacles:
            for i, obs in enumerate(st.session_state.obstacles):
                col_a, col_b = st.columns([3, 1])
                with col_a:
                    h = obs.get("height", 0)
                    name = obs.get("name", "未知")
                    if h >= st.session_state.flight_altitude:
                        st.markdown(f"🔴 **{name}** - {h}m")
                    else:
                        st.markdown(f"🟢 **{name}** - {h}m")
                with col_b:
                    if st.button("🗑️", key=f"del_{i}"):
                        st.session_state.obstacles.pop(i)
                        st.session_state.current_route = []
                        save_config()
                        st.rerun()
        else:
            st.info("暂无障-碍物")
        
        # 航线信息
        if st.session_state.current_route:
            st.divider()
            total_dist = 0
            for i in range(len(st.session_state.current_route)-1):
                total_dist += distance_between_points(st.session_state.current_route[i], st.session_state.current_route[i+1])
            st.metric("📏 航线距离", f"{total_dist:.1f} m")
            st.metric("📍 航点数", len(st.session_state.current_route))

# ====================== 标签页2 ======================
with tab2:
    col_a, col_b = st.columns([1, 2])
    
    with col_a:
        st.subheader("🎮 控制面板")
        
        if not st.session_state.heartbeat_running:
            if st.button("▶️ 开始心跳", use_container_width=True, type="primary"):
                st.session_state.heartbeat_running = True
                st.rerun()
        else:
            if st.button("⏸️ 停止心跳", use_container_width=True):
                st.session_state.heartbeat_running = False
                st.rerun()
        
        if st.button("📡 手动心跳", use_container_width=True):
            st.session_state.heartbeat_history.append({
                "seq": len(st.session_state.heartbeat_history) + 1,
                "time": datetime.now().strftime("%H:%M:%S.%f")[:-3],
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
        
        mode_name = {"best": "最佳航线", "left": "向左绕行", "right": "向右绕行"}
        st.metric("绕行模式", mode_name[st.session_state.route_mode])
        
        if st.session_state.obstacles:
            max_h = max(o.get("height", 0) for o in st.session_state.obstacles)
            if st.session_state.flight_altitude > max_h:
                st.success("✅ 可直飞")
            else:
                st.warning(f"⚠️ 需绕行 (最高{max_h}m)")
    
    with col_b:
        st.subheader("📋 心跳记录")
        if st.session_state.heartbeat_history:
            df = pd.DataFrame(st.session_state.heartbeat_history)
            st.dataframe(df, use_container_width=True, hide_index=True)
        else:
            st.info("暂无心跳数据")
        
        st.divider()
        
        st.subheader("📈 心跳趋势图")
        if len(st.session_state.heartbeat_history) > 0:
            seq = [h["seq"] for h in st.session_state.heartbeat_history]
            fig, ax = plt.subplots(figsize=(10, 3))
            ax.plot(range(1, len(seq)+1), seq, marker='o', color='#2ecc71', linewidth=2)
            ax.set_xlabel("次数")
            ax.set_ylabel("序列号")
            ax.set_title("无人机心跳序列")
            ax.grid(True, alpha=0.3)
            st.pyplot(fig)

# ====================== 自动心跳 ======================
if st.session_state.heartbeat_running:
    import time
    now = datetime.now()
    
    if len(st.session_state.heartbeat_history) == 0:
        st.session_state.heartbeat_history.append({
            "seq": 1,
            "time": now.strftime("%H:%M:%S.%f")[:-3],
            "status": "正常"
        })
        time.sleep(0.1)
        st.rerun()
    else:
        last_time = datetime.strptime(st.session_state.heartbeat_history[-1]["time"], "%H:%M:%S.%f")
        if (now - last_time).total_seconds() >= 2:
            st.session_state.heartbeat_history.append({
                "seq": len(st.session_state.heartbeat_history) + 1,
                "time": now.strftime("%H:%M:%S.%f")[:-3],
                "status": "正常"
            })
            time.sleep(0.1)
            st.rerun()

# ====================== 页脚 ======================
st.markdown("---")
st.markdown("🚁 无人机航线规划系统 | 三种绕行模式 | 高德地图")
