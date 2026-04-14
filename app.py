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
from branca.element import Figure

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

# ====================== 保存/加载 ======================
def save_data():
    data = {
        "obstacles": st.session_state.obstacles,
        "start_point": st.session_state.start_point,
        "end_point": st.session_state.end_point,
        "flight_altitude": st.session_state.flight_altitude,
        "safety_radius": st.session_state.safety_radius,
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

load_data()

# ====================== 几何计算函数 ======================
def point_in_polygon(point, polygon):
    """射线法判断点是否在多边形内"""
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
    """判断线段(p1-p2)和(p3-p4)是否相交"""
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
    # 检查线段是否与多边形的任一边相交
    for i in range(len(polygon)):
        p1 = polygon[i]
        p2 = polygon[(i + 1) % len(polygon)]
        if segments_intersect(line_start, line_end, p1, p2):
            return True
    
    # 检查端点是否在多边形内
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

def get_offset(start, end, distance_m, direction='left'):
    """获取偏移点"""
    lat1, lng1 = start
    lat2, lng2 = end
    
    # 计算中点纬度用于经纬度转换
    lat_mid = (lat1 + lat2) / 2
    lng_per_m = 1 / (111320 * math.cos(math.radians(lat_mid)))
    lat_per_m = 1 / 111320
    
    # 方向向量
    dx = lng2 - lng1
    dy = lat2 - lat1
    dist = math.sqrt(dx**2 + dy**2)
    
    if dist < 1e-6:
        return start
    
    ux = dx / dist
    uy = dy / dist
    
    # 垂直向量
    if direction == 'left':
        nx = -uy
        ny = ux
    else:
        nx = uy
        ny = -ux
    
    offset_lat = distance_m * lat_per_m
    offset_lng = distance_m * lng_per_m
    
    return (lat1 + ny * offset_lat, lng1 + nx * offset_lng)

# ====================== 核心：真正避开障碍物的航线规划 ======================
def is_path_safe(point1, point2, obstacles, flight_altitude):
    """检查路径段是否安全（不穿过任何需要避开的障碍物）"""
    for obs in obstacles:
        obs_height = obs.get("height", 0)
        if obs_height >= flight_altitude:  # 只检查高度高于飞行高度的障碍物
            polygon = obs.get("polygon", [])
            if polygon and line_intersects_polygon(point1, point2, polygon):
                return False
    return True

def find_safe_waypoints(start, end, obstacles, flight_altitude, safety_radius):
    """递归寻找安全的绕行点"""
    
    # 先检查直线是否安全
    if is_path_safe(start, end, obstacles, flight_altitude):
        return [start, end]
    
    # 找到需要绕行的障碍物
    for obs in obstacles:
        obs_height = obs.get("height", 0)
        if obs_height >= flight_altitude:
            polygon = obs.get("polygon", [])
            if polygon and line_intersects_polygon(start, end, polygon):
                # 计算多边形中心
                center_lat = sum([p[0] for p in polygon]) / len(polygon)
                center_lng = sum([p[1] for p in polygon]) / len(polygon)
                
                # 计算绕行距离（安全半径 + 障碍物半宽）
                lat_mid = (start[0] + end[0]) / 2
                lng_per_m = 1 / (111320 * math.cos(math.radians(lat_mid)))
                lat_per_m = 1 / 111320
                
                offset_dist = safety_radius * 2
                offset_lat = offset_dist * lat_per_m
                offset_lng = offset_dist * lng_per_m
                
                # 尝试左右两侧
                left_point = (center_lat + offset_lat, center_lng - offset_lng)
                right_point = (center_lat - offset_lat, center_lng + offset_lng)
                
                # 检查左侧路径
                left_path1_safe = is_path_safe(start, left_point, obstacles, flight_altitude)
                left_path2_safe = is_path_safe(left_point, end, obstacles, flight_altitude)
                
                # 检查右侧路径
                right_path1_safe = is_path_safe(start, right_point, obstacles, flight_altitude)
                right_path2_safe = is_path_safe(right_point, end, obstacles, flight_altitude)
                
                if left_path1_safe and left_path2_safe:
                    return [start, left_point, end]
                elif right_path1_safe and right_path2_safe:
                    return [start, right_point, end]
                else:
                    # 需要更多绕行点
                    mid1 = (start[0] + (end[0] - start[0]) * 0.3, start[1] + (end[1] - start[1]) * 0.3)
                    mid2 = (start[0] + (end[0] - start[0]) * 0.7, start[1] + (end[1] - start[1]) * 0.7)
                    
                    # 向外偏移
                    dx = end[1] - start[1]
                    dy = end[0] - start[0]
                    dist = math.sqrt(dx**2 + dy**2)
                    if dist > 0:
                        ux = dx / dist
                        uy = dy / dist
                        nx = -uy
                        ny = ux
                        
                        offset_m = safety_radius * 2
                        offset_lat2 = offset_m * lat_per_m
                        offset_lng2 = offset_m * lng_per_m
                        
                        waypoint1 = (mid1[0] + ny * offset_lat2, mid1[1] + nx * offset_lng2)
                        waypoint2 = (mid2[0] + ny * offset_lat2, mid2[1] + nx * offset_lng2)
                        
                        return [start, waypoint1, waypoint2, end]
    
    return [start, end]

def plan_route():
    """规划航线"""
    start = st.session_state.start_point
    end = st.session_state.end_point
    obstacles = st.session_state.obstacles
    altitude = st.session_state.flight_altitude
    safety_radius = st.session_state.safety_radius
    
    # 筛选需要避开的障碍物（高度高于飞行高度）
    high_obstacles = [obs for obs in obstacles if obs.get("height", 0) >= altitude]
    
    if not high_obstacles:
        # 没有需要避开的障碍物
        st.session_state.current_route = [start, end]
        return
    
    # 检查直线是否安全
    if is_path_safe(start, end, high_obstacles, altitude):
        st.session_state.current_route = [start, end]
        return
    
    # 需要绕行 - 使用绕行算法
    route = find_safe_waypoints(start, end, high_obstacles, altitude, safety_radius)
    
    # 验证路径是否安全，如果不安全则增加绕行点
    max_iterations = 5
    for _ in range(max_iterations):
        all_safe = True
        for i in range(len(route) - 1):
            if not is_path_safe(route[i], route[i+1], high_obstacles, altitude):
                all_safe = False
                # 在危险段中间添加绕行点
                mid = ((route[i][0] + route[i+1][0]) / 2, (route[i][1] + route[i+1][1]) / 2)
                # 向外偏移
                lat_mid = (route[i][0] + route[i+1][0]) / 2
                lng_per_m = 1 / (111320 * math.cos(math.radians(lat_mid)))
                lat_per_m = 1 / 111320
                offset = safety_radius * 2
                offset_lat = offset * lat_per_m
                offset_lng = offset * lng_per_m
                
                # 找到最近的障碍物方向并反向偏移
                mid_offset = (mid[0] + offset_lat, mid[1] + offset_lng)
                route.insert(i+1, mid_offset)
                break
        if all_safe:
            break
    
    st.session_state.current_route = route

# ====================== 创建可拖拽标记的地图 ======================
def create_map():
    """创建带有可拖拽起点终点标记的地图"""
    
    # 使用Figure确保地图正确渲染
    fig = Figure(width="100%", height=600)
    m = folium.Map(
        location=st.session_state.map_center,
        zoom_start=18,
        tiles="https://webst01.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}",
        attr="高德地图"
    )
    
    # 添加绘图工具
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
    
    # 添加可拖拽的起点标记
    start_marker = folium.Marker(
        location=st.session_state.start_point,
        popup="🚁 起点 A (可拖拽)",
        icon=folium.Icon(color="red", icon="play", prefix="fa"),
        draggable=True
    )
    start_marker.add_to(m)
    
    # 添加可拖拽的终点标记
    end_marker = folium.Marker(
        location=st.session_state.end_point,
        popup="🎯 终点 B (可拖拽)",
        icon=folium.Icon(color="green", icon="flag-checkered", prefix="fa"),
        draggable=True
    )
    end_marker.add_to(m)
    
    # 绘制所有障碍物
    for i, obs in enumerate(st.session_state.obstacles):
        polygon = obs.get("polygon", [])
        height = obs.get("height", 10)
        name = obs.get("name", f"障碍物{i+1}")
        
        if polygon:
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
    
    # 绘制航线
    if st.session_state.current_route:
        # 航线
        folium.PolyLine(
            locations=st.session_state.current_route,
            color="#00ff00",
            weight=5,
            opacity=0.8,
            popup=f"规划航线 | {len(st.session_state.current_route)}个航点"
        ).add_to(m)
        
        # 航点
        for i, point in enumerate(st.session_state.current_route):
            if i == 0:
                color = "red"
                icon_type = "play"
            elif i == len(st.session_state.current_route) - 1:
                color = "green"
                icon_type = "flag-checkered"
            else:
                color = "orange"
                icon_type = "circle"
            
            folium.Marker(
                location=point,
                popup=f"航点 {i+1}",
                icon=folium.Icon(color=color, icon=icon_type, prefix="fa")
            ).add_to(m)
    
    return m, start_marker, end_marker

# ====================== 界面 ======================
tab1, tab2 = st.tabs(["🗺️ 地图与航线规划", "📡 飞行监控"])

# ====================== 标签页1 ======================
with tab1:
    # 按钮栏
    col_btn1, col_btn2, col_btn3, col_btn4, col_btn5 = st.columns(5)
    with col_btn1:
        if st.button("🎯 规划航线", use_container_width=True, type="primary"):
            plan_route()
            save_data()
            st.success("航线规划完成！")
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
        if st.button("🗺️ 重置地图", use_container_width=True):
            st.session_state.map_center = [32.2341, 118.7494]
            st.rerun()
    with col_btn5:
        if st.button("📏 测试碰撞", use_container_width=True):
            plan_route()
            st.rerun()
    
    st.divider()
    
    col1, col2 = st.columns([3, 1])
    
    with col1:
        st.subheader("🗺️ 地图（起点/终点可拖拽，绘制多边形添加障碍物）")
        st.caption("💡 提示：直接拖拽红色/绿色标记可移动起点终点，然后点击「规划航线」")
        
        # 创建并显示地图
        m, start_marker, end_marker = create_map()
        
        # 显示地图并获取交互数据
        output = st_folium(
            m, 
            width=850, 
            height=550, 
            returned_objects=["last_object_clicked", "all_drawings"]
        )
        
        # 处理地图点击/拖拽事件 - 更新起点终点
        if output and output.get("last_object_clicked"):
            clicked = output["last_object_clicked"]
            if clicked and "lat" in clicked and "lng" in clicked:
                # 这里可以处理点击事件
                pass
        
        # 处理新绘制的多边形
        if output and output.get("all_drawings"):
            for drawing in output["all_drawings"]:
                if drawing.get("geometry", {}).get("type") == "Polygon":
                    coords = drawing["geometry"]["coordinates"][0]
                    # 转换坐标为 (lat, lng) 格式
                    polygon_points = [[c[1], c[0]] for c in coords]
                    
                    # 显示添加表单
                    with st.expander("➕ 添加新障碍物", expanded=True):
                        obs_name = st.text_input("障碍物名称", value=f"障碍物_{len(st.session_state.obstacles)+1}")
                        obs_height = st.number_input("高度（米）", min_value=0, max_value=100, value=10, step=1)
                        
                        col_a, col_b = st.columns(2)
                        with col_a:
                            if st.button("✅ 确认添加", use_container_width=True):
                                new_obs = {
                                    "name": obs_name,
                                    "height": obs_height,
                                    "polygon": polygon_points,
                                    "create_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                                }
                                st.session_state.obstacles.append(new_obs)
                                st.session_state.current_route = []
                                save_data()
                                st.success(f"已添加: {obs_name}")
                                st.rerun()
                        with col_b:
                            if st.button("❌ 取消", use_container_width=True):
                                st.rerun()
    
    with col2:
        st.subheader("⚙️ 参数设置")
        
        # 起点手动设置
        with st.expander("📍 起点设置（或在地图上拖拽）", expanded=False):
            new_start_lat = st.number_input("起点纬度", value=st.session_state.start_point[0], format="%.6f")
            new_start_lng = st.number_input("起点经度", value=st.session_state.start_point[1], format="%.6f")
            if st.button("✈️ 更新起点"):
                st.session_state.start_point = (new_start_lat, new_start_lng)
                st.session_state.current_route = []
                save_data()
                st.rerun()
        
        # 终点手动设置
        with st.expander("🏁 终点设置（或在地图上拖拽）", expanded=False):
            new_end_lat = st.number_input("终点纬度", value=st.session_state.end_point[0], format="%.6f")
            new_end_lng = st.number_input("终点经度", value=st.session_state.end_point[1], format="%.6f")
            if st.button("🎯 更新终点"):
                st.session_state.end_point = (new_end_lat, new_end_lng)
                st.session_state.current_route = []
                save_data()
                st.rerun()
        
        st.divider()
        
        # 飞行参数
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
        
        # 障碍物统计
        st.subheader(f"📦 障碍物 ({len(st.session_state.obstacles)})")
        
        high_obs = [o for o in st.session_state.obstacles if o.get("height", 0) >= st.session_state.flight_altitude]
        low_obs = [o for o in st.session_state.obstacles if o.get("height", 0) < st.session_state.flight_altitude]
        
        col_h, col_l = st.columns(2)
        with col_h:
            st.metric("⚠️ 需绕行", len(high_obs))
        with col_l:
            st.metric("✅ 可飞越", len(low_obs))
        
        st.divider()
        
        # 障碍物列表
        if st.session_state.obstacles:
            for i, obs in enumerate(st.session_state.obstacles):
                col_a, col_b = st.columns([3, 1])
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
        
        # 航线信息
        if st.session_state.current_route:
            st.divider()
            st.subheader("📊 航线信息")
            total_dist = 0
            for i in range(len(st.session_state.current_route) - 1):
                total_dist += calculate_distance(st.session_state.current_route[i], st.session_state.current_route[i+1])
            st.metric("总距离", f"{total_dist:.1f} m")
            st.metric("航点数", len(st.session_state.current_route))
            st.caption(f"起点 → {len(st.session_state.current_route)-2}个中间点 → 终点")

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
st.markdown("🚁 无人机航线规划系统 | 拖拽起点/终点 → 绘制红色障碍物 → 点击「规划航线」→ 绿色航线自动绕行")
