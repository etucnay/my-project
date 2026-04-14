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

# ====================== 页面全局配置 ======================
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
    st.session_state.safety_radius = 8.0
if "current_route" not in st.session_state:
    st.session_state.current_route = []
if "route_planned" not in st.session_state:
    st.session_state.route_planned = False

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
        st.session_state.safety_radius = data.get("safety_radius", 8.0)

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

# ====================== 航线规划 ======================
def plan_route():
    """规划避开障碍物的航线"""
    start = st.session_state.start_point
    end = st.session_state.end_point
    obstacles = st.session_state.obstacles
    altitude = st.session_state.flight_altitude
    safety_radius = st.session_state.safety_radius
    
    # 检查障碍物高度
    high_obstacles = [obs for obs in obstacles if obs.get("height", 0) >= altitude]
    
    if not high_obstacles:
        # 没有需要避开的障碍物
        st.session_state.current_route = [start, end]
        st.session_state.route_planned = True
        return
    
    # 检查直线是否穿过任何障碍物
    straight_line_safe = True
    for obs in high_obstacles:
        polygon = obs.get("polygon", [])
        if polygon and line_intersects_polygon(start, end, polygon):
            straight_line_safe = False
            break
    
    if straight_line_safe:
        st.session_state.current_route = [start, end]
        st.session_state.route_planned = True
        return
    
    # 需要绕行 - 生成绕行点
    lat_mid = (start[0] + end[0]) / 2
    lng_per_m = 1 / (111320 * math.cos(math.radians(lat_mid)))
    lat_per_m = 1 / 111320
    
    offset_m = safety_radius * 1.5
    offset_lat = offset_m * lat_per_m
    offset_lng = offset_m * lng_per_m
    
    dx = end[1] - start[1]
    dy = end[0] - start[0]
    dist = math.sqrt(dx**2 + dy**2)
    
    if dist < 1e-6:
        st.session_state.current_route = [start, end]
        st.session_state.route_planned = True
        return
    
    ux = dx / dist
    uy = dy / dist
    nx = -uy
    ny = ux
    
    # 尝试左右两侧绕行
    left_point1 = (start[0] + ny * offset_lat, start[1] + nx * offset_lng)
    left_point2 = (end[0] + ny * offset_lat, end[1] + nx * offset_lng)
    right_point1 = (start[0] - ny * offset_lat, start[1] - nx * offset_lng)
    right_point2 = (end[0] - ny * offset_lat, end[1] - nx * offset_lng)
    
    # 检查左右路径是否安全
    left_safe = True
    for obs in high_obstacles:
        polygon = obs.get("polygon", [])
        if polygon:
            if line_intersects_polygon(start, left_point1, polygon) or \
               line_intersects_polygon(left_point1, left_point2, polygon) or \
               line_intersects_polygon(left_point2, end, polygon):
                left_safe = False
                break
    
    right_safe = True
    for obs in high_obstacles:
        polygon = obs.get("polygon", [])
        if polygon:
            if line_intersects_polygon(start, right_point1, polygon) or \
               line_intersects_polygon(right_point1, right_point2, polygon) or \
               line_intersects_polygon(right_point2, end, polygon):
                right_safe = False
                break
    
    if left_safe and right_safe:
        # 都安全，选择较短的
        left_dist = calculate_distance(start, left_point1) + calculate_distance(left_point1, left_point2) + calculate_distance(left_point2, end)
        right_dist = calculate_distance(start, right_point1) + calculate_distance(right_point1, right_point2) + calculate_distance(right_point2, end)
        if left_dist <= right_dist:
            st.session_state.current_route = [start, left_point1, left_point2, end]
        else:
            st.session_state.current_route = [start, right_point1, right_point2, end]
    elif left_safe:
        st.session_state.current_route = [start, left_point1, left_point2, end]
    elif right_safe:
        st.session_state.current_route = [start, right_point1, right_point2, end]
    else:
        # 都危险，使用更大的偏移
        offset_m2 = safety_radius * 3
        offset_lat2 = offset_m2 * lat_per_m
        offset_lng2 = offset_m2 * lng_per_m
        far_point1 = (start[0] + ny * offset_lat2, start[1] + nx * offset_lng2)
        far_point2 = (end[0] + ny * offset_lat2, end[1] + nx * offset_lng2)
        st.session_state.current_route = [start, far_point1, far_point2, end]
    
    st.session_state.route_planned = True

# ====================== 界面 ======================
tab1, tab2 = st.tabs(["🗺️ 地图与航线规划", "📡 飞行监控"])

# ====================== 标签页1 ======================
with tab1:
    # 按钮栏
    col_btn1, col_btn2, col_btn3, col_btn4 = st.columns(4)
    with col_btn1:
        if st.button("🎯 规划航线", use_container_width=True, type="primary"):
            plan_route()
            save_data()
            st.rerun()
    with col_btn2:
        if st.button("💾 保存数据", use_container_width=True):
            save_data()
            st.success("已保存")
    with col_btn3:
        if st.button("🗑️ 清空障碍物", use_container_width=True):
            st.session_state.obstacles = []
            st.session_state.current_route = []
            st.session_state.route_planned = False
            save_data()
            st.rerun()
    with col_btn4:
        if st.button("🔄 重置视图", use_container_width=True):
            st.rerun()
    
    st.divider()
    
    col1, col2 = st.columns([3, 1])
    
    with col1:
        st.subheader("🗺️ 地图（绘制多边形添加障碍物）")
        
        # 创建地图
        m = folium.Map(
            location=[32.2341, 118.7494],
            zoom_start=18,
            tiles="https://webst01.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}",
            attr="高德地图"
        )
        
        # 添加绘图工具
        draw = Draw(
            draw_options={
                "polygon": {
                    "allowIntersection": False,
                    "drawError": {"color": "#ff0000", "message": "不能交叉"},
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
        
        # 绘制起点
        folium.Marker(
            location=st.session_state.start_point,
            popup="🚁 起点 A",
            icon=folium.Icon(color="red", icon="play", prefix="fa")
        ).add_to(m)
        
        # 绘制终点
        folium.Marker(
            location=st.session_state.end_point,
            popup="🎯 终点 B",
            icon=folium.Icon(color="green", icon="flag-checkered", prefix="fa")
        ).add_to(m)
        
        # 绘制所有障碍物
        for i, obs in enumerate(st.session_state.obstacles):
            polygon = obs.get("polygon", [])
            height = obs.get("height", 10)
            name = obs.get("name", f"障碍物{i+1}")
            
            if polygon:
                # 判断是否需要绕行
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
        if st.session_state.current_route and st.session_state.route_planned:
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
        
        # 显示地图
        output = st_folium(m, width=850, height=550, returned_objects=["all_drawings"])
        
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
                                st.session_state.route_planned = False
                                st.session_state.current_route = []
                                save_data()
                                st.success(f"已添加: {obs_name}")
                                st.rerun()
                        with col_b:
                            if st.button("❌ 取消", use_container_width=True):
                                st.rerun()
    
    with col2:
        st.subheader("⚙️ 参数设置")
        
        # 起点
        with st.expander("📍 起点设置", expanded=False):
            new_start_lat = st.number_input("纬度", value=st.session_state.start_point[0], format="%.6f")
            new_start_lng = st.number_input("经度", value=st.session_state.start_point[1], format="%.6f")
            if st.button("更新起点"):
                st.session_state.start_point = (new_start_lat, new_start_lng)
                st.session_state.route_planned = False
                save_data()
                st.rerun()
        
        # 终点
        with st.expander("🏁 终点设置", expanded=False):
            new_end_lat = st.number_input("纬度", value=st.session_state.end_point[0], format="%.6f")
            new_end_lng = st.number_input("经度", value=st.session_state.end_point[1], format="%.6f")
            if st.button("更新终点"):
                st.session_state.end_point = (new_end_lat, new_end_lng)
                st.session_state.route_planned = False
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
            help="高于障碍物时可直飞"
        )
        if new_altitude != st.session_state.flight_altitude:
            st.session_state.flight_altitude = new_altitude
            st.session_state.route_planned = False
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
            st.session_state.route_planned = False
            save_data()
        
        st.divider()
        
        # 障碍物列表
        st.subheader(f"📦 障碍物列表 ({len(st.session_state.obstacles)})")
        
        if st.session_state.obstacles:
            for i, obs in enumerate(st.session_state.obstacles):
                col_a, col_b = st.columns([3, 1])
                with col_a:
                    st.write(f"**{obs.get('name', '未知')}**")
                    st.write(f"📏 {obs.get('height', 0)}m")
                with col_b:
                    if st.button("🗑️", key=f"del_{i}"):
                        st.session_state.obstacles.pop(i)
                        st.session_state.route_planned = False
                        st.session_state.current_route = []
                        save_data()
                        st.rerun()
                st.divider()
        else:
            st.info("📭 暂无障-碍物，请在地图上绘制多边形")
        
        # 航线信息
        if st.session_state.current_route and st.session_state.route_planned:
            st.divider()
            st.subheader("📊 航线信息")
            total_dist = 0
            for i in range(len(st.session_state.current_route) - 1):
                total_dist += calculate_distance(st.session_state.current_route[i], st.session_state.current_route[i+1])
            st.metric("总距离", f"{total_dist:.1f} m")
            st.metric("航点数", len(st.session_state.current_route))

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
        
        st.subheader("📊 状态")
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
            ax.plot(range(1, len(seq_list)+1), seq_list, marker='o', color='#2ecc71')
            ax.set_xlabel("次数")
            ax.set_ylabel("序列号")
            ax.set_title("心跳序列")
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
st.markdown("🚁 无人机航线规划系统 | 绘制多边形 → 点击「规划航线」→ 自动生成绕行路线")
