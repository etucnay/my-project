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
from shapely.ops import unary_union

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
if "current_route_mode" not in st.session_state:
    st.session_state.current_route_mode = "best" # 默认最佳航线

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
        "current_route_mode": st.session_state.current_route_mode,
        "save_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "version": "v16.0_最终零报错版"
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
        st.session_state.current_route_mode = data.get("current_route_mode", "best")

# 启动时自动加载配置
load_obstacles()

# ====================== 航线规划核心算法 ======================
def calculate_safe_path(start, end, obstacles, safety_radius, mode="best"):
    """
    计算安全航线
    :param start: (lat, lng) 起点
    :param end: (lat, lng) 终点
    :param obstacles: list of {"coords": [(lat,lng),...], "height": int} 障碍物列表
    :param safety_radius: 安全缓冲半径(米)
    :param mode: left(向左绕行), right(向右绕行), best(最佳航线)
    :return: list of (lat,lng) 航线点
    """
    # 1. 基础参数计算
    start_lat, start_lng = start
    end_lat, end_lng = end
    
    # 2. 检查是否需要绕行：飞行高度 > 所有障碍物最高高度 → 直飞
    max_obstacle_height = max([obs["height"] for obs in obstacles]) if obstacles else 0
    if st.session_state.flight_altitude > max_obstacle_height:
        return [start, end]
    
    # 3. 计算方向向量（经纬度转平面近似，小范围场景适用）
    # 1度纬度≈111320米，1度经度≈111320*cos(纬度)米
    lat_mid = (start_lat + end_lat) / 2
    lng_per_m = 1 / (111320 * np.cos(np.radians(lat_mid)))
    lat_per_m = 1 / 111320
    
    dx = end_lng - start_lng
    dy = end_lat - start_lat
    dist = np.sqrt(dx**2 + dy**2)
    if dist < 1e-6:
        return [start, end]
    
    # 单位方向向量
    ux = dx / dist
    uy = dy / dist
    # 单位法向量（左法向量：垂直于方向向量，逆时针90度）
    nx = -uy
    ny = ux
    
    # 4. 安全偏移量（米转经纬度）
    offset_m = safety_radius * 1.5  # 额外50%缓冲，避免擦碰
    offset_lat = offset_m * lat_per_m
    offset_lng = offset_m * lng_per_m
    
    # 5. 根据模式生成绕行路径
    if mode == "left":
        # 向左绕行：沿障碍物左侧（法向量正方向）
        via1 = (start_lat + ny * offset_lat, start_lng + nx * offset_lng)
        via2 = (end_lat + ny * offset_lat, end_lng + nx * offset_lng)
        return [start, via1, via2, end]
    elif mode == "right":
        # 向右绕行：沿障碍物右侧（法向量负方向）
        via1 = (start_lat - ny * offset_lat, start_lng - nx * offset_lng)
        via2 = (end_lat - ny * offset_lat, end_lng - nx * offset_lng)
        return [start, via1, via2, end]
    else: # "best" 最佳航线：取最短路径（左右偏移量减半，综合最优）
        via1 = (start_lat + ny * offset_lat * 0.5, start_lng + nx * offset_lng * 0.5)
        via2 = (end_lat + ny * offset_lat * 0.5, end_lng + nx * offset_lat * 0.5)
        return [start, via1, via2, end]

# ====================== 分页设计 ======================
tab1, tab2 = st.tabs(["🗺️ 地图与航线规划", "📡 飞行监控"])

# ====================== 标签页1：地图与障碍物管理 ======================
with tab1:
    col1, col2 = st.columns([3, 1])

    # 左侧：卫星地图
    with col1:
        st.subheader("🗺️ 卫星地图 (高德)")
        
        # 初始化地图（高德卫星瓦片，国内加载无空白）
        m = folium.Map(
            location=[32.2341, 118.7494],
            zoom_start=18,
            tiles="https://webst01.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}",
            attr="© 高德地图 | © OpenStreetMap contributors"
        )

        # 多边形绘制工具（仅保留多边形，用于圈选障碍物）
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

        # 绘制A/B点（起点红标，终点绿标）
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

        # 绘制已部署障碍物（带高度弹窗）
        for obs in st.session_state.deployed_obstacles:
            folium.Polygon(
                locations=obs["coords"],
                color="red",
                weight=2,
                fill=True,
                fill_color="red",
                fill_opacity=0.5,
                popup=f"障碍物{obs['id']}\n高度: {obs['height']}m\n创建时间: {obs['create_time']}"
            ).add_to(m)

        # 计算并绘制航线
        A = st.session_state.start_point
        B = st.session_state.end_point
        route_points = calculate_safe_path(
            A, B, 
            st.session_state.deployed_obstacles, 
            st.session_state.safety_radius,
            st.session_state.current_route_mode
        )
        
        # 根据航线类型设置样式：直飞蓝色，绕行橙色虚线
        if len(route_points) == 2:
            route_color = "blue"
            route_weight = 4
            route_dash = None
            route_popup = "A→B 直飞航线（高度足够飞越障碍物）"
        else:
            route_color = "orange"
            route_weight = 5
            route_dash = "5, 10"
            mode_name = {"left":"向左绕行", "right":"向右绕行", "best":"最佳航线"}[st.session_state.current_route_mode]
            route_popup = f"A→B {mode_name}航线（障碍物高度高于飞行高度）"
        
        folium.PolyLine(
            locations=route_points,
            color=route_color,
            weight=route_weight,
            opacity=0.8,
            dash_array=route_dash,
            popup=route_popup
        ).add_to(m)

        # 渲染地图（固定key，避免刷新闪烁）
        map_data = st_folium(m, width=800, height=650, key="fixed_map_key")

        # 监听绘制事件，添加障碍物（带高度设置）
        if map_data and map_data.get("last_active_drawing"):
            drawing = map_data["last_active_drawing"]
            if drawing["geometry"]["type"] == "Polygon":
                # 转换坐标格式（folium返回[lng,lat]，转[lat,lng]）
                coords = [(lat, lng) for lng, lat in drawing["geometry"]["coordinates"][0]]
                # 右侧弹出高度输入框
                with col2:
                    st.subheader("🏗️ 新障碍物设置")
                    new_height = st.number_input(
                        "设置障碍物高度 (米)", 
                        min_value=1, 
                        max_value=500, 
                        value=10, 
                        key="new_obstacle_height",
                        help="请输入障碍物的实际高度，用于航线避障判断"
                    )
                    if st.button("✅ 确认添加障碍物", use_container_width=True, type="primary"):
                        # 生成新障碍物对象
                        new_obs = {
                            "id": len(st.session_state.obstacles) + 1,
                            "coords": coords,
                            "height": new_height,
                            "create_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                        }
                        # 避免重复添加
                        if new_obs not in st.session_state.obstacles:
                            st.session_state.obstacles.append(new_obs)
                            save_obstacles()
                            st.success(f"✅ 障碍物 {new_obs['id']} 添加成功！")
                            # 清除绘制状态，防止重复添加
                            st.session_state.pop("last_active_drawing", None)
                            st.rerun()

    # 右侧：控制面板
    with col2:
        # 1. A/B点设置
        st.subheader("📍 A/B点坐标设置")
        st.caption("坐标系：GCJ-02（高德/国内地图标准）")
        
        # A点
        a_lat = st.number_input(
            "A纬度", 
            value=st.session_state.start_point[0], 
            format="%.6f", 
            step=0.0001,
            key="a_lat_input"
        )
        a_lng = st.number_input(
            "A经度", 
            value=st.session_state.start_point[1], 
            format="%.6f", 
            step=0.0001,
            key="a_lng_input"
        )
        if st.button("✅ 确认A点", use_container_width=True):
            st.session_state.start_point = (a_lat, a_lng)
            save_obstacles()
            st.success("A点坐标已保存！")

        st.divider()

        # B点
        b_lat = st.number_input(
            "B纬度", 
            value=st.session_state.end_point[0], 
            format="%.6f", 
            step=0.0001,
            key="b_lat_input"
        )
        b_lng = st.number_input(
            "B经度", 
            value=st.session_state.end_point[1], 
            format="%.6f", 
            step=0.0001,
            key="b_lng_input"
        )
        if st.button("✅ 确认B点", use_container_width=True):
            st.session_state.end_point = (b_lat, b_lng)
            save_obstacles()
            st.success("B点坐标已保存！")

        st.divider()

        # 2. 飞行参数配置（核心需求）
        st.subheader("🛫 飞行参数配置")
        st.session_state.flight_altitude = st.number_input(
            "无人机飞行高度 (m)", 
            min_value=1.0, 
            max_value=100.0, 
            value=st.session_state.flight_altitude, 
            step=0.5,
            help="当飞行高度 > 障碍物最高高度时，无人机将直飞；否则自动绕行"
        )
        st.session_state.safety_radius = st.number_input(
            "安全绕行半径 (m)", 
            min_value=1.0, 
            max_value=50.0, 
            value=st.session_state.safety_radius, 
            step=0.5,
            help="无人机绕行障碍物时的安全缓冲距离，默认5米"
        )

        st.divider()

        # 3. 航线规划模式（3种选项，核心需求）
        st.subheader("🧭 航线规划模式")
        mode = st.radio(
            "选择绕行策略",
            ("left", "right", "best"),
            format_func=lambda x: {
                "left": "🔴 向左绕行", 
                "right": "🟢 向右绕行", 
                "best": "🔵 最佳航线"
            }[x],
            horizontal=True,
            key="route_mode_radio"
        )
        st.session_state.current_route_mode = mode
        
        if st.button("🔄 重新规划航线", use_container_width=True, type="primary"):
            save_obstacles()
            st.rerun()

        st.divider()

        # 4. 障碍物管理按钮（4个，持久化功能）
        st.subheader("🚀 障碍物配置管理")
        col_btn1, col_btn2, col_btn3, col_btn4 = st.columns(4)
        with col_btn1:
            if st.button("💾 永久保存", key="save_obs", use_container_width=True, type="primary"):
                save_obstacles()
                st.success(f"已保存 {len(st.session_state.obstacles)} 个障碍物！")
        with col_btn2:
            if st.button("📂 加载配置", key="load_obs", use_container_width=True):
                load_obstacles()
                st.success(f"已加载 {len(st.session_state.obstacles)} 个障碍物！")
                st.rerun()
        with col_btn3:
            if st.button("🗑️ 清空全部", key="clear_obs", use_container_width=True):
                st.session_state.obstacles = []
                st.session_state.deployed_obstacles = []
                save_obstacles()
                st.success("已清空所有障碍物！")
                st.rerun()
        with col_btn4:
            if st.button("🚀 一键部署", key="deploy_obs", use_container_width=True, type="primary"):
                st.session_state.deployed_obstacles = st.session_state.obstacles.copy()
                save_obstacles()
                st.success(f"已部署 {len(st.session_state.deployed_obstacles)} 个障碍物！")
                st.rerun()

        st.divider()

        # 5. 状态显示
        st.subheader("📊 当前状态")
        st.info(f"已绘制障碍物：{len(st.session_state.obstacles)} 个\n已部署障碍物：{len(st.session_state.deployed_obstacles)} 个")
        st.info(f"飞行高度：{st.session_state.flight_altitude}m | 安全半径：{st.session_state.safety_radius}m")
        st.code(CONFIG_FILE, language="text")

# ====================== 标签页2：飞行监控 ======================
with tab2:
    st.subheader("📡 无人机飞行监控")

    # 控制按钮区
    col_ctrl1, col_ctrl2, col_ctrl3 = st.columns(3)
    with col_ctrl1:
        if st.button("▶️ 开始模拟心跳", key="start_hb", use_container_width=True, type="primary"):
            st.session_state.heartbeat_running = True
            st.rerun()
    with col_ctrl2:
        if st.button("⏸️ 停止模拟", key="stop_hb", use_container_width=True):
            st.session_state.heartbeat_running = False
            st.rerun()
    with col_ctrl3:
        if st.button("🗑️ 清除历史数据", key="clear_hb", use_container_width=True):
            st.session_state.heartbeat_history = []
            st.session_state.last_heartbeat_time = "--:--:--.---"
            st.rerun()

    st.divider()

    # 状态信息区
    col_stat1, col_stat2, col_stat3 = st.columns(3)
    with col_stat1:
        st.metric("最新心跳序列号", len(st.session_state.heartbeat_history))
    with col_stat2:
        st.success("✅ 心跳正常接收中" if st.session_state.heartbeat_running else "⏸️ 心跳已停止")
    with col_stat3:
        st.metric("最后心跳时间", st.session_state.last_heartbeat_time)

    st.divider()

    # 心跳数据生成（模拟实时接收）
    if st.session_state.heartbeat_running:
        new_seq = len(st.session_state.heartbeat_history)
        new_time = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        st.session_state.last_heartbeat_time = new_time
        st.session_state.heartbeat_history.append({
            "序列号": new_seq,
            "接收时间": new_time,
            "状态": "正常"
        })

    # 心跳数据表
    st.subheader("📋 心跳数据记录表")
    if st.session_state.heartbeat_history:
        df_hb = pd.DataFrame(st.session_state.heartbeat_history)
        st.dataframe(df_hb, use_container_width=True, hide_index=True)
    else:
        st.info("📊 暂无心跳数据，请点击「开始模拟心跳」")

    st.divider()

    # 心跳时间线图
    st.subheader("📈 心跳序列时间轨迹图")
    if len(st.session_state.heartbeat_history) > 0:
        seq_list = [item["序列号"] for item in st.session_state.heartbeat_history]
        time_list = [datetime.strptime(item["接收时间"], "%H:%M:%S.%f") for item in st.session_state.heartbeat_history]
        
        # 解决中文乱码问题
        plt.rcParams["font.sans-serif"] = ["SimHei"]
        plt.rcParams["axes.unicode_minus"] = False
        
        fig, ax = plt.subplots(figsize=(12, 3))
        ax.plot(time_list, seq_list, color="#2ecc71", linewidth=2, marker='o', markersize=4)
        ax.set_title("无人机心跳序列时间轨迹", fontsize=12, fontweight="bold")
        ax.set_xlabel("接收时间", fontsize=10)
        ax.set_ylabel("心跳序列号", fontsize=10)
        ax.grid(True, alpha=0.3)
        # 自动旋转x轴标签，避免重叠
        fig.autofmt_xdate()
        st.pyplot(fig)
    else:
        st.info("📈 启动心跳模拟后自动生成时间线图")

# ====================== 页脚版权 ======================
st.markdown("---")
st.markdown("Leaflet | © 高德地图 | © OpenStreetMap contributors | 无人机航线规划系统 v16.0")
