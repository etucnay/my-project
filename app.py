import streamlit as st
import pandas as pd
import plotly.graph_objects as go
import pydeck as pdk
import time
from datetime import datetime

# ==================== 页面配置 ====================
st.set_page_config(layout="wide", page_title="无人机地面站")
st.sidebar.title("导航")
page = st.sidebar.radio("功能页面", ["航线规划", "飞行监控"])

# ==================== 初始化 session_state ====================
if "point_a" not in st.session_state:
    # 南京科技职业学院附近坐标 (GCJ-02)
    st.session_state.point_a = (32.2322, 118.749)   # (纬度, 经度)
    st.session_state.point_b = (32.2343, 118.749)
    st.session_state.obstacles = [
        (32.2325, 118.7487, 35),
        (32.2329, 118.7485, 45),
        (32.2333, 118.7488, 30),
        (32.2337, 118.7490, 50),
        (32.2339, 118.7493, 40),
    ]
if "coord_system" not in st.session_state:
    st.session_state.coord_system = "GCJ-02 (高德/百度)"
if "running" not in st.session_state:
    st.session_state.running = False
if "heartbeat_data" not in st.session_state:
    st.session_state.heartbeat_data = []
if "last_received_time" not in st.session_state:
    st.session_state.last_received_time = time.time()

# ==================== 地图绘制函数（航线规划使用）====================
def draw_3d_map(point_a, point_b, obstacles):
    """绘制3D地图，包含起点A、终点B和障碍物"""
    # 起点A（绿色）
    a_layer = pdk.Layer(
        "ScatterplotLayer",
        data=[{"lat": point_a[0], "lon": point_a[1]}],
        get_position="[lon, lat]",
        get_color="[0, 255, 0, 200]",
        get_radius=100,
        pickable=True,
    )
    # 终点B（红色）
    b_layer = pdk.Layer(
        "ScatterplotLayer",
        data=[{"lat": point_b[0], "lon": point_b[1]}],
        get_position="[lon, lat]",
        get_color="[255, 0, 0, 200]",
        get_radius=100,
        pickable=True,
    )
    # 障碍物（橙色柱状）
    obs_data = [{"lat": o[0], "lon": o[1], "height": o[2]} for o in obstacles]
    obstacle_layer = pdk.Layer(
        "ColumnLayer",
        data=obs_data,
        get_position="[lon, lat]",
        get_elevation="height",
        elevation_scale=80,
        radius=25,
        get_fill_color="[255, 165, 0, 200]",
        pickable=True,
    )
    # 视图居中，倾斜视角增强3D效果
    center_lat = (point_a[0] + point_b[0]) / 2
    center_lon = (point_a[1] + point_b[1]) / 2
    view_state = pdk.ViewState(
        latitude=center_lat,
        longitude=center_lon,
        zoom=14,
        pitch=60,
        bearing=0,
    )
    r = pdk.Deck(
        layers=[a_layer, b_layer, obstacle_layer],
        initial_view_state=view_state,
        tooltip={"text": "高度: {height} m" if "height" in obs_data[0] else "{position}"},
    )
    st.pydeck_chart(r)

# ==================== 心跳监控函数（飞行监控使用）====================
def heartbeat_monitor():
    """飞行监控页面：心跳模拟与可视化"""
    st.subheader("飞行监控")
    data = st.session_state.heartbeat_data
    placeholder = st.empty()          # 用于动态显示表格
    chart_placeholder = st.empty()    # 用于动态显示图表

    # 模拟10次心跳（可根据需要修改循环次数）
    for i in range(1, 11):
        current_time = time.time()
        # 添加一条心跳记录
        data.append({
            "序号": len(data) + 1,
            "时间": datetime.now().strftime("%H:%M:%S"),
            "接收": 1
        })
        # 显示最近10条数据
        df = pd.DataFrame(data)
        placeholder.dataframe(df.tail(10))

        # 绘制折线图（心跳序号随时间变化）
        fig = go.Figure()
        fig.add_trace(go.Scatter(
            x=list(range(1, len(data)+1)),
            y=[1]*len(data),
            mode='lines+markers',
            name='心跳状态'
        ))
        fig.update_layout(
            title="心跳序号随时间变化",
            xaxis_title="序号",
            yaxis_title="是否接收"
        )
        chart_placeholder.plotly_chart(fig, use_container_width=True)

        # 掉线检测：如果距离上次接收超过3秒，则报警
        if current_time - st.session_state.last_received_time > 3:
            st.error("连接超时！")
        st.session_state.last_received_time = current_time

        time.sleep(1)   # 模拟每秒一次心跳
    st.success("演示结束")

# ==================== 页面路由 ====================
if page == "航线规划":
    st.header("航线规划")
    # 坐标系选择
    coord_system = st.radio(
        "坐标系统",
        ["GCJ-02 (高德/百度)", "WGS-84 (GPS)"],
        horizontal=True,
        key="coord_radio"
    )
    st.session_state.coord_system = coord_system
    st.markdown(f"### 当前坐标系：{coord_system}")

    col1, col2 = st.columns(2)
    with col1:
        st.subheader("起点 A")
        lat_a = st.number_input(
            f"纬度 ({coord_system})",
            value=st.session_state.point_a[0],
            step=0.0001,
            format="%.6f",
            key="lat_a"
        )
        lon_a = st.number_input(
            f"经度 ({coord_system})",
            value=st.session_state.point_a[1],
            step=0.0001,
            format="%.6f",
            key="lon_a"
        )
        if st.button("设置A点"):
            st.session_state.point_a = (lat_a, lon_a)
            st.success("A点已设置")
    with col2:
        st.subheader("终点 B")
        lat_b = st.number_input(
            f"纬度 ({coord_system})",
            value=st.session_state.point_b[0],
            step=0.0001,
            format="%.6f",
            key="lat_b"
        )
        lon_b = st.number_input(
            f"经度 ({coord_system})",
            value=st.session_state.point_b[1],
            step=0.0001,
            format="%.6f",
            key="lon_b"
        )
        if st.button("设置B点"):
            st.session_state.point_b = (lat_b, lon_b)
            st.success("B点已设置")

    st.slider("设定飞行高度 (m)", 0, 200, 50, key="flight_height")
    # 调用地图绘制函数（显示在航线规划页面）
    draw_3d_map(
        st.session_state.point_a,
        st.session_state.point_b,
        st.session_state.obstacles
    )
    st.caption("提示：绿色为起点A，红色为终点B，橙色柱体为障碍物。鼠标拖拽可旋转视角。")

else:   # 页面为“飞行监控”
    heartbeat_monitor()
