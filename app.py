import streamlit as st
import folium
from streamlit_folium import st_folium
from pyproj import Transformer

# --- 页面配置 ---
st.set_page_config(page_title="无人机轨迹可视化", layout="wide")

# --- 标题 ---
st.title("🚁 无人机飞行轨迹监控系统")

# --- 侧边栏：模拟数据输入 ---
st.sidebar.header("1. 输入无人机坐标数据")
st.sidebar.info("请输入 WGS84 坐标 (GPS原始坐标)，用逗号分隔。格式：经度,纬度")

# 默认的模拟数据（北京某地）
default_data = """116.397128,39.916527
116.397500,39.916800
116.398000,39.917000
116.398500,39.916500
116.399000,39.916000"""

# 获取用户输入，如果没有输入则使用默认数据
input_data = st.sidebar.text_area("粘贴坐标数据:", value=default_data, height=150)

# --- 数据处理函数 ---
def parse_coordinates(data_str):
    """将字符串解析为坐标列表"""
    coords = []
    lines = data_str.strip().split('\n')
    for line in lines:
        if ',' in line:
            try:
                lon, lat = map(float, line.split(','))
                coords.append((lat, lon)) # 注意：folium使用 (lat, lon)
            except ValueError:
                continue
    return coords

# --- 主程序逻辑 ---
coordinates = parse_coordinates(input_data)

if coordinates:
    st.success(f"成功解析到 {len(coordinates)} 个坐标点！")

    # 1. 创建地图对象，中心点设为第一个坐标
    m = folium.Map(location=coordinates[0], zoom_start=15)

    # 2. 添加轨迹线
    folium.PolyLine(
        coordinates,
        color="blue",
        weight=2.5,
        opacity=0.8,
        tooltip="飞行轨迹"
    ).add_to(m)

    # 3. 添加起点和终点标记
    folium.Marker(
        coordinates[0],
        popup="起点",
        icon=folium.Icon(color="green", icon="play")
    ).add_to(m)

    folium.Marker(
        coordinates[-1],
        popup="终点",
        icon=folium.Icon(color="red", icon="stop")
    ).add_to(m)

    # 4. 自动调整视野以适应所有点
    # 创建一个包含所有点的矩形框
    # 注意：folium的LatLngBounds需要 (lat, lon)
    # 这里我们手动计算边界或使用 fit_bounds
    m.fit_bounds(coordinates)

    # 5. 在页面上显示地图
    st_folium(m, width=700, height=500)

else:
    st.warning("请在左侧输入有效的坐标数据，格式：经度,纬度")

