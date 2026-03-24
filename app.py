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

input_data = st.sidebar.text_area("粘贴坐标数据：", value=default_data, height=200)

# --- 数据处理函数 ---
@st.cache_data
def process_data(data_str):
    """
    解析文本数据，提取经纬度
    """
    if not data_str.strip():
        return []

    lines = data_str.strip().split('\n')
    points = []

    for line in lines:
        parts = line.split(',')
        if len(parts) == 2:
            try:
                lng = float(parts[0].strip())
                lat = float(parts[1].strip())
                points.append((lat, lng)) # Folium 需要 (lat, lng) 格式
            except ValueError:
                continue
    return points

# --- 主程序逻辑 ---
coordinates = process_data(input_data)

if coordinates:
    # 创建地图，中心点设为第一个坐标
    m = folium.Map(location=coordinates[0], zoom_start=15)

    # 绘制轨迹线
    folium.PolyLine(
        coo
