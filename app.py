import streamlit as st
import folium
from streamlit_folium import st_folium

# --- 1. 页面基础设置 ---
st.set_page_config(page_title="无人机轨迹可视化", layout="wide")
st.title("🚁 无人机飞行轨迹监控系统")

# --- 2. 侧边栏：输入数据 ---
st.sidebar.header("1. 输入无人机坐标数据")
st.sidebar.info("请输入 WGS84 坐标，用逗号分隔。格式：经度,纬度")

# 默认数据
default_data = """116.397128,39.916527
116.397500,39.916800
116.398000,39.917000
116.398500,39.916500
116.399000,39.916000"""

# 文本输入框
input_data = st.sidebar.text_area("坐标数据:", value=default_data, height=200)

# --- 3. 数据处理逻辑 ---
# 初始化一个空列表存坐标
coordinates = []

# 按行分割数据
lines = input_data.strip().split('\n')

for line in lines:
    line = line.strip()
    if line:  # 如果行不为空
        try:
            # 按逗号分割经纬度
            lon, lat = map(float, line.split(','))
            coordinates.append((lat, lon)) # 注意：Folium 需要 (纬度, 经度) 的顺序
        except ValueError:
            st.error(f"数据格式错误: {line}")

# --- 4. 主界面显示逻辑 ---
if coordinates:
    st.success(f"成功解析到 {len(coordinates)} 个坐标点！")

    # --- 5. 绘制地图 ---
    # 取第一个点作为地图中心
    center_lat, center_lon = coordinates[0]

    # 创建地图对象
    m = folium.Map(location=[center_lat, center_lon], zoom_start=15)

    # 添加轨迹线
    folium.PolyLine(
        coordinates,
        color="blue",
        weight=2.5,
        opacity=1,
        tooltip="无人机轨迹"
    ).add_to(m)

    # 添加起点标记
    folium.Marker(
        coordinates[0],
        popup="起点",
        icon=folium.Icon(color="green", icon="play")
    ).add_to(m)

    # 添加终点标记
    folium.Marker(
        coordinates[-1],
        popup="终点",
        icon=folium.Icon(color="red", icon="stop")
    ).add_to(m)

    # --- 6. 在 Streamlit 中显示地图 ---
    st.subheader("🗺️ 实时轨迹地图")
    st_data = st_folium(m, width=700, height=500)

else:
    st.warning("请在左侧输入有效的坐标数据")
