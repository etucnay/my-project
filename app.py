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
    解析文本数据，并使用 pyproj 将 WGS84 坐标转换为 Web Mercator (用于地图显示)
    """
    if not data_str.strip():
        return []

    lines = data_str.strip().split('\n')
    points = []

    # 定义转换器：从 WGS84 (GPS标准) 转换为 Web Mercator (地图标准)
    # EPSG:4326 是经纬度标准
    # EPSG:3857 是 Google Maps / Leaflet / Folium 使用的投影标准
    transformer = Transformer.from_crs("EPSG:4326", "EPSG:3857", always_xy=True)

    try:
        for i, line in enumerate(lines):
            parts = line.split(',')
            if len(parts) >= 2:
                # 尝试读取经度和纬度
                lon = float(parts[0].strip())
                lat = float(parts[1].strip())

                # 进行坐标转换
                x, y = transformer.transform(lon, lat)

                points.append({
                    "id": i + 1,
                    "original_lon": lon,
                    "original_lat": lat,
                    "map_x": x,
                    "map_y": y
                })
        return points
    except Exception as e:
        st.error(f"数据解析错误: {e}")
        return []

# --- 主程序逻辑 ---

# 1. 处理数据
trajectory_points = process_data(input_data)

if trajectory_points:
    # 2. 获取第一个点作为地图中心
    first_point = trajectory_points[0]

    # 3. 创建地图对象
    # 注意：folium 的 Location 需要传入原始的经纬度 (lat, lon)，而不是转换后的 x,y
    m = folium.Map(
        location=[first_point['original_lat'], first_point['original_lon']],
        zoom_start=16,
        tiles='OpenStreetMap' # 也可以使用 'Stamen Terrain' 或 'CartoDB Positron'
    )

    # 4. 提取轨迹线坐标 (Folium 需要原始的经纬度列表)
    # 格式必须是 [[lat, lon], [lat, lon], ...]
    route_coords = [[p['original_lat'], p['original_lon']] for p in trajectory_points]

    # 5. 在地图上画线
    folium.PolyLine(
        route_coords,
        color="blue",
        weight=5,
        opacity=0.8,
        tooltip="无人机飞行轨迹"
    ).add_to(m)

    # 6. 标记起点和终点
    folium.Marker(
        route_coords[0],
        popup="起飞点",
        icon=folium.Icon(color="green", icon="play")
    ).add_to(m)

    folium.Marker(
        route_coords[-1],
        popup="降落点",
        icon=folium.Icon(color="red", icon="stop")
    ).add_to(m)

    # 7. 显示地图
    st.subheader("🗺️ 实时轨迹地图")
    st_folium(m, width=700, height=500)

    # 8. 显示数据表格
    with st.expander("查看坐标转换详情"):
        st.dataframe(trajectory_points)

else:
    st.warning("请在左侧输入有效的坐标数据。")
