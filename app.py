import streamlit as st
import folium
from streamlit_folium import st_folium
import coordtransform# 刚才安装的坐标系转换库

# --- 页面配置 ---
st.set_page_config(page_title="无人机轨迹可视化", layout="wide")

# --- 标题 ---
st.title("🚁 无人机飞行轨迹监控系统")

# --- 侧边栏：输入数据 ---
st.sidebar.header("1. 坐标系设置")
coord_system = st.sidebar.radio(
    "选择输入坐标系:",
    ("WGS-84 (GPS原始坐标)", "GCJ-02 (高德/腾讯坐标)")
)

st.sidebar.header("2. 输入无人机坐标数据")
st.sidebar.info("请输入坐标，用逗号分隔。格式：经度,纬度")

# 默认数据（模拟北京某地）
default_data = """116.397128,39.916527
116.397500,39.916800
116.398000,39.917000
116.398500,39.916500
116.399000,39.916000"""

input_data = st.sidebar.text_area("坐标数据:", value=default_data, height=200)

# --- 3. 数据处理与转换 ---
coordinates = []
if input_data:
    lines = input_data.strip().split('\n')
    for line in lines:
        line = line.strip()
        if line:
            try:
                lon, lat = map(float, line.split(','))

                # 如果是 WGS-84，就转成 GCJ-02
                # 如果本来就是 GCJ-02，就不转
                if "WGS" in coord_system:
                    gcj_lon, gcj_lat = coordtransform.wgs84_to_gcj02(lon, lat)
                else:
                    gcj_lon, gcj_lat = lon, lat

                coordinates.append((gcj_lat, gcj_lon)) # Folium 需要 (纬度, 经度) 的顺序

            except ValueError:
                st.sidebar.warning(f"无法解析这一行: {line}")

# --- 4. 绘制地图 ---
st.subheader("🗺️ 实时轨迹地图")

if coordinates:
    # 取第一个点作为地图中心
    center_lat, center_lon = coordinates[0]

    # 创建地图对象
    # tiles 参数控制地图样式：
    # 'OpenStreetMap' (默认，国内经常加载不出)
    # 'CartoDB Positron' (白底图，很清晰)
    # 'Stamen Terrain' (地形图)
    # 为了模拟你图片里的卫星图效果，我们尝试用 'CartoDB Positron' 或者保持默认
    m = folium.Map(
        location=[center_lat, center_lon],
        zoom_start=16,
        tiles='CartoDB Positron' # 这里可以换成 'OpenStreetMap' 试试
    )

    # 添加轨迹线
    folium.PolyLine(
        coordinates,
        color="blue",
        weight=5,
        opacity=0.8,
        tooltip="无人机轨迹"
    ).add_to(m)

    # 添加起点和终点标记
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

    # 在 Streamlit 中显示地图
    # 注意：height 和 width 必须设置，否则可能不显示
    st_data = st_folium(m, width=700, height=500)

else:
    st.info("请在左侧输入有效的坐标数据")
