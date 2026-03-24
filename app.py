import streamlit as st
import folium
from streamlit_folium import st_folium
import coordtransform # 用于坐标转换

# --- 页面配置 ---
st.set_page_config(page_title="无人机航线规划", layout="wide")

# --- 侧边栏/控制面板 (对应截图左侧) ---
st.sidebar.header("⚙️ 控制面板")

# 1. 坐标系选择
coord_system = st.sidebar.radio(
    "输入坐标系",
    ("WGS-84 (GPS原始)", "GCJ-02 (高德/百度)")
)

# 2. A点/B点 输入框
st.sidebar.subheader("📍 起点 A")
lat_a = st.sidebar.number_input("纬度 A", value=32.2322, format="%.6f")
lng_a = st.sidebar.number_input("经度 A", value=118.7490, format="%.6f")

st.sidebar.subheader("🏁 终点 B")
lat_b = st.sidebar.number_input("纬度 B", value=32.2343, format="%.6f")
lng_b = st.sidebar.number_input("经度 B", value=118.7495, format="%.6f")

# --- 地图初始化 (对应截图右侧) ---
st.subheader("🗺️ 航线规划地图")

# 默认中心点设为A点
center_lat = lat_a
center_lng = lng_a

# 创建 Folium 地图对象
# tiles参数可以是 'OpenStreetMap', 'CartoDB Positron' 等，如果要卫星图需使用自定义URL
m = folium.Map(location=[center_lat, center_lng], zoom_start=18, tiles=None)

# 添加高德卫星图层 (解决"显示二维地图样貌/卫星图"的需求)
# 注意：这里使用了高德的卫星图URL
folium.TileLayer(
    tiles='https://webrd0{s}.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=6&x={x}&y={y}&z={z}',
    attr='高德地图',
    name='卫星图',
    overlay=True,
    control=True,
    subdomains='1234'
).add_to(m)

# 添加路网注记 (让地图上有路名文字)
folium.TileLayer(
    tiles='https://webst0{s}.is.autonavi.com/appmaptile?style=8&x={x}&y={y}&z={z}',
    attr='高德地图',
    name='路网',
    overlay=True,
    control=True,
    subdomains='1234'
).add_to(m)

# --- 核心功能：坐标转换与绘图 ---

def convert_coords(lat, lng, target):
    """转换坐标系的辅助函数"""
    if target == "WGS-84 (GPS原始)":
        # 如果输入是WGS84，地图用的是GCJ02(高德)，需要转换
        # 注意 gcoord 的数组顺序是 [经度, 纬度]
        result = gcoord.transform(
            [lng, lat],
            gcoord.WGS84,
            gcoord.GCJ02
        )
        return result[1], result[0] # 返回 lat, lng
    else:
        # 如果输入已经是GCJ02，直接返回
        return lat, lng

# 1. 处理 A 点
if st.sidebar.button("设置A点"):
    final_lat_a, final_lng_a = convert_coords(lat_a, lng_a, coord_system)
    folium.Marker(
        [final_lat_a, final_lng_a],
        popup="起点 A",
        icon=folium.Icon(color='green', icon='play')
    ).add_to(m)
    # 飞线逻辑可以在这里加
    st.sidebar.success(f"A点已设置: {final_lat_a}, {final_lng_a}")

# 2. 处理 B 点
if st.sidebar.button("设置B点"):
    final_lat_b, final_lng_b = convert_coords(lat_b, lng_b, coord_system)
    folium.Marker(
        [final_lat_b, final_lng_b],
        popup="终点 B",
        icon=folium.Icon(color='red', icon='stop')
    ).add_to(m)
    st.sidebar.success(f"B点已设置: {final_lat_b}, {final_lng_b}")


# 3. 绘制障碍物 (示例：硬编码几个障碍物，或者通过点击地图获取)
# 这里演示如何在AB连线中间画个圈作为障碍物示例
# 实际项目中，你可能需要让用户点击地图来添加，或者读取心跳包中的障碍物数据
obstacles = [
    [32.2330, 118.7492], # 示例障碍物坐标
]

for obs in obstacles:
    # 如果障碍物坐标也是WGS84，记得也要转换！
    folium.Circle(
        radius=30, # 半径30米
        location=obs,
        popup="障碍物",
        color="red",
        fill=True,
        fillColor="red"
    ).add_to(m)

# --- 显示地图 ---
# st_folium 是实现交互的关键
output = st_folium(m, width=700, height=500)

# --- 进阶：点击地图获取坐标 (方便画障碍物) ---
# 如果用户点击了地图，显示坐标
if output['last_clicked']:
    click_lat = output['last_clicked']['lat']
    click_lng = output['last_clicked']['lng']
    st.info(f"你点击的位置坐标: {click_lat}, {click_lng}")
    # 这里可以加个按钮“在此处添加障碍物”
