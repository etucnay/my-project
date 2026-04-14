import streamlit as st
import folium
from streamlit_folium import st_folium
import math
from typing import List, Dict, Tuple

# ================== 页面配置 ==================
st.set_page_config(page_title="无人机航线规划", layout="wide")
st.title("✈️ 无人机航线规划与障碍物管理系统")

# ================== 初始化 session_state ==================
if "obstacles" not in st.session_state:
    st.session_state.obstacles = []          # 每个元素: {"lat": float, "lon": float, "height": float}
if "start_point" not in st.session_state:
    st.session_state.start_point = {"lat": 39.9042, "lon": 116.4074}   # 默认北京天安门
if "end_point" not in st.session_state:
    st.session_state.end_point = {"lat": 39.9142, "lon": 116.4174}
if "flight_height" not in st.session_state:
    st.session_state.flight_height = 10.0    # 米
if "safe_radius" not in st.session_state:
    st.session_state.safe_radius = 5.0       # 米
if "route_mode" not in st.session_state:
    st.session_state.route_mode = "智能最优"
if "planned_route" not in st.session_state:
    st.session_state.planned_route = []      # 路径点列表 [(lat, lon), ...]
if "map_center" not in st.session_state:
    st.session_state.map_center = [39.9042, 116.4074]

# ================== 辅助函数 ==================
def haversine(lat1, lon1, lat2, lon2):
    """计算两点间距离（米）"""
    R = 6371000
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    a = math.sin(delta_phi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(delta_lambda/2)**2
    c = 2*math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c

def point_to_segment_distance(lat, lon, lat1, lon1, lat2, lon2):
    """点到线段的最短距离（米）"""
    # 投影参数
    dx = lon2 - lon1
    dy = lat2 - lat1
    if dx == 0 and dy == 0:
        return haversine(lat, lon, lat1, lon1)
    t = ((lon - lon1)*dx + (lat - lat1)*dy) / (dx*dx + dy*dy)
    if t < 0:
        closest = (lat1, lon1)
    elif t > 1:
        closest = (lat2, lon2)
    else:
        closest = (lat1 + t*dy, lon1 + t*dx)
    return haversine(lat, lon, closest[0], closest[1])

def is_obstacle_blocking(obs: Dict, start: Tuple, end: Tuple, safe_radius: float, flight_height: float) -> bool:
    """
    判断障碍物是否阻挡航线（考虑安全半径和飞行高度）
    如果飞行高度 > 障碍物高度，则忽略该障碍物
    """
    if flight_height > obs["height"]:
        return False
    obs_center = (obs["lat"], obs["lon"])
    dist = point_to_segment_distance(obs_center[0], obs_center[1], start[0], start[1], end[0], end[1])
    return dist < safe_radius

def calculate_safe_path(start: Dict, end: Dict, obstacles: List, flight_height: float, safe_radius: float, mode: str) -> List[Tuple]:
    """
    规划避障路径（简化版：仅处理第一个相交障碍物，按模式绕行）
    返回路径点列表（包含起点和终点）
    """
    start_pt = (start["lat"], start["lon"])
    end_pt = (end["lat"], end["lon"])
    
    # 过滤出高度威胁的障碍物
    threatening = [obs for obs in obstacles if flight_height <= obs["height"]]
    if not threatening:
        # 无威胁，直接直线
        return [start_pt, end_pt]
    
    # 找到与直线段距离最近的障碍物（且距离小于安全半径）
    min_dist = float("inf")
    block_obs = None
    for obs in threatening:
        obs_center = (obs["lat"], obs["lon"])
        dist = point_to_segment_distance(obs_center[0], obs_center[1], start_pt[0], start_pt[1], end_pt[0], end_pt[1])
        if dist < safe_radius and dist < min_dist:
            min_dist = dist
            block_obs = obs
    
    if block_obs is None:
        return [start_pt, end_pt]
    
    # 计算绕行点（垂直于航线方向偏移）
    # 航线方向向量
    dx = end_pt[1] - start_pt[1]
    dy = end_pt[0] - start_pt[0]
    length = math.hypot(dx, dy)
    if length == 0:
        return [start_pt, end_pt]
    # 单位方向向量
    ux = dx / length
    uy = dy / length
    # 垂直于航线的单位向量（顺时针旋转90度）
    perp_x = uy
    perp_y = -ux
    
    obs_center = (block_obs["lat"], block_obs["lon"])
    # 障碍物到直线的垂足
    t = ((obs_center[1] - start_pt[1])*dx + (obs_center[0] - start_pt[0])*dy) / (dx*dx + dy*dy)
    foot_lat = start_pt[0] + t*dy
    foot_lon = start_pt[1] + t*dx
    
    # 偏移距离 = 安全半径 + 10%（确保避开）
    offset_dist = safe_radius * 1.2
    
    # 根据模式选择偏移方向
    if mode == "左侧绕行":
        # 使用 perp 正方向
        offset_lat = foot_lat + perp_y * offset_dist / 111320  # 纬度偏移近似
        offset_lon = foot_lon + perp_x * offset_dist / (111320 * math.cos(math.radians(foot_lat)))
    elif mode == "右侧绕行":
        # 使用 perp 反方向
        offset_lat = foot_lat - perp_y * offset_dist / 111320
        offset_lon = foot_lon - perp_x * offset_dist / (111320 * math.cos(math.radians(foot_lat)))
    else:  # 智能最优：选择偏移后路径更短的一侧
        # 计算左右两个候选点
        left_lat = foot_lat + perp_y * offset_dist / 111320
        left_lon = foot_lon + perp_x * offset_dist / (111320 * math.cos(math.radians(foot_lat)))
        right_lat = foot_lat - perp_y * offset_dist / 111320
        right_lon = foot_lon - perp_x * offset_dist / (111320 * math.cos(math.radians(foot_lat)))
        # 比较 (起点->候选点->终点) 总长度
        left_len = haversine(start_pt[0], start_pt[1], left_lat, left_lon) + haversine(left_lat, left_lon, end_pt[0], end_pt[1])
        right_len = haversine(start_pt[0], start_pt[1], right_lat, right_lon) + haversine(right_lat, right_lon, end_pt[0], end_pt[1])
        if left_len <= right_len:
            offset_lat, offset_lon = left_lat, left_lon
        else:
            offset_lat, offset_lon = right_lat, right_lon
    
    # 返回路径：起点 -> 绕行点 -> 终点
    return [start_pt, (offset_lat, offset_lon), end_pt]

# ================== 地图绘制函数 ==================
def draw_map():
    """生成 folium 地图，绘制障碍物、起点、终点及规划路径"""
    # 使用高德卫星图瓦片（需要替换为自己的key？高德卫星图无需key但可能有referer限制，此处用无key的公开瓦片）
    # 备选：https://webst0{s}.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}   (s=1-4)
    tile_url = "https://webst0{s}.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}"
    m = folium.Map(
        location=st.session_state.map_center,
        zoom_start=15,
        tiles=tile_url,
        attr="高德地图"
    )
    
    # 绘制障碍物（红色圆圈）
    for idx, obs in enumerate(st.session_state.obstacles):
        popup_text = f"障碍物 {idx+1}<br>高度: {obs['height']}m"
        folium.Circle(
            radius=st.session_state.safe_radius,
            location=(obs["lat"], obs["lon"]),
            color="red",
            fill=True,
            fill_opacity=0.4,
            popup=popup_text,
            tooltip=f"高 {obs['height']}m"
        ).add_to(m)
        # 标记中心点
        folium.Marker(
            (obs["lat"], obs["lon"]),
            icon=folium.DivIcon(html=f'<div style="font-size:10pt;color:red;">⚠️</div>')
        ).add_to(m)
    
    # 绘制起点（绿色）和终点（蓝色）
    folium.Marker(
        (st.session_state.start_point["lat"], st.session_state.start_point["lon"]),
        popup="起点 A",
        icon=folium.Icon(color="green", icon="play", prefix="fa")
    ).add_to(m)
    folium.Marker(
        (st.session_state.end_point["lat"], st.session_state.end_point["lon"]),
        popup="终点 B",
        icon=folium.Icon(color="blue", icon="stop", prefix="fa")
    ).add_to(m)
    
    # 绘制规划路径
    if st.session_state.planned_route:
        folium.PolyLine(
            st.session_state.planned_route,
            color="cyan",
            weight=5,
            opacity=0.8,
            tooltip="规划航线"
        ).add_to(m)
        # 标注路径点
        for i, pt in enumerate(st.session_state.planned_route):
            folium.CircleMarker(
                location=pt,
                radius=4,
                color="yellow",
                fill=True,
                popup=f"航点 {i+1}"
            ).add_to(m)
    
    return m

# ================== 侧边栏控件 ==================
with st.sidebar:
    st.header("⚙️ 飞行参数")
    st.session_state.flight_height = st.number_input("飞行高度 (米)", min_value=0.0, value=10.0, step=1.0)
    st.session_state.safe_radius = st.number_input("安全半径 (米)", min_value=1.0, value=5.0, step=1.0)
    st.session_state.route_mode = st.selectbox("航线模式", ["智能最优", "左侧绕行", "右侧绕行"])
    
    st.header("📍 起点 / 终点")
    col1, col2 = st.columns(2)
    with col1:
        st.write("**起点 A**")
        start_lat = st.number_input("纬度", key="start_lat", value=st.session_state.start_point["lat"], format="%.6f")
        start_lon = st.number_input("经度", key="start_lon", value=st.session_state.start_point["lon"], format="%.6f")
        if st.button("设为当前地图中心", key="center_start"):
            st.session_state.map_center = [start_lat, start_lon]
        st.session_state.start_point = {"lat": start_lat, "lon": start_lon}
    with col2:
        st.write("**终点 B**")
        end_lat = st.number_input("纬度", key="end_lat", value=st.session_state.end_point["lat"], format="%.6f")
        end_lon = st.number_input("经度", key="end_lon", value=st.session_state.end_point["lon"], format="%.6f")
        if st.button("设为当前地图中心", key="center_end"):
            st.session_state.map_center = [end_lat, end_lon]
        st.session_state.end_point = {"lat": end_lat, "lon": end_lon}
    
    st.header("🚧 障碍物管理")
    # 手动添加障碍物
    with st.expander("➕ 手动添加障碍物"):
        obs_lat = st.number_input("纬度", key="new_obs_lat", value=39.9042, format="%.6f")
        obs_lon = st.number_input("经度", key="new_obs_lon", value=116.4074, format="%.6f")
        obs_height = st.number_input("高度 (米)", key="new_obs_height", min_value=0.0, value=20.0, step=1.0)
        if st.button("添加障碍物"):
            st.session_state.obstacles.append({
                "lat": obs_lat,
                "lon": obs_lon,
                "height": obs_height
            })
            st.success("已添加")
            st.experimental_rerun()
    
    # 显示现有障碍物列表
    if st.session_state.obstacles:
        st.subheader("📋 现有障碍物")
        for i, obs in enumerate(st.session_state.obstacles):
            col_a, col_b = st.columns([4,1])
            with col_a:
                st.write(f"{i+1}. ({obs['lat']:.5f}, {obs['lon']:.5f}) 高 {obs['height']}m")
            with col_b:
                if st.button("🗑️", key=f"del_{i}"):
                    st.session_state.obstacles.pop(i)
                    st.experimental_rerun()
    else:
        st.info("暂无障碍物，请手动添加或在地图上点击添加")
    
    # 一键规划航线按钮
    st.markdown("---")
    if st.button("🚀 一键规划航线", use_container_width=True):
        with st.spinner("正在规划安全航线..."):
            try:
                route = calculate_safe_path(
                    st.session_state.start_point,
                    st.session_state.end_point,
                    st.session_state.obstacles,
                    st.session_state.flight_height,
                    st.session_state.safe_radius,
                    st.session_state.route_mode
                )
                if route:
                    st.session_state.planned_route = route
                    st.success("航线规划成功！")
                else:
                    st.error("无法规划航线，请检查起点/终点或障碍物设置")
            except Exception as e:
                st.error(f"规划出错: {str(e)}")
    
    # 清空所有障碍物
    if st.button("清空所有障碍物", use_container_width=True):
        st.session_state.obstacles.clear()
        st.session_state.planned_route = []
        st.experimental_rerun()

# ================== 主区域：地图 + 点击添加障碍物 ==================
st.subheader("🗺️ 卫星地图（高德）")
st.caption("点击地图任意位置 → 弹出窗口输入高度 → 自动添加障碍物")

# 显示地图并捕获点击事件
map_obj = draw_map()
output = st_folium(map_obj, width=1000, height=600, returned_objects=["last_clicked"])

# 处理地图点击：添加新障碍物
if output and output.get("last_clicked"):
    clicked = output["last_clicked"]
    lat, lon = clicked["lat"], clicked["lng"]
    # 使用对话框输入高度（Streamlit没有原生弹窗，用 session 临时标记）
    if "pending_click" not in st.session_state:
        st.session_state.pending_click = (lat, lon)
        st.experimental_rerun()

if "pending_click" in st.session_state:
    lat, lon = st.session_state.pending_click
    st.info(f"📍 点击位置: ({lat:.5f}, {lon:.5f})，请设置障碍物高度")
    height_input = st.number_input("障碍物高度 (米)", min_value=0.0, value=20.0, step=1.0, key="click_height")
    col_ok, col_cancel = st.columns(2)
    if col_ok.button("✅ 确认添加"):
        st.session_state.obstacles.append({"lat": lat, "lon": lon, "height": height_input})
        del st.session_state.pending_click
        st.success("障碍物已添加")
        st.experimental_rerun()
    if col_cancel.button("❌ 取消"):
        del st.session_state.pending_click
        st.experimental_rerun()

# ================== 底部信息 ==================
st.markdown("---")
st.caption("💡 说明：\n- 红色圆圈表示障碍物的安全影响区（半径 = 安全半径）\n- 当飞行高度 ≤ 障碍物高度时，航线会尝试绕行\n- 绕行算法为简化版，仅处理第一个相交障碍物")
