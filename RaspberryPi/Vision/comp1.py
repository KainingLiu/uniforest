# 导入OpenCV视觉库（处理图像、摄像头）
import cv2
# 导入数值计算库（处理颜色阈值、矩阵运算）
import numpy as np

# ===================== 比赛参数 =====================
# 正方体真实边长：10厘米（测距公式需要用真实尺寸）
REAL_WIDTH = 10.0       
# 相机标定焦距：固定值（决定测距精度）
FOCAL_LENGTH = 550      
# 摄像头画面分辨率：宽640，高480
FRAME_W, FRAME_H = 640, 480
# 画面中心点X坐标（320），机器人要对准的目标位置
CX_CENTER = FRAME_W // 2

# 中心允许误差：±15像素
# 只要物体中心在320±15范围内，就判定为“已对准”，避免机器人疯狂抖动
OFFSET_TOLERANCE = 15
# ====================================================

# 颜色阈值：HSV格式（识别橙色）
# 最低阈值
lower_orange = np.array([10, 150, 80])
# 最高阈值
upper_orange = np.array([22, 255, 255])

# 颜色阈值：HSV格式（识别紫色）
lower_purple = np.array([125, 60, 50])
upper_purple = np.array([165, 255, 255])

# 打开摄像头（0=默认摄像头，CAP_DSHOW=Windows专用驱动）
cap = cv2.VideoCapture(1)
# 设置摄像头宽度
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
# 设置摄像头高度
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)

# ==================== 去噪函数：清理二值化图像 ====================
# 输入：颜色掩码（黑白图）
# 输出：干净的掩码（去掉噪点）
def denoise_mask(mask):
    # 使用高斯模糊对二值掩码进行降噪处理
    mask = cv2.GaussianBlur(mask, (5, 5), 0)
    # 二值化恢复清晰掩码
    _, mask = cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY)
    return mask

# ==================== 核心函数：检测立方体 + 对准 + 测距 ====================
# 输入：当前画面、颜色最低阈值、最高阈值、颜色名称、绘制颜色
def detect_and_align_cube(frame, lower, upper, color_name, box_color):
    # 1. 把画面从BGR转成HSV颜色空间（颜色识别更稳定）
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # 2. 颜色提取：生成掩码（目标颜色=白色，其他=黑色）
    mask = cv2.inRange(hsv, lower, upper)
    # 3. 调用去噪函数，清理掩码
    mask = denoise_mask(mask)

    # 4. 查找轮廓：只找最外层轮廓（RETR_EXTERNAL）
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 遍历所有找到的轮廓
    for cnt in contours:
        # 计算轮廓面积
        area = cv2.contourArea(cnt)
        # 面积太小（<800）判定为干扰，跳过
        if area < 800:
            continue

        # 5. 计算轮廓的重心（中心坐标）
        M = cv2.moments(cnt)
        # 防止分母为0报错
        if M['m00'] == 0:
            continue
        # 重心X坐标
        cx = int(M['m10'] / M['m00'])
        # 重心Y坐标
        cy = int(M['m01'] / M['m00'])

        # ===== 第一步：计算偏移量，判断是否对准中心 =====
        # 偏移量 = 物体中心X - 画面中心X
        offset_x = cx - CX_CENTER
        # 如果偏移量绝对值 < 允许误差 → 判定为已对准中心
        is_center = abs(offset_x) < OFFSET_TOLERANCE

        # ===== 第二步：只有对准中心后，才开始测距 =====
        distance = 0
        if is_center:
            # 获取轮廓的外接矩形（x,y=左上角坐标，w=宽度，h=高度）
            x, y, w, h = cv2.boundingRect(cnt)
            # 测距公式：距离 = (真实宽度 × 焦距) ÷ 图像像素宽度
            distance = (REAL_WIDTH * FOCAL_LENGTH) / w

        # ===== 画面绘制 =====
        # 在重心位置画一个实心小圆点
        cv2.circle(frame, (cx, cy), 5, box_color, -1)
        # 在圆点上方显示颜色名称
        cv2.putText(frame, f"{color_name}", (cx-30, cy-25), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 2)

        # 如果已对准中心
        if is_center:
            # 显示距离（绿色字体）
            cv2.putText(frame, f"{distance:.1f}cm", (cx-30, cy-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
            # 显示“ALIGNED”已对准提示
            cv2.putText(frame, "ALIGNED", (cx-35, cy+20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,255,0), 1)
        # 如果未对准
        else:
            # 显示“对准中”+偏移量
            cv2.putText(frame, f"ALIGN... {offset_x}", (cx-40, cy-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 1)

        # ==================== 控制台输出（给机器人控制用） ====================
        if is_center:
            print(f"[{color_name}] 已对准 | 距离:{distance:.1f}cm")
        else:
            print(f"[{color_name}] 对准中... 偏移X:{offset_x}")

# ==================== 主循环：持续读取摄像头画面 ====================
while True:
    # 读取一帧画面 ret=是否成功，frame=画面数据
    ret, frame = cap.read()
    # 读取失败则退出循环
    if not ret:
        break

    # 调用函数：识别橙色立方体
    detect_and_align_cube(frame, lower_orange, upper_orange, "ORANGE", (0,165,255))
    # 调用函数：识别紫色立方体
    detect_and_align_cube(frame, lower_purple, upper_purple, "PURPLE", (255,0,255))

    # 在画面中心画一条绿色竖线（参考线）
    cv2.line(frame, (CX_CENTER, 0), (CX_CENTER, FRAME_H), (0,255,0), 1)

    # 显示最终处理后的画面
    cv2.imshow("Cube Align & Distance", frame)
    # 按ESC键（ASCII=27）退出程序
    if cv2.waitKey(1) & 0xFF == 27:
        break

# 释放摄像头资源
cap.release()
# 关闭所有OpenCV窗口
cv2.destroyAllWindows()