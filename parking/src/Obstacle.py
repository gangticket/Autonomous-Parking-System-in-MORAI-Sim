import numpy as np
import matplotlib.pyplot as plt


class CircleObstacle(object):
    def __init__(self, x, y, r):
        self.x = x
        self.y = y
        self.r = r

    def is_inside(self, x, y):
        dist = np.hypot(x - self.x, y - self.y)
        if dist <= self.r:
            return True
        else:
            return False

    def plot(self, color='k'):
        theta = np.linspace(0, np.pi*2, num=30)
        x = self.x + self.r * np.cos(theta)
        y = self.y + self.r * np.sin(theta)

        plt.plot(x, y, color=color)

class RectangleObstacle(object):
    def __init__(self, center_x, center_y, width, height, angle):
        self.x = center_x
        self.y = center_y
        self.width = width
        self.height = height
        self.angle = angle

    def is_inside(self, x, y):
        # 좌표를 회전된 좌표계로 변환
        cos_val = np.cos(-self.angle)
        sin_val = np.sin(-self.angle)
        rotated_x = (x - self.x) * cos_val - (y - self.y) * sin_val
        rotated_y = (x - self.x) * sin_val + (y - self.y) * cos_val

        # 회전된 좌표로 충돌 검사
        half_width = 1.3 * self.width # 0.7
        half_height = 1.3 * self.height
        if -half_width <= rotated_x <= half_width and -half_height <= rotated_y <= half_height:
            return True

        return False
    def plot(self, color='g'):
        # 회전된 사각형의 꼭지점을 계산
        half_width = 0.5 * self.width
        half_height = 0.5 * self.height

        # 직사각형의 중심 좌표
        cx = self.x
        cy = self.y

        # 꼭지점 좌표 계산
        x1 = cx - half_width
        y1 = cy - half_height

        x2 = cx + half_width
        y2 = cy - half_height

        x3 = cx + half_width
        y3 = cy + half_height

        x4 = cx - half_width
        y4 = cy + half_height

        # 좌표 회전
        angle_rad = self.angle  # 각도를 라디안으로 변환
        cos_val = np.cos(angle_rad)
        sin_val = np.sin(angle_rad)

        x1_rotated = cx + (x1 - cx) * cos_val - (y1 - cy) * sin_val
        y1_rotated = cy + (x1 - cx) * sin_val + (y1 - cy) * cos_val

        x2_rotated = cx + (x2 - cx) * cos_val - (y2 - cy) * sin_val
        y2_rotated = cy + (x2 - cx) * sin_val + (y2 - cy) * cos_val

        x3_rotated = cx + (x3 - cx) * cos_val - (y3 - cy) * sin_val
        y3_rotated = cy + (x3 - cx) * sin_val + (y3 - cy) * cos_val

        x4_rotated = cx + (x4 - cx) * cos_val - (y4 - cy) * sin_val
        y4_rotated = cy + (x4 - cx) * sin_val + (y4 - cy) * cos_val

        # 회전된 직사각형 그리기
        plt.plot([x1_rotated, x2_rotated, x3_rotated, x4_rotated, x1_rotated],
                 [y1_rotated, y2_rotated, y3_rotated, y4_rotated, y1_rotated], color)


class Car(object):
    def __init__(self, center_x, center_y, width, height, angle):
        self.x = center_x
        self.y = center_y
        self.width = width
        self.height = height
        self.angle = angle

    def get_corners(self):
        half_width = 0.5 * self.width
        half_height = 0.5 * self.height

        # 꼭지점 좌표 계산 (회전 전)
        corners_before_rotation = [
            [self.x - half_width, self.y - half_height],
            [self.x + half_width, self.y - half_height],
            [self.x + half_width, self.y + half_height],
            [self.x - half_width, self.y + half_height]
        ]

        # 좌표 회전
        #angle_rad = np.radians(self.angle)
        cos_val = np.cos(self.angle)
        sin_val = np.sin(self.angle)

        corners_after_rotation = []

        for corner in corners_before_rotation:
            x_rotated = cos_val * (corner[0] - self.x) - sin_val * (corner[1] - self.y) + self.x
            y_rotated = sin_val * (corner[0] - self.x) + cos_val * (corner[1] - self.y) + self.y

            corners_after_rotation.append([x_rotated, y_rotated])

        return corners_after_rotation