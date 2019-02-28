# Python 2/3 compatibility
from __future__ import print_function

import math

import cv2 as cv
import numpy as np


class ObstacleAvoidance(object):
    def __init__(self, w, h, z=2):
        self.width = w
        self.height = h
        # sensor_width_in_mm/focal_length_in_mm
        self.focal_length = 0.9  # Logitec C270: 3.6mm/4mm
        self.min_dist_in_meter = z
        self.ply_header = '''ply
        format ascii 1.0
        element vertex %(vert_num)d
        property float x
        property float y
        property float z
        property uchar red
        property uchar green
        property uchar blue
        end_header
        '''

    def pre_targets(self):
        D = None
        XZ_A = 0
        YZ_A = 0
        rz = None
        pl = None
        targets = []
        for ry in range(0, int(self.height / 2), 100):
            for rx in range(0, int(self.width / 2), 100):
                targets.append([rx, ry, rz, XZ_A, YZ_A, D, pl])
                if rx > 0:
                    targets.append([rx * -1, ry, rz, XZ_A, YZ_A, D, pl])
                if ry > 0:
                    targets.append([rx, ry * -1, rz, XZ_A, YZ_A, D, pl])
                if rx > 0 and ry > 0:
                    targets.append([rx * -1, ry * -1, rz, XZ_A, YZ_A, D, pl])
        for i, target in enumerate(targets):
            x_min = target[0] - 100
            x_max = target[0] + 100
            y_min = target[1] - 100
            y_max = target[1] + 100
            pl = [x_min, x_max, y_min, y_max]
            targets[i][6] = pl
        return targets

    def find_targets(self, imgL, imgR):
        # disparity range is tuned for L and R image pair
        # SGBM Parameters -----------------
        window_size = 3  # wsize default 3; 5; 7 for SGBM reduced size image; 15 for SGBM full size image (1300px and above); 5 Works nicely

        left_matcher = cv.StereoSGBM_create(
            minDisparity=0,
            numDisparities=160,  # max_disp has to be dividable by 16 f. E. HH 192, 256
            blockSize=5,
            P1=8 * 3 * window_size ** 2,
            # wsize default 3; 5; 7 for SGBM reduced size image; 15 for SGBM full size image (1300px and above); 5 Works nicely
            P2=32 * 3 * window_size ** 2,
            disp12MaxDiff=1,
            uniquenessRatio=15,
            speckleWindowSize=0,
            speckleRange=2,
            preFilterCap=63,
            mode=cv.STEREO_SGBM_MODE_SGBM_3WAY
        )

        right_matcher = cv.ximgproc.createRightMatcher(left_matcher)

        # FILTER Parameters
        lmbda = 80000
        sigma = 1.2
        visual_multiplier = 1.0

        wls_filter = cv.ximgproc.createDisparityWLSFilter(matcher_left=left_matcher)
        wls_filter.setLambda(lmbda)
        wls_filter.setSigmaColor(sigma)

        print('computing disparity...')
        displ = left_matcher.compute(imgL, imgR)  # .astype(np.float32)/16
        dispr = right_matcher.compute(imgR, imgL)  # .astype(np.float32)/16
        displ = np.int16(displ)
        dispr = np.int16(dispr)
        filteredImg = wls_filter.filter(displ, imgL, None, dispr)  # important to put "imgL" here!!!

        filteredImg = cv.normalize(src=filteredImg, dst=filteredImg, beta=0, alpha=255, norm_type=cv.NORM_MINMAX);
        filteredImg = np.uint8(filteredImg)

        print('Finding distance map...')
        B = 0.8
        out_points = []
        f = 0.9 * self.width
        middleX = int(self.width / 2)
        middleY = int(self.height / 2)
        for x in range(self.width):
            for y in range(self.height):
                try:
                    d = filteredImg[x, y]
                    Z = f * B / d
                    if Z == float('Inf'):
                        Z = self.min_dist_in_meter
                    out_points.append([x - middleX, y - middleY, Z])
                    # print("Point", (x, y), "Distance", Z)
                except:
                    pass
        out_points = np.array(out_points)

        print('generating possible targets...')
        possible_targets = self.pre_targets()
        # print("P", possible_targets)
        for i, p in enumerate(out_points):
            for j, t in enumerate(possible_targets):
                if possible_targets[j][2] is None:
                    if p[0] > t[6][0] and p[0] < t[6][1] and p[1] > t[6][2] and p[1] < t[6][3]:
                        possible_targets[j][2] = p[2]
                else:
                    if p[2] < possible_targets[j][2]:
                        if p[0] > t[6][0] and p[0] < t[6][1] and p[1] > t[6][2] and p[1] < t[6][3]:
                            possible_targets[j][2] = p[2]

        print('generating available paths...')
        available_targets = []
        for target in possible_targets:
            if target[2] is None or target[2] >= self.min_dist_in_meter:
                if target[2] is None:
                    target[2] = self.min_dist_in_meter
                target[3] = self.get_angle_of_line_between_two_points((0, 0), (target[0], target[2]))
                target[4] = self.get_angle_of_line_between_two_points((0, 0), (target[1], target[2]))
                target[5] = self.distance_from_centre((target[0], target[1], target[2]))
                available_targets.append(target)
                print("Target:", target)

                txt = str(round(target[2], 1))
                px = middleX + target[0]
                py = middleY + target[1]

                cv.putText(filteredImg, txt, (px, py), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv.LINE_AA)
                ## 100= circle radius, 255= line colorn 1= non fill -1= fill
                cv.circle(filteredImg, (px, py), 100, 255, 1)

        # cv.imshow('imgL', imgL)
        cv.imshow('disparity', filteredImg)
        # cv.imshow('imgR', imgR)

        return available_targets

    def find_targets2(self, imgL, imgR):
        # disparity range is tuned for L and R image pair
        window_size = 3
        min_disp = 32
        num_disp = 112 - min_disp
        stereo = cv.StereoSGBM_create(
            minDisparity=min_disp,
            numDisparities=num_disp,
            blockSize=11,
            P1=8 * 3 * window_size ** 2,
            P2=32 * 3 * window_size ** 2,
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=32
        )

        print('computing disparity...')
        disp = stereo.compute(imgL, imgR).astype(np.float32) / 16.0
        disparity = (disp - min_disp) / num_disp

        print('Finding distance map...')
        B = 0.8
        out_points = []
        f = 0.9 * self.width
        middleX = int(self.width / 2)
        middleY = int(self.height / 2)
        for x in range(self.width):
            for y in range(self.height):
                try:
                    d = disparity[x, y]
                    Z = f * B / d
                    out_points.append([x - middleX, y - middleY, Z])
                    # print("Point", (x, y), "Distance", Z)
                except:
                    pass
        out_points = np.array(out_points)

        print('generating possible targets...')
        possible_targets = self.pre_targets()
        for i, p in enumerate(out_points):
            for j, t in enumerate(possible_targets):
                if possible_targets[j][2] is None:
                    if p[0] > t[6][0] and p[0] < t[6][1] and p[1] > t[6][2] and p[1] < t[6][3]:
                        possible_targets[j][2] = p[2]
                else:
                    if p[2] < possible_targets[j][2]:
                        if p[0] > t[6][0] and p[0] < t[6][1] and p[1] > t[6][2] and p[1] < t[6][3]:
                            possible_targets[j][2] = p[2]

        print('generating available paths...')
        available_targets = []
        for target in possible_targets:
            if target[2] is None or target[2] > self.min_dist_in_meter:
                if target[2] is None:
                    target[2] = self.min_dist_in_meter
                target[3] = self.get_angle_of_line_between_two_points((0, 0), (target[0], target[2]))
                target[4] = self.get_angle_of_line_between_two_points((0, 0), (target[1], target[2]))
                target[5] = self.distance_from_centre((target[0], target[1], target[2]))
                available_targets.append(target)
                print("Target", target)

                txt = str(target[5])
                px = middleX + target[0]
                py = middleY + target[1]

                cv.putText(disparity, txt, (px, py), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv.LINE_AA)
                cv.circle(disparity, (px, py), 100, 255, 1)  ## 100= circle radius, 255= line colorn 1=non fill -1= fill

        # cv.imshow('imgL', imgL)
        cv.imshow('disparity', disparity)
        # cv.imshow('imgR', imgR)

        return available_targets

    def find_targets3(self, imgL, imgR):
        # disparity range is tuned for L and R image pair
        window_size = 3
        min_disp = 32
        num_disp = 112 - min_disp
        stereo = cv.StereoSGBM_create(
            minDisparity=min_disp,
            numDisparities=num_disp,
            blockSize=11,
            P1=8 * 3 * window_size ** 2,
            P2=32 * 3 * window_size ** 2,
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=32
        )

        print('computing disparity...')
        disp = stereo.compute(imgL, imgR).astype(np.float32) / 16.0

        print('generating 3d point cloud...')
        f = self.focal_length * self.width  # guess for focal length
        Q = np.float32([[1, 0, 0, -0.5 * self.width],
                        [0, -1, 0, 0.5 * self.height],  # turn points 180 deg around x-axis,
                        [0, 0, 0, -f],  # so that y-axis looks up
                        [0, 0, 1, 0]])
        points = cv.reprojectImageTo3D(disp, Q)
        colors = cv.cvtColor(imgL, cv.COLOR_BGR2RGB)
        mask = disp > disp.min()
        out_points = points[mask]
        out_colors = colors[mask]
        out_fn = 'out.ply'
        self.write_ply(out_fn, out_points, out_colors)
        print('%s saved' % out_fn)

        print('generating possible targets...')
        possible_targets = self.pre_targets()
        for i, p in enumerate(out_points):
            for j, t in enumerate(possible_targets):
                if possible_targets[j][2] is None:
                    if p[0] > t[6][0] and p[0] < t[6][1] and p[1] > t[6][2] and p[1] < t[6][3]:
                        possible_targets[j][2] = p[2]
                else:
                    if p[2] > possible_targets[j][2]:
                        if p[0] > t[6][0] and p[0] < t[6][1] and p[1] > t[6][2] and p[1] < t[6][3]:
                            possible_targets[j][2] = p[2]

        middleX = int(self.width / 2)
        middleY = int(self.height / 2)
        disparity = (disp - min_disp) / num_disp

        print('generating available paths...')
        available_targets = []
        for target in possible_targets:
            if target[2] is None or (abs(target[2]) / self.focal_length) > self.min_dist_in_meter:
                if target[2] is None:
                    target[2] = -1 * (self.min_dist_in_meter * self.focal_length)
                pz = abs(target[2])
                target[3] = self.get_angle_of_line_between_two_points((0, 0), (target[0], pz))
                target[4] = self.get_angle_of_line_between_two_points((0, 0), (target[1], pz))
                target[5] = self.distance_from_centre((target[0], target[1], pz))
                available_targets.append(target)
                print("Target", target)

                txt = str(target[5])
                px = middleX + target[0]
                py = middleY + target[1]

                cv.putText(disparity, txt, (px, py), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv.LINE_AA)
                cv.circle(disparity, (px, py), 100, 255, 1)  ## 100= circle radius, 255= line colorn 1=non fill -1= fill

        cv.imshow('imgL', imgL)
        cv.imshow('disparity', disparity)
        cv.imshow('imgR', imgR)

        return available_targets

    def write_ply(self, fn, verts, colors):
        verts = verts.reshape(-1, 3)
        colors = colors.reshape(-1, 3)
        verts = np.hstack([verts, colors])
        with open(fn, 'wb') as f:
            f.write((self.ply_header % dict(vert_num=len(verts))).encode('utf-8'))
            np.savetxt(f, verts, fmt='%f %f %f %d %d %d ')

    def distance_between_two_point(self, p1, p2):
        dist = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        return dist

    def distance_from_centre(self, p):
        dist = p[0] ** 2 + p[1] ** 2 + p[2] ** 2
        return dist

    def get_angle_of_line_between_two_points(self, p1, p2):
        ax1Diff = p2[0] - p1[0]
        ax2Diff = p2[1] - p1[1]
        return math.degrees(math.atan2(ax1Diff, ax2Diff))
