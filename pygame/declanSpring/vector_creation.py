import numpy as np
import math
import cv2
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
import matplotlib.pyplot as plt
import geopandas as gpd

'''

NOTE: I MIXED UP CLOCKWISE AND COUNTER CLOCKWISE (*_*)

General Idea
- utility script to make vector field

Steps
- Draw main clockwise vector fields over roads
- Draw sub counter clockwise vector fields
'''

background = cv2.imread('./C3_map.png')
height, width = background.shape[:2]

'''
Clockwise Initial Polygons
'''
TopLeftClock = Polygon([(39, 195), (45, 161), (99, 89), (191, 25), (297, 27), (299, 191), (303, 245), (213, 259)])
BottomLeftClock = Polygon([(46, 1104), (41, 1369), (250, 1372), (272, 1098)])
BottomRightClock = Polygon([(1689, 1135), (1695, 1382), (2000, 1372), (1992, 1152)])
CircleClock = Polygon([(1566, 824), (1573, 661), (1581, 639), (1591, 619), (1607, 591), (1631, 563), (1661, 533), 
        (1693, 511), (1727, 495), (1741, 484), (1959, 495), (1957, 514), (1983, 534), (2023, 565), (2061, 625),
        (2081, 679), (2089, 740), (2085, 807), (2073, 858), (2050, 906), (2020, 944), (1963, 1000), (1742, 1018),
        (1693, 996), (1651, 962), (1603, 912)])
TopRightClock = Polygon([(1642, 288), (2004, 296), (1998, 14), (1622, 10)])
TopCenterClock = Polygon([(257, 30), (260, 192), (1652, 190), (1760, 24)])
LeftCenterClock = Polygon([(49, 170), (221, 248), (215, 1122), (49, 1307)])
BottomCenterClock = Polygon([(112, 1364), (279, 1197), (1669, 1193), (1851, 1364)])
RightCenterBottomClock = Polygon([(1736, 1150), (1967, 1239), (1961, 997), (1741, 1013)])
RightCenterTopClock = Polygon([(1740, 490), (1957, 514), (1963, 215), (1733, 283)])
CenterTopDown = Polygon([(1044, 192), (1044, 385), (837, 665), (1039, 660), (1390, 192)])
CenterTopStraightDown = Polygon([(1040, 192), (1245, 192), (1044, 387)])
CenterBottomDown = Polygon([(797, 820), (797, 1198), (956, 1200), (956, 820)])
CenterMidLeft = Polygon([(210, 660), (210, 822), (1572, 822), (1575, 660)])

'''
do other directions of the same general parts

do angel/direction calculation based on current pose and radius pose (and if counter/clockwise)

draw in correction order with right pixel values
'''
TopLeftCounter = Polygon([(212, 262), (135, 270),
                        (134, 216), (135, 200), (137, 188), (141, 177), (147, 166), (158, 151), (173, 138),
                        (190, 130), (211, 123), (240, 117), (272, 112), (281, 113), (284, 191), (276, 192),
                        (271, 194), (263, 198), (250, 207), (239, 218), (224, 235), (213, 254)])
BottomLeftCounter = Polygon([(136, 1100), (135, 1141), (135, 1170), (135, 1198), (136, 1208), (138, 1228),
                        (142, 1245), (149, 1254), (156, 1258), (160, 1262), (168, 1270), (185, 1274), (210, 1278),
                        (228, 1281), (240, 1281), (258, 1282), (275, 1282), (289, 1282), (299, 1282), (298, 1197),
                        (282, 1198), (264, 1193), (242, 1178), (225, 1158), (217, 1142), (213, 1128), (212, 1102)])
BottomRightCounter = Polygon([(1663, 1198), (1663, 1193), (1678, 1193), (1702, 1185), (1723, 1172), (1734, 1159),
                        (1741, 1149), (1742, 1128), (1844, 1126), (1844, 1148), (1845, 1182), (1845, 1200), (1844, 1209),
                        (1840, 1226), (1830, 1245), (1814, 1260), (1789, 1274), (1765, 1283), (1724, 1282), (1662, 1280)])
CircleCounter = Polygon([(1664, 671), (1653, 747), (1664, 831), (1685, 872), (1713, 900), (1739, 914), (1750, 917),
                        (1805, 921), (1861, 923), (1907, 910), (1933, 891), (1958, 871), (1969, 856), (1980, 831),
                        (1987, 815), (1993, 796), (1995, 775), (1995, 754), (1992, 727), (1989, 701), (1983, 680),
                        (1977, 663), (1971, 649), (1961, 635), (1952, 623), (1943, 615), (1899, 592), (1857, 581),
                        (1810, 577), (1778, 576), (1751, 583), (1735, 589), (1716, 599), (1704, 608), (1693, 619),
                        (1682, 631), (1671, 647), (1665, 661)])
TopRightCounter = Polygon([(1585, 192), (1581, 111), (1667, 112), (1700, 113), (1719, 115), (1728, 116), (1736, 118),
                        (1747, 122), (1758, 128), (1771, 138), (1783, 150), (1793, 160), (1802, 170), (1813, 185),
                        (1823, 198), (1830, 210), (1836, 223), (1841, 237), (1845, 255), (1846, 275), (1846, 290), (1742, 296),
                        (1736, 283), (1726, 266), (1713, 251), (1693, 235), (1668, 220), (1641, 207), (1616, 199), (1601, 195)])
TopCenterCounter = Polygon([(280, 112), (280, 191), (1585, 191), (1585, 112)])
LeftCenterCounter = Polygon([(134, 262), (211, 259), (211, 1111), (135, 1139)])
BottomCenterCounter = Polygon([(278, 1196), (185, 1273), (237, 1281), (785, 1281), (975, 1279), (1731, 1281), (1665, 1195),
                        (957, 1197), (795, 1195)])
RightCenterBottomCounter = Polygon([(1742, 1008), (1845, 996), (1844, 1198), (1741, 1146)])
RightCenterTopCounter = Polygon([(1741, 294), (1845, 260), (1847, 300), (1848, 494), (1743, 490)])
CenterTopUp = Polygon([(941, 665), (952, 645), (1173, 326), (1235, 191), (1388, 191), (1372, 213), (1037, 663)])
CenterBottomUp = Polygon([(878, 821), (955, 819), (955, 1198), (877, 1200)])
CenterMidRight = Polygon([(212, 746), (209, 821), (795, 821), (955, 823), (1565, 821), (1569, 745), (1043, 745), (795, 743)])

polygons_inorder = [CenterMidRight, CenterBottomUp, CenterTopUp, RightCenterTopCounter, RightCenterBottomCounter,
                    BottomCenterCounter, LeftCenterCounter, TopCenterCounter, TopRightCounter, CircleCounter,
                    BottomRightCounter, BottomLeftCounter, TopLeftCounter, CenterMidLeft, CenterBottomDown, 
                    CenterTopDown, RightCenterTopClock, RightCenterBottomClock, BottomCenterClock, LeftCenterClock,
                    TopCenterClock, TopRightClock, CircleClock, BottomRightClock, BottomLeftClock, TopLeftClock]


# fig, ax = plt.subplots(1,1)
# for poly in polygons_inorder:
#     p = gpd.GeoSeries(poly)
#     p.plot(ax=ax)

# plt.show()


'''
Now make vector field math
'''

map = np.ones(background.shape, dtype=np.uint8)
height, width = background.shape[:2]

# Pixel Value = [Blue Green Red]
# Vector Values = [X Y Risk?Rules]
# 127 is zero
for y_pxl in range(height):
    for x_pxl in range(width):
        bg_pxl = background[y_pxl][x_pxl]
        if bg_pxl[2] != 0x6a:
            map[y_pxl][x_pxl] = [127, 127, 0]
            if CenterTopStraightDown.contains(Point(x_pxl, y_pxl)):
                map[y_pxl][x_pxl] = [127, 254, 0] 
                continue
            for i, poly in enumerate(polygons_inorder):
                if poly.contains(Point(x_pxl, y_pxl)):
                    if i == 0:
                        # Center Mid Right
                        map[y_pxl][x_pxl] = [254, 127, 0] # <1, 0>
                    elif i == 1:
                        # Center Bottom Up
                        map[y_pxl][x_pxl] = [127, 0, 0] # <0, -1>
                    elif i == 2:
                        # Center Top UP
                        map[y_pxl][x_pxl] = [int(127 + 127 * np.cos(55 * np.pi/180)), int(127 - 127 * np.sin(55 * np.pi/180)), 0]
                    elif i == 3:
                        # Right Center Top Counter
                        map[y_pxl][x_pxl] = [127, 254, 0] # <0, 1>
                    elif i == 4:
                        # Right Center Bottom Counter
                        map[y_pxl][x_pxl] = [127, 254, 0] # <0, 1>
                    elif i == 5:
                        # Bottom Center Counter
                        map[y_pxl][x_pxl] = [0, 127, 0] # <-1, 0>
                    elif i == 6:
                        # Left Center Counter
                        map[y_pxl][x_pxl] = [127, 0, 0] # <0, -1>
                    elif i == 7:
                        # Top Center Counter
                        map[y_pxl][x_pxl] = [254, 127, 0] # <1, 0>
                    elif i == 8:
                        # Top Right Counter
                        point = np.array([x_pxl, y_pxl])
                        clock_wise = 1 # make -1 if counter
                        center = np.array([1612, 290])
                        axis = np.array([2000, 290])
                        vec1 = point - center
                        vec2 = point - axis
                        cos_of_angle = np.dot(vec1, vec2)/(np.linalg.norm(vec1) * np.linalg.norm(vec2))
                        theta = np.arccos(cos_of_angle)
                        angle = np.pi/2 - theta
                        sign_y = clock_wise * ((x_pxl - center[0])/(abs(x_pxl - center[0])))
                        if x_pxl > center[0]:
                            sign_y = -1 * clock_wise
                        sign_x = clock_wise * -1 * ((y_pxl - center[1])/(abs(y_pxl - center[1])))
                        if math.isnan(sign_y):
                            sign_y = 1
                        if math.isnan(sign_x):
                            sign_x = 1
                        map[y_pxl][x_pxl] = [int(127+sign_x*127*np.cos(angle)), int(127+sign_y*127*np.sin(angle)), 0]
                    elif i == 9:
                        # Circle Counter
                        point = np.array([x_pxl, y_pxl])
                        clock_wise = 1 # make -1 if counter
                        center = np.array([1838, 749])
                        axis = np.array([2100, 749])
                        vec1 = point - center
                        vec2 = point - axis
                        cos_of_angle = np.dot(vec1, vec2)/(np.linalg.norm(vec1) * np.linalg.norm(vec2))
                        theta = np.arccos(cos_of_angle)
                        angle = np.pi/2 - theta
                        sign_y = clock_wise * ((x_pxl - center[0])/(abs(x_pxl - center[0])))
                        if x_pxl > center[0]:
                            sign_y = -1 * clock_wise
                        sign_x = clock_wise * -1 * ((y_pxl - center[1])/(abs(y_pxl - center[1])))
                        if math.isnan(sign_y):
                            sign_y = 1
                        if math.isnan(sign_x):
                            sign_x = 1
                        map[y_pxl][x_pxl] = [int(127+sign_x*127*np.cos(angle)), int(127+sign_y*127*np.sin(angle)), 0]
                    elif i == 10:
                        # Bottom right Counter
                        point = np.array([x_pxl, y_pxl])
                        clock_wise = 1 # make -1 if counter
                        center = np.array([1665, 1123])
                        axis = np.array([2100, 1123])
                        vec1 = point - center
                        vec2 = point - axis
                        cos_of_angle = np.dot(vec1, vec2)/(np.linalg.norm(vec1) * np.linalg.norm(vec2))
                        theta = np.arccos(cos_of_angle)
                        angle = np.pi/2 - theta
                        sign_y = clock_wise * ((x_pxl - center[0])/(abs(x_pxl - center[0])))
                        if x_pxl > center[0]:
                            sign_y = -1 * clock_wise
                        sign_x = clock_wise * -1 * ((y_pxl - center[1])/(abs(y_pxl - center[1])))
                        if math.isnan(sign_y):
                            sign_y = 1
                        if math.isnan(sign_x):
                            sign_x = 1
                        map[y_pxl][x_pxl] = [int(127+sign_x*127*np.cos(angle)), int(127+sign_y*127*np.sin(angle)), 0]
                    elif i == 11:
                        # Bottom left Counter
                        point = np.array([x_pxl, y_pxl])
                        clock_wise = 1 # make -1 if counter
                        center = np.array([317, 1111])
                        axis = np.array([2000, 1111])
                        vec1 = point - center
                        vec2 = point - axis
                        cos_of_angle = np.dot(vec1, vec2)/(np.linalg.norm(vec1) * np.linalg.norm(vec2))
                        theta = np.arccos(cos_of_angle)
                        angle = np.pi/2 - theta
                        sign_y = clock_wise * ((x_pxl - center[0])/(abs(x_pxl - center[0])))
                        sign_x = clock_wise * -1 * ((y_pxl - center[1])/(abs(y_pxl - center[1])))
                        if math.isnan(sign_y):
                            sign_y = 1
                        if math.isnan(sign_x):
                            sign_x = 1
                        map[y_pxl][x_pxl] = [int(127+sign_x*127*np.cos(angle)), int(127+sign_y*127*np.sin(angle)), 0]
                    elif i == 12:
                        # Top Left Counter
                        point = np.array([x_pxl, y_pxl])
                        clock_wise = 1 # make -1 if counter
                        center = np.array([321, 288])
                        axis = np.array([2000, 288])
                        vec1 = point - center
                        vec2 = point - axis
                        cos_of_angle = np.dot(vec1, vec2)/(np.linalg.norm(vec1) * np.linalg.norm(vec2))
                        theta = np.arccos(cos_of_angle)
                        angle = np.pi/2 - theta
                        sign_y = clock_wise * ((x_pxl - center[0])/(abs(x_pxl - center[0])))
                        sign_x = clock_wise * -1 * ((y_pxl - center[1])/(abs(y_pxl - center[1])))
                        if math.isnan(sign_y):
                            sign_y = 1
                        if math.isnan(sign_x):
                            sign_x = 1
                        map[y_pxl][x_pxl] = [int(127+sign_x*127*np.cos(angle)), int(127+sign_y*127*np.sin(angle)), 0]
                    elif i == 13:
                        # Center Mid  left
                        map[y_pxl][x_pxl] = [0, 127, 0] # <-1, 0
                    elif i == 14:
                        # Center Bottom Down
                        map[y_pxl][x_pxl] = [127, 254, 0] # <0, 1>
                    elif i == 15:
                        # Center Top Down
                        map[y_pxl][x_pxl] = [int(127 - 127 * np.cos(55 * np.pi/180)), int(127 + 127 * np.sin(55 * np.pi/180)), 0]
                    elif i == 16:
                        # Right Center Top Clock
                        map[y_pxl][x_pxl] = [127, 0, 0] # <0, 1>
                    elif i == 17:
                        # Right Cente Bottom Clock
                        map[y_pxl][x_pxl] = [127, 0, 0] # <0, 1>
                    elif i == 18:
                        # Bottom Center Clock
                        map[y_pxl][x_pxl] = [254, 127, 0] # <1, 0>
                    elif i == 19:
                        # Left Center Clock
                        map[y_pxl][x_pxl] = [127, 254, 0] # <0, 1>
                    elif i == 20:
                        # Top Center Clock
                        map[y_pxl][x_pxl] = [0, 127, 0] # <-1, 0>
                    # all clock curves
                    elif i == 21:
                        # Top Right Counter
                        point = np.array([x_pxl, y_pxl])
                        clock_wise = -1 # make -1 if counter
                        center = np.array([1612, 290])
                        axis = np.array([2000, 290])
                        vec1 = point - center
                        vec2 = point - axis
                        cos_of_angle = np.dot(vec1, vec2)/(np.linalg.norm(vec1) * np.linalg.norm(vec2))
                        theta = np.arccos(cos_of_angle)
                        angle = np.pi/2 - theta
                        sign_y = clock_wise * ((x_pxl - center[0])/(abs(x_pxl - center[0])))
                        if x_pxl > center[0]:
                            sign_y = -1 * clock_wise
                        sign_x = clock_wise * -1 * ((y_pxl - center[1])/(abs(y_pxl - center[1])))
                        if math.isnan(sign_y):
                            sign_y = 1
                        if math.isnan(sign_x):
                            sign_x = 1
                        map[y_pxl][x_pxl] = [int(127+sign_x*127*np.cos(angle)), int(127+sign_y*127*np.sin(angle)), 0]
                    elif i == 22:
                        # Circle Counter
                        point = np.array([x_pxl, y_pxl])
                        clock_wise = -1 # make -1 if counter
                        center = np.array([1838, 749])
                        axis = np.array([2100, 749])
                        vec1 = point - center
                        vec2 = point - axis
                        cos_of_angle = np.dot(vec1, vec2)/(np.linalg.norm(vec1) * np.linalg.norm(vec2))
                        theta = np.arccos(cos_of_angle)
                        angle = np.pi/2 - theta
                        sign_y = clock_wise * ((x_pxl - center[0])/(abs(x_pxl - center[0])))
                        if x_pxl > center[0]:
                            sign_y = -1 * clock_wise
                        sign_x = clock_wise * -1 * ((y_pxl - center[1])/(abs(y_pxl - center[1])))
                        if math.isnan(sign_y):
                            sign_y = 1
                        if math.isnan(sign_x):
                            sign_x = 1
                        map[y_pxl][x_pxl] = [int(127+sign_x*127*np.cos(angle)), int(127+sign_y*127*np.sin(angle)), 0]
                    elif i == 23:
                        # Bottom right Counter
                        point = np.array([x_pxl, y_pxl])
                        clock_wise = -1 # make -1 if counter
                        center = np.array([1665, 1123])
                        axis = np.array([2100, 1123])
                        vec1 = point - center
                        vec2 = point - axis
                        cos_of_angle = np.dot(vec1, vec2)/(np.linalg.norm(vec1) * np.linalg.norm(vec2))
                        theta = np.arccos(cos_of_angle)
                        angle = np.pi/2 - theta
                        sign_y = clock_wise * ((x_pxl - center[0])/(abs(x_pxl - center[0])))
                        if x_pxl > center[0]:
                            sign_y = -1 * clock_wise
                        sign_x = clock_wise * -1 * ((y_pxl - center[1])/(abs(y_pxl - center[1])))
                        if math.isnan(sign_y):
                            sign_y = 1
                        if math.isnan(sign_x):
                            sign_x = 1
                        map[y_pxl][x_pxl] = [int(127+sign_x*127*np.cos(angle)), int(127+sign_y*127*np.sin(angle)), 0]
                    elif i == 24:
                        # Bottom left Counter
                        point = np.array([x_pxl, y_pxl])
                        clock_wise = -1 # make -1 if counter
                        center = np.array([317, 1111])
                        axis = np.array([2000, 1111])
                        vec1 = point - center
                        vec2 = point - axis
                        cos_of_angle = np.dot(vec1, vec2)/(np.linalg.norm(vec1) * np.linalg.norm(vec2))
                        theta = np.arccos(cos_of_angle)
                        angle = np.pi/2 - theta
                        sign_y = clock_wise * ((x_pxl - center[0])/(abs(x_pxl - center[0])))
                        sign_x = clock_wise * -1 * ((y_pxl - center[1])/(abs(y_pxl - center[1])))
                        if math.isnan(sign_y):
                            sign_y = 1
                        if math.isnan(sign_x):
                            sign_x = 1
                        map[y_pxl][x_pxl] = [int(127+sign_x*127*np.cos(angle)), int(127+sign_y*127*np.sin(angle)), 0]
                    elif i == 25:
                        # Top Left Counter
                        point = np.array([x_pxl, y_pxl])
                        clock_wise = -1 # make -1 if counter
                        center = np.array([321, 288])
                        axis = np.array([2000, 288])
                        vec1 = point - center
                        vec2 = point - axis
                        cos_of_angle = np.dot(vec1, vec2)/(np.linalg.norm(vec1) * np.linalg.norm(vec2))
                        theta = np.arccos(cos_of_angle)
                        angle = np.pi/2 - theta
                        sign_y = clock_wise * ((x_pxl - center[0])/(abs(x_pxl - center[0])))
                        sign_x = clock_wise * -1 * ((y_pxl - center[1])/(abs(y_pxl - center[1])))
                        if math.isnan(sign_y):
                            sign_y = 1
                        if math.isnan(sign_x):
                            sign_x = 1
                        map[y_pxl][x_pxl] = [int(127+sign_x*127*np.cos(angle)), int(127+sign_y*127*np.sin(angle)), 0]
                    break
cv2.imwrite('./vector_map.png', map)



'''
intellegent driving module (IDM)
- modulates speed to seems smarter
'''
