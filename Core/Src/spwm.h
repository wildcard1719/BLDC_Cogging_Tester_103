/*
 * spwm.h
 *
 *  Created on: Mar 7, 2025
 *      Author: wildc
 */

#ifndef SPWM_H_
#define SPWM_H_


uint16_t spwm_arr[720] = {
		900, 907, 915, 923, 931, 939, 947, 954, 962, 970, 978, 986, 994, 1001, 1009, 1017,
		1025, 1032, 1040, 1048, 1056, 1063, 1071, 1079, 1087, 1094, 1102, 1109, 1117, 1125, 1132, 1140,
		1147, 1155, 1162, 1170, 1177, 1185, 1192, 1200, 1207, 1215, 1222, 1229, 1236, 1244, 1251, 1258,
		1265, 1273, 1280, 1287, 1294, 1301, 1308, 1315, 1322, 1329, 1336, 1342, 1349, 1356, 1363, 1369,
		1376, 1383, 1389, 1396, 1402, 1409, 1415, 1422, 1428, 1435, 1441, 1447, 1453, 1459, 1466, 1472,
		1478, 1484, 1490, 1496, 1501, 1507, 1513, 1519, 1524, 1530, 1536, 1541, 1547, 1552, 1557, 1563,
		1568, 1573, 1578, 1583, 1589, 1594, 1599, 1603, 1608, 1613, 1618, 1623, 1627, 1632, 1636, 1641,
		1645, 1650, 1654, 1658, 1662, 1666, 1671, 1675, 1678, 1682, 1686, 1690, 1694, 1697, 1701, 1704,
		1708, 1711, 1715, 1718, 1721, 1724, 1727, 1731, 1734, 1736, 1739, 1742, 1745, 1747, 1750, 1753,
		1755, 1757, 1760, 1762, 1764, 1766, 1768, 1770, 1772, 1774, 1776, 1778, 1779, 1781, 1782, 1784,
		1785, 1787, 1788, 1789, 1790, 1791, 1792, 1793, 1794, 1795, 1796, 1796, 1797, 1797, 1798, 1798,
		1798, 1799, 1799, 1799, 1799, 1799, 1799, 1799, 1798, 1798, 1798, 1797, 1797, 1796, 1796, 1795,
		1794, 1793, 1792, 1791, 1790, 1789, 1788, 1787, 1785, 1784, 1782, 1781, 1779, 1778, 1776, 1774,
		1772, 1770, 1768, 1766, 1764, 1762, 1760, 1757, 1755, 1753, 1750, 1747, 1745, 1742, 1739, 1736,
		1734, 1731, 1727, 1724, 1721, 1718, 1715, 1711, 1708, 1704, 1701, 1697, 1694, 1690, 1686, 1682,
		1678, 1675, 1671, 1666, 1662, 1658, 1654, 1650, 1645, 1641, 1636, 1632, 1627, 1623, 1618, 1613,
		1608, 1603, 1599, 1594, 1589, 1583, 1578, 1573, 1568, 1563, 1557, 1552, 1547, 1541, 1536, 1530,
		1524, 1519, 1513, 1507, 1501, 1496, 1490, 1484, 1478, 1472, 1466, 1459, 1453, 1447, 1441, 1435,
		1428, 1422, 1415, 1409, 1402, 1396, 1389, 1383, 1376, 1369, 1363, 1356, 1349, 1342, 1336, 1329,
		1322, 1315, 1308, 1301, 1294, 1287, 1280, 1273, 1265, 1258, 1251, 1244, 1236, 1229, 1222, 1215,
		1207, 1200, 1192, 1185, 1177, 1170, 1162, 1155, 1147, 1140, 1132, 1125, 1117, 1109, 1102, 1094,
		1087, 1079, 1071, 1063, 1056, 1048, 1040, 1032, 1025, 1017, 1009, 1001, 994, 986, 978, 970,
		962, 954, 947, 939, 931, 923, 915, 907, 900, 892, 884, 876, 868, 860, 852, 845,
		837, 829, 821, 813, 805, 798, 790, 782, 774, 767, 759, 751, 743, 736, 728, 720,
		712, 705, 697, 690, 682, 674, 667, 659, 652, 644, 637, 629, 622, 614, 607, 599,
		592, 584, 577, 570, 563, 555, 548, 541, 534, 526, 519, 512, 505, 498, 491, 484,
		477, 470, 463, 457, 450, 443, 436, 430, 423, 416, 410, 403, 397, 390, 384, 377,
		371, 364, 358, 352, 346, 340, 333, 327, 321, 315, 309, 303, 298, 292, 286, 280,
		275, 269, 263, 258, 252, 247, 242, 236, 231, 226, 221, 216, 210, 205, 200, 196,
		191, 186, 181, 176, 172, 167, 163, 158, 154, 149, 145, 141, 137, 133, 128, 124,
		121, 117, 113, 109, 105, 102, 98, 95, 91, 88, 84, 81, 78, 75, 72, 68,
		65, 63, 60, 57, 54, 52, 49, 46, 44, 42, 39, 37, 35, 33, 31, 29,
		27, 25, 23, 21, 20, 18, 17, 15, 14, 12, 11, 10, 9, 8, 7, 6,
		5, 4, 3, 3, 2, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0,
		1, 1, 1, 2, 2, 3, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,
		14, 15, 17, 18, 20, 21, 23, 25, 27, 29, 31, 33, 35, 37, 39, 42,
		44, 46, 49, 52, 54, 57, 60, 63, 65, 68, 72, 75, 78, 81, 84, 88,
		91, 95, 98, 102, 105, 109, 113, 117, 121, 124, 128, 133, 137, 141, 145, 149,
		154, 158, 163, 167, 172, 176, 181, 186, 191, 196, 200, 205, 210, 216, 221, 226,
		231, 236, 242, 247, 252, 258, 263, 269, 275, 280, 286, 292, 298, 303, 309, 315,
		321, 327, 333, 340, 346, 352, 358, 364, 371, 377, 384, 390, 397, 403, 410, 416,
		423, 430, 436, 443, 450, 457, 463, 470, 477, 484, 491, 498, 505, 512, 519, 526,
		534, 541, 548, 555, 563, 570, 577, 584, 592, 599, 607, 614, 622, 629, 637, 644,
		652, 659, 667, 674, 682, 690, 697, 705, 712, 720, 728, 736, 743, 751, 759, 767,
		774, 782, 790, 798, 805, 813, 821, 829, 837, 845, 852, 860, 868, 876, 884, 892
};

#endif /* SPWM_H_ */
