#ifndef __ACQUIRE_H
#define __ACQUIRE_H

#ifdef __cplusplus
 extern "C" {
#endif

static const uint16_t acq_data[] = {
#if 0
  65535,1927,1840,65535,1918,1838,65535,1913,1854,65535,1905,1858,65535,1903,1863,65535,1904,1874,65535,1919,
  1875,65535,1943,1872,65535,1968,1872,65535,1986,1866,65535,2003,1846,65535,2015,1845,65535,2022,1831,65535,
  2009,1808,65535,2005,1803,65535,1987,1795,65535,1974,1801,65535,1963,1793,65535,1951,1802,65535,1936,1801,
  65535,1931,1818,65535,1925,1806,65535,1915,1806,65535,1914,1808, 65535,1911,1807,65535,1918,1810,65535,1934,
  1811,65535,1942,1821, 65535,1965,1811,65535,1983,1808,65535,2001,1794,65535,2019,1775, 65535,2027,1751,65535,
  2027,1729,65535,2011,1714,65535,1999,1707, 65535,1971,1699,65535,1957,1700,65535,1935,1703,65535,1916,1712,
  65535,1902,1716,65535,1887,1726,65535,1879,1729,65535,1873,1731,65535,1873,1735,65535,1878,1735,65535,1892,
  1743,65535,1911,1742,65535,1929,1735,65535,1938,1727,65535,1961,1731,65535,1970,1712,65535,1969,1695,65535,
  1956,1677,65535,1936,1663,65535,1916,1655,65535,1889,1654,65535,1870,1657,65535,1846,1664,65535,1834,1671,
  65535,1819,1676,65535,1809,1683,65535,1806,1684,65535,1797,1693,65535,1798,1702,65535,1799,1711,65535,1808,
  1719,65535,1821,1724,65535,1833,1731,65535,1846,1728,65535,1859,1727,65535,1869,1714,65535,1868,1703,65535,
  1858,1691,65535,1847,1680,65535,1834,1674,65535,1819,1676,65535,1808,1680,65535,1796,1689,65535,1787,1696
#else
  //20000, 21736, 23420, 25000, 26427, 27660, 28660, 29396, 29848, 30000, 29848, 29396, 28660, 27660, 26427,
  //25000, 23420, 21736, 20000, 18264, 16580, 15000, 13573, 12340, 11340, 10604, 10152, 10000, 10152, 10604,
  //11340, 12340, 13573, 15000, 16580, 18264
  0x4E20, 0x54E8, 0x5B7C, 0x61A8, 0x673B, 0x6C0C, 0x6FF4, 0x72D4, 0x7498, 
  0x7530, 0x7498, 0x72D4, 0x6FF4, 0x6C0C, 0x673B, 0x61A8, 0x5B7C, 0x54E8,
  0x4E20, 0x4758, 0x40C4, 0x3A98, 0x3505, 0x3034, 0x2C4C, 0x296C, 0x27A8,
  0x2710, 0x27A8, 0x296C, 0x2C4C, 0x3034, 0x3505, 0x3A98, 0x40C4, 0x4758
#endif
};




extern uint32_t ble_output(uint8_t * s, uint16_t len);

#if 0
__STATIC_INLINE void acquire(void) {
  char buf[80];
  static unsigned row;
  int len = snprintf(buf, sizeof(buf), "%u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u", 
    acq_data[row][0], acq_data[row][1], acq_data[row][2],
    acq_data[row][3], acq_data[row][4], acq_data[row][5],
    acq_data[row][6], acq_data[row][7], acq_data[row][8],
    acq_data[row][9], acq_data[row][10], acq_data[row][11]
  );

  if (++row == 20) {
    row = 0;
  }

  ble_output((unsigned char*)buf, len);
  
}
#else

__STATIC_INLINE uint16_t swap_byte(uint16_t w) {
  union {
    uint16_t w;
    char c[2];
  } u;

  u.c[0] = w >> 8;
  u.c[1] = w;
  return u.w;
}

__STATIC_INLINE void insert_w(uint8_t *s, uint16_t w) {
  uint16_t *p = (uint16_t*)s;
  *p = w;
}

__STATIC_INLINE void fill_string(uint8_t *s, uint16_t w, uint8_t len) {
  //uint8_t *c = (uint8_t*)&w;
  insert_w(s, 0xFFFF);
  for (int i = 0; i < len; i++) {
    s += 2;
    insert_w(s, w);
  }
}


__STATIC_INLINE void acquire(void) {
  #if 0
  ble_output((unsigned char*)acq_data, 244);
  #else

  //#define BUF_SIZE 244
  #define BUF_SIZE 228

  static int m;
  unsigned char buf[BUF_SIZE] = {0};

  for (int i = 0; i < 6; i++) {
    fill_string(&buf[i * 19 * 2], acq_data[m], 18);
    if (++m == sizeof(acq_data) / 2) {
      m = 0;
    }
  }

  ble_output(buf, sizeof(buf));
  #endif
}
#endif


#ifdef __cplusplus
}
#endif

#endif
