#ifndef __DATETIME_H
#define __DATETIME_H

#ifdef __cplusplus
 extern "C" {
#endif

#define JD0 2440588

typedef struct{
  uint16_t year;
  uint8_t  month;
  uint8_t  day;
  uint8_t  wday;
  uint8_t  hour;
  uint8_t  minute;
  uint8_t  second;
} time_struct_t;


__STATIC_INLINE unsigned time_to_unixtime(time_struct_t * t) {

  /* Convert date and time to unix timestamp */ 

  uint8_t a = (14 - t->month) / 12;
  uint16_t y = t->year + 4800 - a;
  uint8_t m = t->month + (12 * a) - 3;
  uint32_t counter = t->day;

  counter += (153 * m + 2) / 5;
  counter += 365 * y;
  counter += y / 4;
  counter -= y / 100;
  counter += y / 400;
  counter -= 32045;
  counter -= JD0;
  counter *= 86400;
  counter += (t->hour * 3600);
  counter += (t->minute * 60);
  counter += (t->second);

  return counter;
}


__STATIC_INLINE void unixtime_to_time(uint32_t utime, time_struct_t * t) {

  /* Convert unix timestamp to readable date and time */ 

  uint32_t ace = (utime / 86400) + 32044 + JD0;
  uint32_t b = (4 * ace + 3) / 146097;
  ace = ace - ((146097 * b) / 4);
  uint32_t d = (4 * ace + 3) / 1461;
  ace = ace - ((1461 * d) / 4);
  uint32_t m = (5 * ace + 2) / 153;
  t->day = (uint8_t)(ace - ((153 * m + 2) / 5) + 1);
  t->month = (uint8_t) (m + 3 - (12 * (m / 10)));
  t->year = (uint16_t) (100 * b + d - 4800 + (m / 10));
  t->hour = (utime / 3600) % 24;
  t->minute = (utime / 60) % 60;
  t->second = (utime % 60);
}


#ifdef __cplusplus
}
#endif

#endif /* __DATETIME_H */
