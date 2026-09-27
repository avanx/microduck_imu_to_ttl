#ifndef LSM6_H
#define LSM6_H

/* Returns 0 when WHO_AM_I matches and SFLP is running. */
int lsm6_init(void);
void lsm6_poll(void);

#endif
