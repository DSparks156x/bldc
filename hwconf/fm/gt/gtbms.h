#ifndef GTBMS_H_
#define GTBMS_H_

#include <stdint.h>
#include <stdbool.h>

// Functions
void gtbms_init(void);
bool gtbms_is_running(void);
float gtbms_get_voltage(void);
float gtbms_get_soc(void);
float gtbms_get_current(void);
float gtbms_get_temp(void);
float gtbms_get_cell_voltage(int cell);
int gtbms_get_cell_num(void);

#endif /* GTBMS_H_ */
