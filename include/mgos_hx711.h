/*
Copyright (c) 2024 Aleksey Kotelnikov <kotelnikov.www@gmail.com>

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/
#ifndef MGOS_HX_711_LIB_C
#define MGOS_HX_711_LIB_C

#include "mgos.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

void mgos_hx711_power_up(void);
void mgos_hx711_power_down(void);
void mgos_hx711_start_polling(void);
int32_t mgos_hx711_get_raw_data(void);
int32_t mgos_hx711_get_value(void);
double mgos_hx711_get_units(void);
void mgos_hx711_set_offset(int32_t offset);
void mgos_hx711_set_scale(float scale);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif