/*
 * lps25hb.h
 *
 *  Created on: Nov 17, 2017
 *      Author: kerhoas
 */

#ifndef INC_LPS25HB_H_
#define INC_LPS25HB_H_


#include "main.h"

uint8_t lps25hb_whoAmI();
void lps25hb_setup();
int lps25hb_getPressure(float*);
int lps25hb_getTemperature(float*);


#endif /* INC_LPS25HB_H_ */
