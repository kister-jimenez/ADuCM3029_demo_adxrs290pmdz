/*****************************************************************************
 * ADuCM3029_demo_adxrs290pmdz.c
 *****************************************************************************/

#include <sys/platform.h>
#include "adi_initialize.h"
#include "adxrs290_pmdz.h"
#include "error.h"

int main(int argc, char *argv[])
{
	/**
	 * Initialize managed drivers and/or services that have been added to 
	 * the project.
	 * @return zero on success 
	 */
	adi_initComponents();
	
	struct adxrs290_pmdz_init_param adxrs290_pmdz_init;
	struct adxrs290_pmdz_dev *adxrs290_pmdz_dev;
	int32_t ret;

	adxrs290_pmdz_get_config(&adxrs290_pmdz_init);

	ret = adxrs290_pmdz_setup(&adxrs290_pmdz_dev, &adxrs290_pmdz_init);
	if(ret != SUCCESS)
		return ret;

	while(1) {
		ret = adxrs290_pmdz_process(adxrs290_pmdz_dev);
		if(ret != SUCCESS)
			return ret;
	}

	return ret;
}

