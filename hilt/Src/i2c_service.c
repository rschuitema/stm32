/* copyright rschuitema 2018 */

/* includes */
#include "cmsis_os.h"
#include "hilt_message.h"
#include "service_identifiers.h"
#include "i2c_service.h"

/* local constants */

/* local type definitions */

/* local function declarations */

/* private variables */
extern osMessageQId i2c_service_input_q;
extern osMessageQId i2c_service_output_q;
extern osPoolId i2c_service_input_pool;
extern osPoolId i2c_servcice_output_pool;

/* public function implementations */
void i2c_service_task(const void * argument)
{
    osEvent event;
    hilt_message_t* msg;

    for(;;)
    {
        event = osMessageGet(i2c_service_input_q, 100);
    	msg = event.value.p;

        if (event.status == osEventMessage)
        {
        	msg->data[4] = 4;
        }

        osPoolFree(i2c_service_input_pool, msg);
        osDelay(1100);
    }
}

/* local function implementations */
