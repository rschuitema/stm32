/* copyright rschuitema 2018 */

/* includes */
#include "cmsis_os.h"
#include "hilt_message.h"
#include "service_identifiers.h"
#include "gpio_service.h"

/* local constants */

/* local type definitions */

/* local function declarations */

/* private variables */
extern osMessageQId gpio_service_input_q;
extern osMessageQId gpio_service_output_q;
extern osPoolId gpio_service_input_pool;
extern osPoolId gpio_service_output_pool;

/* public function implementations */
void gpio_service_task(const void * argument)
{
    osEvent event;
    hilt_message_t* msg;

    for(;;)
    {
        event = osMessageGet(gpio_service_input_q, 100);
    	msg = event.value.p;

        if (event.status == osEventMessage)
        {
        	msg->data[4] = 9;
        }

        osPoolFree(gpio_service_input_pool, msg);

        osDelay(1100);
    }
}

/* local function implementations */
