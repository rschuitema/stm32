/* copyright rschuitema 2018 */

/* includes */
#include "cmsis_os.h"
#include "hilt_message.h"
#include "service_identifiers.h"
#include "response_service.h"

/* local constants */

/* local type definitions */

/* local function declarations */

/* private variables */
extern osMessageQId response_service_input_q;
extern osMessageQId response_service_output_q;
extern osPoolId response_service_input_pool;
extern osPoolId response_service_output_pool;

/* public function implementations */
void response_service_task(const void * argument)
{
    osEvent event;
    hilt_message_t* msg;

    for(;;)
    {
        event = osMessageGet(response_service_input_q, 10);
    	msg = event.value.p;

        if (event.status == osEventMessage)
        {
			osStatus status = osMessagePut(response_service_output_q, (uint32_t)msg, 10);
        }
    }
}

/* local function implementations */
