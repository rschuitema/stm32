/* copyright rschuitema 2018 */

/* includes */
#include <stdbool.h>
#include <string.h>
#include "cmsis_os.h"
#include "hilt_message.h"
#include "service_identifiers.h"
#include "action_service.h"

/* local constants */
#define SERVICE_QUEUE_MAP_SIZE (5)

/* local type definitions */

/* local function declarations */
static bool is_valid_service(uint32_t service);
static void initialize_service_queue_map(osMessageQId* map);

/* private variables */
extern osMessageQId action_service_input_q;
extern osMessageQId action_service_output_q;
extern osPoolId action_service_input_pool;
extern osPoolId action_service_output_pool;

extern osMessageQId gpio_service_input_q;
extern osMessageQId i2c_service_input_q;

static osMessageQId service_queue_map [SERVICE_QUEUE_MAP_SIZE];

/* public function implementations */
void action_service_task(const void * argument)
{
    osEvent event;
    hilt_message_t* msg;

    initialize_service_queue_map(service_queue_map);

    for(;;)
    {
        event = osMessageGet(action_service_input_q, 10);
    	msg = event.value.p;

        if (event.status == osEventMessage)
        {
			if (is_valid_service(msg->service))
			{
				// copy message and relay it
				hilt_message_t* relay_msg = osPoolAlloc(action_service_input_pool);
				relay_msg->id = msg->id;
				relay_msg->service = msg->service;
				relay_msg->action = msg->action;
				relay_msg->length = msg->length;
				relay_msg->type = msg->type;
				memcpy(relay_msg->data, msg->data, 20);

				osStatus status = osMessagePut(service_queue_map[msg->service], (uint32_t)relay_msg, 10);
				if (status == osOK)
				{
					msg->id += 1;
				}
			}
			else
			{
				// error_message = createResponseMessage(msg->id, unknown_service)
				// osMessagePut(action_service_output_q, error_msg, 10)
			}

			osPoolFree(action_service_input_pool, msg);
        }
    }
}

/* local function implementations */
static bool is_valid_service(uint32_t service)
{
    bool valid = false;

    if (service < SERVICE_QUEUE_MAP_SIZE)
    {
    	if (service_queue_map[service])
    	{
    		valid = true;
    	}
    }

    return valid;
}

static void initialize_service_queue_map(osMessageQId* map)
{
	map[0] = NULL;
	map[GPIO_SERVICE] = gpio_service_input_q;
	map[I2C_SERVICE] = i2c_service_input_q;
	map[3] = NULL;
	map[4] = NULL;
}
