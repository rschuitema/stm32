/* copyright rschuitema 2018 */

/* includes */
#include "cmsis_os.h"
#include "stm32f7xx_hal.h"
#include "cwpack.h"
#include "hilt_message.h"
#include "service_identifiers.h"
#include "gpio_service.h"

/* local constants */
#define NR_GPIO_PINS (16)
#define GPIO_INVALID_PIN (0xFFFF)
#define GPIO_ERROR_INVALID_PIN (1)
#define GPIO_ERROR_INVALID_ACTION (2)

/* local type definitions */
typedef enum
{
	GPIO_ACTION_SET_PIN = 0,
	GPIO_ACTION_RESET_PIN,
	GPIO_ACTION_READ_PIN,
	GPIO_ACTION_CONFIGURE_PIN
}gpio_action_t;

typedef enum
{
	GPIO_PINMODE_INPUT = 0,
	GPIO_PINMODE_OUTPUT_PUSH_PULL,
	GPIO_PINMODE_OUTPUT_OPEN_DRAIN,
}gpio_pinmode_t;

typedef enum
{
	GPIO_PULLUP_NOPULL = 0,
	GPIO_PULLUP_PULLUP,
	GPIO_PULLUP_PULLDOWN
}gpio_pullup_t;

typedef struct
{
    gpio_pinmode_t mode;
    gpio_pullup_t  pull;
}gpio_config_t;

typedef void (*gpio_handler_t)(hilt_message_t* msg);

/* local function declarations */

static void gpio_set_pin(hilt_message_t* msg);
static void gpio_reset_pin(hilt_message_t* msg);
static void gpio_read_pin(hilt_message_t* msg);
static void gpio_configure_pin(hilt_message_t* msg);

static void gpio_handle_message(hilt_message_t* msg);
static uint16_t gpio_convert_to_pin(uint16_t pin);
static void gpio_send_error_response(uint32_t id, uint8_t action, uint8_t error);

/* private variables */
extern osMessageQId gpio_service_input_q;
extern osMessageQId gpio_service_output_q;
extern osPoolId gpio_service_input_pool;
extern osPoolId gpio_service_output_pool;

static gpio_handler_t handlers[] =
{
	&gpio_set_pin,
	&gpio_reset_pin,
	&gpio_read_pin,
	&gpio_configure_pin
};
static uint16_t gpio_pin_map [NR_GPIO_PINS] =
{
    GPIO_PIN_0,
	GPIO_PIN_1,
	GPIO_PIN_2,
	GPIO_PIN_3,
	GPIO_PIN_4,
	GPIO_PIN_5,
	GPIO_PIN_6,
	GPIO_PIN_7,
	GPIO_PIN_8,
	GPIO_PIN_9,
	GPIO_PIN_10,
	GPIO_PIN_11,
	GPIO_PIN_12,
	GPIO_PIN_13,
	GPIO_PIN_14,
	GPIO_PIN_15
};

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
        	gpio_handle_message(msg);
        }

        osPoolFree(gpio_service_input_pool, msg);

        osDelay(1100);
    }
}

/* local function implementations */
static void gpio_handle_message(hilt_message_t* msg)
{
	if (msg->action < 4)
	{
	    (*handlers[msg->action])(msg);
	}
	else
	{
		// send response invalid action
		gpio_send_error_response(msg->id, msg->action, GPIO_ERROR_INVALID_ACTION);
	}
}

static void gpio_set_pin(hilt_message_t* msg)
{
	cw_unpack_context uc;
	cw_unpack_context_init (&uc, msg->data, msg->length, NULL);

	cw_unpack_next(&uc);
	uint16_t pin = uc.item.as.u64;
	uint16_t gpio_pin = gpio_convert_to_pin(pin);

	if (gpio_pin != GPIO_INVALID_PIN)
	{
		HAL_GPIO_WritePin(GPIOB, gpio_pin, GPIO_PIN_SET);
	}
	else
	{
		// send error response invalid pin
		gpio_send_error_response(msg->id, msg->action, GPIO_ERROR_INVALID_PIN);
	}
}

static void gpio_reset_pin(hilt_message_t* msg)
{
	cw_unpack_context uc;
	cw_unpack_context_init (&uc, msg->data, msg->length, NULL);

	cw_unpack_next(&uc);
	uint16_t pin = uc.item.as.u64;
	uint16_t gpio_pin = gpio_convert_to_pin(pin);

	if (gpio_pin != GPIO_INVALID_PIN)
	{
		HAL_GPIO_WritePin(GPIOB, gpio_pin, GPIO_PIN_RESET);
	}
	else
	{
		// send error response invalid pin
		gpio_send_error_response(msg->id, msg->action, GPIO_ERROR_INVALID_PIN);
	}
}

static void gpio_read_pin(hilt_message_t* msg)
{
	cw_unpack_context uc;
	cw_unpack_context_init (&uc, msg->data, msg->length, NULL);

	cw_unpack_next(&uc);
	uint16_t pin = uc.item.as.u64;
	uint16_t gpio_pin = gpio_convert_to_pin(pin);

	if (gpio_pin != GPIO_INVALID_PIN)
	{
		GPIO_PinState pinstate;

		pinstate = HAL_GPIO_ReadPin(GPIOB, gpio_pin);

		// send response
	}
	else
	{
		// send error response invalid pin
		gpio_send_error_response(msg->id, msg->action, GPIO_ERROR_INVALID_PIN);
	}
}

static void gpio_configure_pin(hilt_message_t* msg)
{
	cw_unpack_context uc;
	cw_unpack_context_init (&uc, msg->data, msg->length, NULL);

	cw_unpack_next(&uc);
	uint16_t pin = uc.item.as.u64;
	uint16_t gpio_pin = gpio_convert_to_pin(pin);

	if (gpio_pin != GPIO_INVALID_PIN)
	{
	}
	else
	{
		// send error response invalid pin
		gpio_send_error_response(msg->id, msg->action, GPIO_ERROR_INVALID_PIN);
	}
}

static void gpio_send_error_response(uint32_t id, uint8_t action, uint8_t error)
{
	hilt_message_t* error_message = osPoolAlloc(gpio_service_output_pool);

	error_message->id = id;
	error_message->type = 1; // response
	error_message->action = action;
	error_message->service = GPIO_SERVICE;
	error_message->length = 1;
	error_message->data[0] = error;

	osStatus status = osMessagePut(gpio_service_output_q, (uint32_t)error_message, 10);
}

static uint16_t gpio_convert_to_pin(uint16_t pin)
{
	if (pin < 16)
	{
		return gpio_pin_map[pin];
	}
	else
	{
		return GPIO_INVALID_PIN;
	}
}
