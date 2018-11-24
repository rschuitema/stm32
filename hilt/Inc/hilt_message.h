/* copyright rschuitema 2018 */

/*
 * Messages can be exchanged between a client and hilt.
 * There are 3 types of messages:
 * - actions
 * - responses
 * - events
 *
 * The actions requested from hilt are always acknowledged with the same id as in the action.
 * When an actions requires a response then the response will contain the same id as in the action
 * The id is meant to match the actions and responses.
 * HILT can also sent events autonomously.
 *
 * This is shown in the following diagram.
 *
 *    client                  hilt
 *      |                      |
 *      |    action (id=3)     |
 *      |--------------------->|
 *      |                      |
 *      |    ack (id=3)        |
 *      |<---------------------|
 *      |                      |
 *      |                      |
 *      |                      |
 *      |    response (id=3)   |
 *      |<---------------------|
 *      |                      |
 *      |                      |
 *      |                      |
 *      |                      |
 *      |    event (id=10)     |
 *      |<---------------------|
 *      |                      |
 */

#ifndef HILT_MESSAGE_H_
#define HILT_MESSAGE_H_

#ifdef __cplusplus
extern "C" {
#endif

/* includes */

/* constants */

/* type definitions */

typedef struct
{
	uint32_t id;			/** Identifier of the message. used for matching command and responses.*/
	uint8_t type;           /** Type of message, action, response, event, acknowledge.*/
	uint8_t service;        /** The service that needs to handle the message.*/
	uint8_t action;         /** The action required from the service.*/
	uint8_t length;         /** The length of the message payload.*/
	uint8_t data[20];       /** The payload of the message.*/
} hilt_message_t;

/* interface declarations */

#ifdef __cplusplus
}
#endif

#endif /* HILT_MESSAGE_H_ */
