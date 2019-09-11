# -*- coding: utf-8 -*-
import random
import logging
import os

from ask_sdk_core.utils import is_intent_name, is_request_type
from ask_sdk_model.ui import SimpleCard
from ask_sdk_core.skill_builder import SkillBuilder

from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient

AWS_IOT_ENDPOINT = os.environ['AWS_IOT_ENDPOINT']

createMQTTClient = AWSIoTMQTTClient("iRobotLambdaController")
createMQTTClient.configureEndpoint(AWS_IOT_ENDPOINT, 8883)
createMQTTClient.configureCredentials('./certs/AmazonRootCA1.crt','./certs/iRobotRaspberryPi.private.key', './certs/iRobotRaspberryPi.cert.pem')

createMQTTClient.configureAutoReconnectBackoffTime(1, 32, 20)
createMQTTClient.configureOfflinePublishQueueing(-1)  # Infinite offline Publish queueing
createMQTTClient.configureDrainingFrequency(2)  # Draining: 2 Hz
createMQTTClient.configureConnectDisconnectTimeout(10)  # 10 sec
createMQTTClient.configureMQTTOperationTimeout(5)  # 5 sec

createMQTTClient.connect()

SKILL_NAME = "The Create Robot Controller"
HELP_MESSAGE = "You can tell the robot to move a direction, like move forward, or to spin around."
HELP_REPROMPT = "What can I help you with?"
STOP_MESSAGE = "Goodbye for now!"
FALLBACK_MESSAGE = "Oops! This robot isn't smart enough, yet. Say something like move backwards."
FALLBACK_REPROMPT = 'What can I help you with?'
EXCEPTION_MESSAGE = "Sorry. This robot cannot comply"

sb = SkillBuilder()
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

''' Makes a string message MQTT friendly by making it look like JSON '''
def format_mqtt_message(msg):
    msg = '{"data": "' + msg + '"}'
    return msg

def send_mqtt_instruction(instruction):
    msg = format_mqtt_message(instruction)
    createMQTTClient.publish('/voice/drive', msg, 1)

@sb.request_handler(can_handle_func = is_intent_name("GetCrazyIntent"))
def spin_around_intent_handler(handler_input):
    speech = "Ok, I'll get crazy"
    send_mqtt_instruction("get crazy")

    handler_input.response_builder.speak(speech).set_card(SimpleCard(SKILL_NAME, speech)).set_should_end_session(False)
    return handler_input.response_builder.response

@sb.request_handler(can_handle_func = is_intent_name("SpinAroundIntent"))
def spin_around_intent_handler(handler_input):
    speech = "Ok, spinning"
    send_mqtt_instruction("spin")

    handler_input.response_builder.speak(speech).set_card(SimpleCard(SKILL_NAME, speech)).set_should_end_session(False)
    return handler_input.response_builder.response

@sb.request_handler(can_handle_func = is_intent_name("StopMovingIntent"))
def stop_moving_intent_handler(handler_input):
    speech = "Ok, I'll stop the robot"
    send_mqtt_instruction("stop")

    handler_input.response_builder.speak(speech).set_card(SimpleCard(SKILL_NAME, speech)).set_should_end_session(False)
    return handler_input.response_builder.response


@sb.request_handler(can_handle_func = is_intent_name("MoveDirectionIntent"))
def move_direction_intent_handler(handler_input):
    print("In Move Direction Handler")
    # Parse direction from event
    direction_value = handler_input.request_envelope.request.intent.slots['direction'].resolutions.resolutions_per_authority[0].values[0].value.name
    
    print("Direction value:", direction_value)
    if direction_value.find('forw') == 0:
        direction = "forward"
        speech = "Ok, moving forward"
        send_mqtt_instruction(direction)
    elif direction_value.find('back') == 0:
        direction = "back"
        speech = "Ok, moving backwards"
        send_mqtt_instruction(direction)
    else:
        direction = False
        speech = "Hmm. Please ask me to move only forward, backwards, or to spin."

    print("Send to MQTT")

    handler_input.response_builder.speak(speech).set_card(SimpleCard(SKILL_NAME, speech)).set_should_end_session(False)
    return handler_input.response_builder.response

@sb.request_handler(can_handle_func=is_request_type("LaunchRequest"))
def launch_request_handler(handler_input):
    speech = "Hi! I'm your friendly robot! You can give me commands like move forward, move backwards, or spin around!"
   
    handler_input.response_builder.speak(speech).set_card(SimpleCard(SKILL_NAME, speech)).set_should_end_session(False)
    return handler_input.response_builder.response

@sb.request_handler(can_handle_func = is_intent_name("HelloWorldIntent"))
def hello_world_intent_handler(handler_input):
    
    speech = "Hi! I'm your friendly robot! You can give me commands like move forward, move backwards, or spin around!"
    
    handler_input.response_builder.speak(speech).set_card(SimpleCard(SKILL_NAME, speech)).set_should_end_session(False)
    return handler_input.response_builder.response

@sb.request_handler(can_handle_func = is_intent_name("AMAZON.HelpIntent"))
def help_intent_hanlder(handler_input):
    return handler_input.response_builder.speak(HELP_MESSAGE).ask(HELP_REPROMPT).set_card(SimpleCard(SKILL_NAME, HELP_MESSAGE)).response

@sb.request_handler(can_handle_func = (is_intent_name("AMAZON.CancelIntent") or is_intent_name("AMAZON.StopIntent")))
def cancel_or_stop_intent_handler(handler_input):
    return handler_input.response_builder.speak(STOP_MESSAGE).response

@sb.request_handler(can_handle_func = is_intent_name("AMAZON.FallbackIntent"))
def fallback_intent_handler(handler_input):
    return handler_input.response_builder.speak(FALLBACK_MESSAGE).ask(FALLBACK_REPROMPT).set_card(SimpleCard(SKILL_NAME, FALLBACK_MESSAGE)).response

@sb.request_handler(can_handle_func = is_request_type("SessionEndedRequest"))
def session_ended_request(handler_input):
    logger.info("In SessionEndedRequestHandler")
    logger.info("Session ended reason: {}".format(handler_input.request_envelope.request.reason))
    return handler_input.response_builder.response

@sb.exception_handler(can_handle_func = lambda i, e: 'AskSdk' in e.__class__.__name__)
def ask_exception_intent_handler(handler_input, exception):
    return handler_input.response_builder.speak(EXCEPTION_MESSAGE).ask(HELP_REPROMPT).response

@sb.global_request_interceptor()
def request_logger(handler_input):
    print("Request received: {}".format(handler_input.request_envelope.request))

# Handler name that is used on AWS lambda
lambda_handler = sb.lambda_handler()