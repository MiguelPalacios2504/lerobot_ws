#!/usr/bin/env python3
from flask import Flask
from ask_sdk_core.skill_builder import SkillBuilder
from flask_ask_sdk.skill_adapter import SkillAdapter
from ask_sdk_core.utils import is_request_type, is_intent_name
from ask_sdk_core.handler_input import HandlerInput
from ask_sdk_model.ui import SimpleCard
from ask_sdk_core.dispatch_components import AbstractRequestHandler, AbstractExceptionHandler

import logging, threading
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from lerobot_msgs.action import LerobotTask

log = logging.getLogger(__name__)
app = Flask(__name__)

# --- ROS2 bien inicializado y spinned en background ---
rclpy.init()
node = rclpy.create_node('alexa_interface')
executor = SingleThreadedExecutor()
executor.add_node(node)
threading.Thread(target=executor.spin, daemon=True).start()

action_client = ActionClient(node, LerobotTask, "task_server")

def send_goal(task_number: int):
    # No bloquees mucho a Alexa; espera breve por el servidor
    if not action_client.server_is_ready():
        action_client.wait_for_server(timeout_sec=0.5)
    goal = LerobotTask.Goal()
    goal.task_number = task_number
    action_client.send_goal_async(goal)

class LaunchRequestHandler(AbstractRequestHandler):
    def can_handle(self, handler_input: HandlerInput) -> bool:
        return is_request_type("LaunchRequest")(handler_input)

    def handle(self, handler_input: HandlerInput):
        speech = "Hi, how can we help?"
        handler_input.response_builder.speak(speech)\
            .set_card(SimpleCard("Online", speech))\
            .set_should_end_session(False)
        send_goal(0)
        return handler_input.response_builder.response

class PickIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input): 
        return is_intent_name("Pickintent")(handler_input)

    def handle(self, handler_input):
        speech = "Ok, I'm moving"
        handler_input.response_builder.speak(speech)\
            .set_card(SimpleCard("Pick", speech))\
            .set_should_end_session(True)
        send_goal(1)
        return handler_input.response_builder.response

class SleepIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        return is_intent_name("Sleepintent")(handler_input)

    def handle(self, handler_input):
        speech = "Ok, see you later"
        handler_input.response_builder.speak(speech)\
            .set_card(SimpleCard("Sleep", speech))\
            .set_should_end_session(True)
        send_goal(0)
        return handler_input.response_builder.response

class WakeIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        return is_intent_name("lerobothello")(handler_input)

    def handle(self, handler_input):
        speech = "Hi, I am ready"
        handler_input.response_builder.speak(speech)\
            .set_card(SimpleCard("Wake", speech))\
            .set_should_end_session(True)
        send_goal(2)
        return handler_input.response_builder.response

class AllExceptionHandler(AbstractExceptionHandler):
    def can_handle(self, handler_input, exception): 
        return True
    def handle(self, handler_input, exception):
        log.exception("Unhandled exception in skill", exc_info=exception)
        speech = "Hmm, I don't know that. Can you please say it again?"
        handler_input.response_builder.speak(speech).ask(speech)
        return handler_input.response_builder.response

sb = SkillBuilder()
sb.add_request_handler(LaunchRequestHandler())
sb.add_request_handler(PickIntentHandler())
sb.add_request_handler(SleepIntentHandler())
sb.add_request_handler(WakeIntentHandler())
sb.add_exception_handler(AllExceptionHandler())

skill_adapter = SkillAdapter(
    skill=sb.create(),
    skill_id="amzn1.ask.skill.c35e78bc-e271-4242-a131-4b3f812e393d",
    app=app,
)
skill_adapter.register(app=app, route="/")

if __name__ == "__main__":
    # Si prefieres, deja 127.0.0.1; con ngrok también vale
    app.run()
