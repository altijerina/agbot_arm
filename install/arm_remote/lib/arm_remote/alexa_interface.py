#!/usr/bin/env python3
from flask import Flask
from ask_sdk_core.skill_builder import SkillBuilder
from flask_ask_sdk.skill_adapter import SkillAdapter

from ask_sdk_core.dispatch_components import AbstractRequestHandler
from ask_sdk_core.utils import is_request_type, is_intent_name
from ask_sdk_core.handler_input import HandlerInput
from ask_sdk_model import Response
from ask_sdk_model.ui import SimpleCard

from ask_sdk_core.dispatch_components import AbstractExceptionHandler

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from arm_msgs.action import ArmTask
import threading

threading.Thread(target=lambda: rclpy.init()).start()
action_client = ActionClient(Node("agbot_alexa_interface"), ArmTask, "agbot_task_server" )
app = Flask(__name__)

class LaunchRequestHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_request_type("LaunchRequest")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "Howdy.  How can I help?"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Hello World", speech_text)).set_should_end_session(
            False)
        
        goal = ArmTask.Goal()
        goal.task_number = 0
        action_client.send_goal_async(goal)            
        return handler_input.response_builder.response

class PositionZeroHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("position_zero")(handler_input)
    
    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "Ok, I'm going home."

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("PositionZero", speech_text)).set_should_end_session(
            False)
            
        goal = ArmTask.Goal()
        goal.task_number = 0
        action_client.send_goal_async(goal)            
        return handler_input.response_builder.response

class PositionOneHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("position_one")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "Ok, I'm going to Position One."

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("PositionOne", speech_text)).set_should_end_session(
            False)
            
        goal = ArmTask.Goal()
        goal.task_number = 1
        action_client.send_goal_async(goal)            
        return handler_input.response_builder.response    

class PositionTwoHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("position_two")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "Ok, I'm going to Position Two."

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("PositionTwo", speech_text)).set_should_end_session(
            False)
            
        goal = ArmTask.Goal()
        goal.task_number = 2
        action_client.send_goal_async(goal)            
        return handler_input.response_builder.response     
    
class PositionThreeHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("position_three")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "Ok, I'm going to Position Three."

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("PositionThree", speech_text)).set_should_end_session(
            False)
            
        goal = ArmTask.Goal()
        goal.task_number = 3
        action_client.send_goal_async(goal)            
        return handler_input.response_builder.response   

class PositionFourHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("position_four")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "Ok, I'm going to Position Four."

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("PositionFour", speech_text)).set_should_end_session(
            False)
            
        goal = ArmTask.Goal()
        goal.task_number = 4
        action_client.send_goal_async(goal)            
        return handler_input.response_builder.response 

class PositionFiveHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("position_five")(handler_input)
    
    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "Ok, I'm going to Position Five."

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("PositionFive", speech_text)).set_should_end_session(
            False)
            
        goal = ArmTask.Goal()
        goal.task_number = 5
        action_client.send_goal_async(goal)            
        return handler_input.response_builder.response 

class SleepIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("SleepIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "Ok, I'm going to sleep."

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("sleep", speech_text)).set_should_end_session(
            True)
            
        goal = ArmTask.Goal()
        goal.task_number = 6
        action_client.send_goal_async(goal)            
        return handler_input.response_builder.response 
    
class AllExceptionHandler(AbstractExceptionHandler):
    def can_handle(self, handler_input, exception):
        # type: (HandlerInput, Exception) -> bool
        return True

    def handle(self, handler_input, exception):
        # type: (HandlerInput, Exception) -> Response
        # Log the exception in CloudWatch Logs
        print(exception)

        speech = "Sorry, I didn't understand. Can you please repeat your comment!!"
        handler_input.response_builder.speak(speech).ask(speech)
        return handler_input.response_builder.response   
    

skill_builder = SkillBuilder()
skill_builder.add_request_handler(LaunchRequestHandler())
skill_builder.add_request_handler(PositionZeroHandler())
skill_builder.add_request_handler(PositionOneHandler())
skill_builder.add_request_handler(PositionTwoHandler())
skill_builder.add_request_handler(PositionThreeHandler())
skill_builder.add_request_handler(PositionFourHandler())
skill_builder.add_request_handler(PositionFiveHandler())
skill_builder.add_request_handler(SleepIntentHandler())
skill_builder.add_exception_handler(AllExceptionHandler())
# Register your intent handlers to the skill_builder object

skill_adapter = SkillAdapter(
    skill=skill_builder.create(), skill_id="amzn1.ask.skill.82c5467b-1fed-4ae3-9a78-44aa2263b6b8", app=app)

@app.route("/")
def invoke_skill():
    return skill_adapter.dispatch_request()


skill_adapter.register(app=app, route="/")

if __name__ == '__main__':
    app.run()