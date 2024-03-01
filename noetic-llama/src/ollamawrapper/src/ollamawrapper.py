#!/usr/bin/env python3

from dataclasses import dataclass
from ollamamessages.srv import OllamaCall, OllamaCallResponse
import inspect
import typing
import jinja2
import ollama
import rospy
import time
import sys
import os

ollama_api_url = rospy.get_param("/stt/ollama_api_url", "127.0.0.1:11434")
base_ollama_model = rospy.get_param("/stt/ollama_base_model", "nexusraven:13b-v2-q2_K")

def handle_ollama_call(req):
    print("Recieved ollama request '%s'" % req.input)
    o = main(req.input)
    # print(o.keys())
    return OllamaCallResponse(
        o["total_duration"],
        o["load_duration"],
        o["prompt_eval_duration"],
        o["eval_count"],
        o["eval_duration"]
    )

rospy.init_node("ollama_wrapper_server")

print("hello?")
import capabilities
from capabilities import *
s = rospy.Service("/stt/ollamacall", OllamaCall, handle_ollama_call)
print("spin")
rospy.spin()

def getfunctioncapabilities():
    functioncapabilities = FunctionCapablilites()

    # not very complicated inspection... the library basically must
    # consist only of multiple modules each with multiple functions,
    # no classes or submodules will be dealt with
    for modulename, module in inspect.getmembers(capabilities):
                                        #  \/ horrible
        if inspect.ismodule(module) and 'capabilities' in inspect.getfile(module):
            for functionname, function in inspect.getmembers(module):
                if inspect.isfunction(function):
                    # print(functionname, function)
                    docstring = inspect.getdoc(function)
                    # only very simple arguments are fetched, i.e. no **kwargs, or default arguments
                    # will be dealt with
                    argnames = inspect.getfullargspec(function).args
                    functioncapabilities.append(FunctionCapability(modulename, module, functionname, function, docstring, argnames))

    return functioncapabilities

@dataclass
class FunctionCapability:
    modulename: str
    module: None
    functioname: str
    function: None
    docstring: str
    argnames: list

class FunctionCapablilites(list):
    def to_modelfile(self, model):
        environment = jinja2.Environment(loader = jinja2.FileSystemLoader(os.path.dirname(__file__)))
        template = environment.get_template("Modelfile.jinja2")

        return template.render(functioncapabilities = self, model = model)

def get_functions(ollama_output):
    return [f.strip() for f in ollama_output[8:].strip().split(";") if f != ""]

MODEL_NAME = "noeticllama:" + str(int(time.time()))
client = ollama.Client(host = "http://%s" % ollama_api_url)

for mn in [m["name"] for m in client.list()["models"]]:
    if mn.startswith("noeticllama"):
        client.delete(mn)

functioncapabilities = getfunctioncapabilities()
modelfile = functioncapabilities.to_modelfile(base_ollama_model)
client.create(model = MODEL_NAME, modelfile = modelfile)

def main(prompt):
    # with open("Modelfile", "r") as f:
    #    ollama.create(model = "temp", modelfile= f.read())
    ollama_output = client.generate(
        model = MODEL_NAME, 
        prompt = prompt, 
        options = {"stop": ["Thought:"]},
        keep_alive = "30m"
    )
    #print(ollama_output)

    for func_str in get_functions(ollama_output["response"]):
        rospy.loginfo("Generated function: " + func_str + ":")
        exec(func_str)

    return ollama_output