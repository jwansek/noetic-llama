#!/usr/bin/env python3

from dataclasses import dataclass
from ollamawrapper.srv import OllamaCall, OllamaCallResponse
import inspect
import typing
import jinja2
import ollama
import rospy
import sys
import os

import capabilities
# yes we need both
from capabilities import *

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

def get_functions(ollama_output):
    return [f.strip() for f in ollama_output[8:].strip().split(";") if f != ""]

def main():
    functioncapabilities = getfunctioncapabilities()
    modelfile = functioncapabilities.to_modelfile("nexusraven:13b-v2-q3_K_S")

    client = ollama.Client(host = "http://192.168.122.1:11434")

    client.create(model = "temp", modelfile = modelfile)

    # with open("Modelfile", "r") as f:
    #    ollama.create(model = "temp", modelfile= f.read())
    ollama_output = client.generate(model='temp', prompt='What\'s the weather like in Lincoln right now? What\'s 2 + 2?', options={"stop": ["Thought:"]})
    print(ollama_output)

    for func_str in get_functions(ollama_output["response"]):
        print(func_str + ":")
        exec(func_str)

    client.delete("temp")

def handle_ollama_call(req):
    print("Recieved ollama request %s" % req.input)
    return OllamaCallResponse(1, 2, 3, 4, 5, 6)

def handle_ollama_server():
    rospy.init_node("ollama_wrapper_server")
    s = rospy.Service("ollama_wrapper", OllamaCall, handle_ollama_call)
    print("Spin")
    rospy.spin()

if __name__ == "__main__":
    handle_ollama_server()